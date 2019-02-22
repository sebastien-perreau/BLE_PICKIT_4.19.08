#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"

#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_gpio.h"
#include "ble_vsd.h"
#include "ble_pickit_board.h"
#include "ble_pickit_service.h"


#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


BLE_PICKIT_DEF(ble_pickit, "no name", "4.19.08");
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */
BLE_PICKIT_SERVICE_DEF(m_msg);
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */
extern uint8_t __data_start__;


static void log_init(void);
static void timers_init(void);
static void rtc_init(void);
static void uart_init(void);
static void power_management_init(bool pwr_mngt_enable);
static void ble_stack_init(void);
static void gap_init(void);
static void gatt_init(void);
static void advertising_init(void);
static void services_init(void);
static void conn_params_init(void);
static void peer_manager_init(void);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
	ret_code_t err_code;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        	err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
			APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void conn_params_evt_handler(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void adv_err_evt_handler(uint32_t nrf_error)
{
	NRF_LOG_INFO("on_adv_err: %d", nrf_error);
}

static void adv_evt_handler(ble_adv_evt_t ble_adv_evt)
{
	ret_code_t err_code;

	switch (ble_adv_evt)
	{
		case BLE_ADV_EVT_IDLE:                /**< Idle; no connectable advertising is ongoing.*/
			NRF_LOG_INFO("Idle advertising");
			ble_pickit.status.is_in_advertising_mode = false;
			err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
			APP_ERROR_CHECK(err_code);
			break;
		case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:  /**< Direct advertising mode has started. */
			break;
		case BLE_ADV_EVT_DIRECTED:            /**< Directed advertising (low duty cycle) has started. */
			break;
		case BLE_ADV_EVT_FAST:                /**< Fast advertising mode has started. */
			ble_pickit.status.is_in_advertising_mode = true;
			NRF_LOG_INFO("Fast advertising");
			break;
		case BLE_ADV_EVT_SLOW:                /**< Slow advertising mode has started. */
			NRF_LOG_INFO("Slow advertising");
			break;
		case BLE_ADV_EVT_FAST_WHITELIST:      /**< Fast advertising mode using the whitelist has started. */
			break;
		case BLE_ADV_EVT_SLOW_WHITELIST:      /**< Slow advertising mode using the whitelist has started. */
			break;
		case BLE_ADV_EVT_WHITELIST_REQUEST:   /**< Request a whitelist from the main application. For whitelist advertising to work, the whitelist must be set when this event occurs. */
			break;
		case BLE_ADV_EVT_PEER_ADDR_REQUEST:
			break;
		default:
			break;
	}
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;
    ble_gap_conn_params_t _t_conn_params = p_ble_evt->evt.gap_evt.params.connected.conn_params;
    ble_gap_conn_params_t _t_conn_params_update_request = p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("DISCONNECTED.");
            ble_pickit.status.is_connected_to_a_central = false;
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("CONNECTED.");
            NRF_LOG_INFO("	min_conn_param: " NRF_LOG_FLOAT_MARKER " ms", NRF_LOG_FLOAT((float)(_t_conn_params.min_conn_interval)*UNIT_1_25_MS/1000));
			NRF_LOG_INFO("	max_conn_param: " NRF_LOG_FLOAT_MARKER " ms", NRF_LOG_FLOAT((float)(_t_conn_params.max_conn_interval)*UNIT_1_25_MS/1000));
			NRF_LOG_INFO("	slave_latency: %d", _t_conn_params.slave_latency);
			NRF_LOG_INFO("	timeout: %d ms", (_t_conn_params.conn_sup_timeout*UNIT_10_MS/1000));
			ble_pickit.status.is_connected_to_a_central = true;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            err_code = sd_ble_gap_conn_param_update(m_conn_handle, &ble_pickit.params.preferred_gap_params.conn_params);
			APP_ERROR_CHECK(err_code);
			err_code = sd_ble_gap_phy_update(m_conn_handle, &ble_pickit.params.preferred_gap_params.phys_params);
			APP_ERROR_CHECK(err_code);
            err_code = sd_ble_gap_data_length_update(m_conn_handle, &ble_pickit.params.preferred_gap_params.mtu_size_params, NULL);
			APP_ERROR_CHECK(err_code);
            break;

		case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
			NRF_LOG_INFO("CONN PARAMS UPDATE REQUEST (accept parameters requested by the peer): ");
			NRF_LOG_INFO("	min_conn_param: " NRF_LOG_FLOAT_MARKER " ms", NRF_LOG_FLOAT((float)(p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params.min_conn_interval)*UNIT_1_25_MS/1000));
			NRF_LOG_INFO("	max_conn_param: " NRF_LOG_FLOAT_MARKER " ms", NRF_LOG_FLOAT((float)(p_ble_evt->evt.gap_evt.params.conn_param_update_request.conn_params.max_conn_interval)*UNIT_1_25_MS/1000));
			err_code = sd_ble_gap_conn_param_update(p_ble_evt->evt.gap_evt.conn_handle, &_t_conn_params_update_request);
			APP_ERROR_CHECK(err_code);
			break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
			NRF_LOG_INFO("CONN PARAMS UPDATED: ");
			NRF_LOG_INFO("	min_conn_param: " NRF_LOG_FLOAT_MARKER " ms", NRF_LOG_FLOAT((float)(p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval)*UNIT_1_25_MS/1000));
			NRF_LOG_INFO("	max_conn_param: " NRF_LOG_FLOAT_MARKER " ms", NRF_LOG_FLOAT((float)(p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval)*UNIT_1_25_MS/1000));
			NRF_LOG_INFO("	slave_latency: %d", p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency);
			NRF_LOG_INFO("	timeout: %d ms", (p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.conn_sup_timeout*UNIT_10_MS/1000));
			ble_pickit.params.current_gap_params.conn_params.min_conn_interval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval;
			ble_pickit.params.current_gap_params.conn_params.max_conn_interval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval;
			ble_pickit.params.current_gap_params.conn_params.slave_latency = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency;
			ble_pickit.params.current_gap_params.conn_params.conn_sup_timeout = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.conn_sup_timeout;

			ble_pickit_parameters_notification_send();
			break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
			NRF_LOG_DEBUG("PHY UPDATE REQUEST: TX = %d / RX = %d", p_ble_evt->evt.gap_evt.params.phy_update_request.peer_preferred_phys.tx_phys, p_ble_evt->evt.gap_evt.params.phy_update_request.peer_preferred_phys.rx_phys);
			err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &(p_ble_evt->evt.gap_evt.params.phy_update_request.peer_preferred_phys));
			APP_ERROR_CHECK(err_code);
			break;

        case BLE_GAP_EVT_PHY_UPDATE:
			NRF_LOG_INFO("PHY UPDATED: TX = %d / RX = %d", p_ble_evt->evt.gap_evt.params.phy_update.tx_phy, p_ble_evt->evt.gap_evt.params.phy_update.rx_phy);
			ble_pickit.params.current_gap_params.phys_params.tx_phys = p_ble_evt->evt.gap_evt.params.phy_update.tx_phy;
			ble_pickit.params.current_gap_params.phys_params.rx_phys = p_ble_evt->evt.gap_evt.params.phy_update.rx_phy;

			ble_pickit_parameters_notification_send();
			break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
			NRF_LOG_INFO("DATA LEN UPDATE REQUEST: TX = %d / RX = %d", p_ble_evt->evt.gap_evt.params.data_length_update_request.peer_params.max_tx_octets-4, p_ble_evt->evt.gap_evt.params.data_length_update_request.peer_params.max_rx_octets-4);
			err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &p_ble_evt->evt.gap_evt.params.data_length_update_request.peer_params, NULL);
			APP_ERROR_CHECK(err_code);
			break;

        case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
			NRF_LOG_INFO("DATA LEN UPDATED: TX = %d / RX = %d", p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_octets-4, p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_octets-4);
			ble_pickit.params.current_gap_params.mtu_size_params.max_tx_octets = p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_octets-4;
			ble_pickit.params.current_gap_params.mtu_size_params.max_rx_octets = p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_octets-4;
			ble_pickit.params.current_gap_params.mtu_size_params.max_tx_time_us = p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_time_us;
			ble_pickit.params.current_gap_params.mtu_size_params.max_rx_time_us = p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_time_us;

			ble_pickit_parameters_notification_send();
			break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
        	NRF_LOG_INFO("GATTC TIMEOUT.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
        	NRF_LOG_INFO("GATTS TIMEOUT.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static void on_service_event_handler(ble_msg_t * p_msg, ble_msg_evt_t * p_evt, const uint8_t *buffer, uint8_t length)
{
//	ret_code_t err_code;

    switch(p_evt->evt_type)
    {

		case SERVICE_EVT_CONNECTED:
			break;

		case SERVICE_EVT_DISCONNECTED:
			p_msg->char_app.is_notification_enabled = false;
			p_msg->char_test.is_notification_enabled = false;
			p_msg->char_params.is_notification_enabled = false;
			break;

        case SERVICE_EVT_APP_NOTIFICATION_ENABLED:
        	p_msg->char_app.is_notification_enabled = true;
            break;

        case SERVICE_EVT_APP_NOTIFICATION_DISABLED:
        	p_msg->char_app.is_notification_enabled = false;
            break;

        case SERVICE_EVT_TEST_NOTIFICATION_ENABLED:
        	p_msg->char_test.is_notification_enabled = true;
			break;

		case SERVICE_EVT_TEST_NOTIFICATION_DISABLED:
			p_msg->char_test.is_notification_enabled = false;
			break;

		case SERVICE_EVT_PARAMS_NOTIFICATION_ENABLED:
			p_msg->char_params.is_notification_enabled = true;
			break;

		case SERVICE_EVT_PARAMS_NOTIFICATION_DISABLED:
			p_msg->char_params.is_notification_enabled = false;
			break;

        case SERVICE_EVT_APP_WRITE:
        	if ((length > 2) && (length == (buffer[1] + 2)))
			{
				memset(&ble_pickit.outgoing_uart_message, 0, sizeof(ble_serial_message_t));

				ble_pickit.outgoing_uart_message.id = buffer[0];
				ble_pickit.outgoing_uart_message.type = 'N';
				ble_pickit.outgoing_uart_message.length = buffer[1];
				for (uint8_t i = 0 ; i < buffer[1] ; i++)
				{
					ble_pickit.outgoing_uart_message.data[i] = buffer[i+2];
				}

				if (	(ble_pickit.outgoing_uart_message.id == ID_SOFTWARE_RESET) 	&& 	\
						(ble_pickit.outgoing_uart_message.type == 'N') 				&& 	\
						(ble_pickit.outgoing_uart_message.length == 1) 				&&	\
						((ble_pickit.outgoing_uart_message.data[0] == RESET_BLE_PICKIT) || (ble_pickit.outgoing_uart_message.data[0] == RESET_ALL)))
				{
					ble_pickit.flags.exec_reset = true;
				}

				ble_pickit.flags.transfer_ble_to_uart = true;
			}
        	break;

        case SERVICE_EVT_TEST_WRITE:
        	if (p_msg->char_test.is_notification_enabled && (length == 1))
        	{
        		p_msg->att_payload = ble_pickit.params.current_gap_params.mtu_size_params.max_tx_octets - 3;	// remove 3 bytes ATT header to keep only ATT Payload
        		p_msg->throughput._type = buffer[0];
        		p_msg->throughput._sm.index = (buffer[0] == 0x01) ? 2 : 0;
        		ble_pickit_throughput_notification_send(p_msg);
        	}
        	else
        	{
        		NRF_LOG_INFO("SERVICE_EVT_TEST_WRITE: ERROR");
        		NRF_LOG_INFO("	Be sure to activate NOTIFICATIONS (at least TEST 0x1502) and write a 1 byte command.");
        	}
        	break;

        case SERVICE_EVT_PARAMS_WRITE:
        	if (p_msg->char_params.is_notification_enabled)
			{
        		if ((length == 1) && (buffer[0] == 0x00))
        		{
        			// Return all parameters by notifying the client.
        			p_msg->ble_params.notify_all_parameters = true;
        			ble_pickit_parameters_notification_send();
        		}
        		else if ((length == 10) && (buffer[0] == 0x01) && (buffer[1] == 8))
        		{
					// Change preferred BLE_CONN_PARAMS
        			ble_pickit.params.preferred_gap_params.conn_params.min_conn_interval = (uint16_t) ((buffer[2] << 8) | (buffer[3] << 0));
        			ble_pickit.params.preferred_gap_params.conn_params.max_conn_interval = (uint16_t) ((buffer[4] << 8) | (buffer[5] << 0));
        			ble_pickit.params.preferred_gap_params.conn_params.slave_latency = (uint16_t) ((buffer[6] << 8) | (buffer[7] << 0));
        			ble_pickit.params.preferred_gap_params.conn_params.conn_sup_timeout = (uint16_t) ((buffer[8] << 8) | (buffer[9] << 0));

        			p_msg->ble_params.change_conn_params_request = true;
        			p_msg->ble_params.notify_all_parameters = true;
        		}
        		else if ((length == 3) && (buffer[0] == 0x02) && (buffer[1] == 1))
				{
        			// Change preferred BLE_PHY_PARAM
        			ble_pickit.params.preferred_gap_params.phys_params.tx_phys = (buffer[2] & 0x03);
        			ble_pickit.params.preferred_gap_params.phys_params.rx_phys = (buffer[2] & 0x03);

        			p_msg->ble_params.change_phy_param_request = true;
        			p_msg->ble_params.notify_all_parameters = true;
				}
        		else if ((length == 4) && (buffer[0] == 0x03) && (buffer[1] == 2))
				{
        			// Change preferred BLE_ATT_SIZE_PARAM
        			ble_pickit.params.preferred_gap_params.mtu_size_params.max_rx_octets = buffer[2] + 4;
					ble_pickit.params.preferred_gap_params.mtu_size_params.max_tx_octets = buffer[3] + 4;

					p_msg->ble_params.change_mtu_size_params_request = true;
					p_msg->ble_params.notify_all_parameters = true;
				}
			}
			else
			{
				NRF_LOG_INFO("SERVICE_EVT_PARAMS_WRITE: ERROR");
				NRF_LOG_INFO("	Be sure to activate NOTIFICATIONS (at least PARAMS 0x1503) in order to get the returned values.");
			}
        	break;

        default:
              // No implementation needed.
              break;
    }
}

static void button_event_handler(uint8_t pin_no, bool button_action)
{

	if(pin_no == BUTTON_1)
	{
		if (button_action == 1)
		{
			sd_nvic_SystemReset();
		}
		else
		{

		}
	}

	if(pin_no == BUTTON_2)
	{
		if (button_action == 1)
		{
			ble_pickit.params.leds_status_enable = !ble_pickit.params.leds_status_enable;
		}
		else
		{

		}
	}
}

void uart_event_handle(app_uart_evt_t * p_event)
{
	switch (p_event->evt_type)
	{
		case APP_UART_DATA_READY:

			break;

		case APP_UART_FIFO_ERROR:

			NRF_LOG_INFO("UART fifo error");
			break;

		case APP_UART_COMMUNICATION_ERROR:

			NRF_LOG_INFO("UART communication error");
			break;

		case APP_UART_TX_EMPTY:

			ble_pickit.uart.transmit_in_progress = false;
			break;

		default:
			break;
	}
}


/**@brief Function for application main entry.
 */
int main(void)
{
	bool pwr_mgmt_enable = false;
	ret_code_t err_code;

    // Initialize.
    log_init();
    timers_init();
    rtc_init();
	uart_init();
    power_management_init(pwr_mgmt_enable);
    board_init(button_event_handler);
	ble_init(&ble_pickit);
    ble_stack_init();

	// Initialization sequence
	// Get the name by the client (PIC or other) before initializing BLE module
	while (1)
	{
		static uint64_t tick_init = 0;

		ble_pickit.params.pa_lna_enable |= (ble_pickit.params.pa_lna_enable | board_button_get(BUTTON_1));

		ble_stack_tasks();

		if (mTickCompare(tick_init) >= TICK_500MS)
		{
			ble_pickit.status.is_init_done = true;
			break;
		}
	}

	board_pa_lna_init(ble_pickit.params.pa_lna_enable);
    gap_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();

	// Start execution.
	NRF_LOG_INFO("BLE PICKIT - NAME: %s", ble_pickit.infos.device_name);
	NRF_LOG_INFO("	Power Amplifier / Low Noise Amplifier: %s", (ble_pickit.params.pa_lna_enable)  ? "enable" : "disable");
	NRF_LOG_INFO("	Preferred connection parameters: ");
	NRF_LOG_INFO("	min_conn_param: " NRF_LOG_FLOAT_MARKER " ms", NRF_LOG_FLOAT((float)(ble_pickit.params.preferred_gap_params.conn_params.min_conn_interval)*UNIT_1_25_MS/1000));
	NRF_LOG_INFO("	max_conn_param: " NRF_LOG_FLOAT_MARKER " ms", NRF_LOG_FLOAT((float)(ble_pickit.params.preferred_gap_params.conn_params.max_conn_interval)*UNIT_1_25_MS/1000));
	NRF_LOG_INFO("	slave_latency: %d", ble_pickit.params.preferred_gap_params.conn_params.slave_latency);
	NRF_LOG_INFO("	timeout: %d ms", (ble_pickit.params.preferred_gap_params.conn_params.conn_sup_timeout*UNIT_10_MS/1000));
	NRF_LOG_INFO("	Preferred PHY parameter: TX = %d / RX = %d", ble_pickit.params.preferred_gap_params.phys_params.tx_phys, ble_pickit.params.preferred_gap_params.phys_params.rx_phys);
	NRF_LOG_INFO("	Preferred MTU size: TX = %d / RX = %d", ble_pickit.params.preferred_gap_params.mtu_size_params.max_tx_octets-4, ble_pickit.params.preferred_gap_params.mtu_size_params.max_rx_octets-4);
	err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(err_code);

    // Enter main loop.
	while (1)
	{

		ble_stack_tasks();


		if (m_msg.ble_params.change_conn_params_request)
		{
			err_code = sd_ble_gap_conn_param_update(m_conn_handle, &ble_pickit.params.preferred_gap_params.conn_params);
			APP_ERROR_CHECK(err_code);
			m_msg.ble_params.change_conn_params_request = false;
		}
		else if (m_msg.ble_params.change_phy_param_request)
		{
			err_code = sd_ble_gap_phy_update(m_conn_handle, &ble_pickit.params.preferred_gap_params.phys_params);
			APP_ERROR_CHECK(err_code);
			m_msg.ble_params.change_phy_param_request = false;
		}
		else if (m_msg.ble_params.change_mtu_size_params_request)
		{
			err_code = sd_ble_gap_data_length_update(m_conn_handle, &ble_pickit.params.preferred_gap_params.mtu_size_params, NULL);
			APP_ERROR_CHECK(err_code);
			m_msg.ble_params.change_mtu_size_params_request = false;
		}




		if ((NRF_LOG_PROCESS() == false) && pwr_mgmt_enable)
		{
			nrf_pwr_mgmt_run();
		}
	}
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void rtc_init(void)
{
	ret_code_t err_code;
	nrf_drv_rtc_config_t config;

	rtc.p_reg            = NRFX_CONCAT_2(NRF_RTC, 2);
	rtc.irq              = NRFX_CONCAT_3(RTC, 2, _IRQn);
	rtc.instance_id      = 2;
	rtc.cc_channel_count = NRF_RTC_CC_CHANNEL_COUNT(2);

	config.prescaler = 0;		// x (+1 avec le 0) ==> (x+1)/32768 = y seconds
	config.interrupt_priority = NRFX_RTC_DEFAULT_CONFIG_IRQ_PRIORITY;
	config.reliable = NRFX_RTC_DEFAULT_CONFIG_RELIABLE;
	config.tick_latency = NRFX_RTC_US_TO_TICKS(NRFX_RTC_MAXIMUM_LATENCY_US, NRFX_RTC_DEFAULT_CONFIG_FREQUENCY);

	err_code = nrf_drv_rtc_init(&rtc, &config, NULL);
	APP_ERROR_CHECK(err_code);
	nrf_drv_rtc_tick_enable(&rtc, false);	// Enable or disable tick event (event each prescaler/32768 ms)
	nrf_drv_rtc_enable(&rtc);
}

static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = NRF_UART_BAUDRATE_1000000
    };

    nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
    APP_UART_FIFO_INIT(&comm_params, 256, 256, uart_event_handle, APP_IRQ_PRIORITY_LOWEST, err_code);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(bool pwr_mngt_enable)
{
	if (pwr_mngt_enable)
	{
		ret_code_t err_code;
    	err_code = nrf_pwr_mgmt_init();
    	APP_ERROR_CHECK(err_code);
	}
}

static void ble_stack_init(void)
{
	ret_code_t err_code;
	ble_cfg_t ble_cfg;
	ble_opt_t ble_opt;

	err_code = nrf_sdh_enable_request();
	APP_ERROR_CHECK(err_code);

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
	APP_ERROR_CHECK(err_code);
	// Configure the maximum event length.
	memset(&ble_cfg, 0x00, sizeof(ble_cfg));
	ble_cfg.conn_cfg.conn_cfg_tag                     	= APP_BLE_CONN_CFG_TAG;
	ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu		= NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
	ble_cfg.conn_cfg.params.gap_conn_cfg.event_length 	= 320;
	ble_cfg.conn_cfg.params.gap_conn_cfg.conn_count   	= BLE_GAP_CONN_COUNT_DEFAULT;
	err_code = sd_ble_cfg_set(BLE_CONN_CFG_GAP, &ble_cfg, ram_start);
	APP_ERROR_CHECK(err_code);
	memset(&ble_opt, 0x00, sizeof(ble_opt));
	ble_opt.common_opt.conn_evt_ext.enable = 1;
	err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &ble_opt);

	// Enable BLE stack.
	err_code = nrf_sdh_ble_enable(&ram_start);

	if (err_code == NRF_SUCCESS)
	{
		// Verify that value provided in linked script (LD file) matches
		// the SD calculations.
		if (ram_start == (uint32_t) &__data_start__)
		{
			NRF_LOG_INFO("Soft Device enabled, __data_start__ is set correctly.");
		}
		else
		{
			NRF_LOG_INFO("Soft Device enabled, __data_start__ is set incorrectly (should be = 0x%08X instead of 0x%08X).", ram_start, (uint32_t) &__data_start__);
			APP_ERROR_CHECK(NRF_ERROR_FORBIDDEN);
		}
	}
	else if (err_code == NRF_ERROR_NO_MEM)
	{
		// Not enough memory for the Soft Device (value provided is too low).
		NRF_LOG_INFO("Soft Device failed to enabled, __data_start__ is set incorrectly (should be = 0x%08X instead of 0x%08X).", ram_start, (uint32_t) &__data_start__);
	}

	APP_ERROR_CHECK(err_code);

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void gap_init(void)
{
	ret_code_t err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)ble_pickit.infos.device_name, strlen(ble_pickit.infos.device_name));
	APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = ble_pickit.params.preferred_gap_params.conn_params.min_conn_interval;	/**< Minimum acceptable connection interval. */
	gap_conn_params.max_conn_interval = ble_pickit.params.preferred_gap_params.conn_params.max_conn_interval;	/**< Maximum acceptable connection interval. */
	gap_conn_params.slave_latency     = ble_pickit.params.preferred_gap_params.conn_params.slave_latency;
	gap_conn_params.conn_sup_timeout  = ble_pickit.params.preferred_gap_params.conn_params.conn_sup_timeout; 	/**< Connection supervisory timeout (4 seconds). */

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}

static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void advertising_init(void)
{
    ret_code_t err_code;
    ble_advertising_init_t init;

    ble_advdata_manuf_data_t manuf_data;
    uint8_t data_array[] =
    {
    		(ble_pickit.infos.vsd_version[0]),
			(ble_pickit.infos.vsd_version[1]),
			(ble_pickit.infos.vsd_version[2]),
			(ble_pickit.infos.vsd_version[3]),
			(ble_pickit.infos.vsd_version[4]),
			(ble_pickit.infos.vsd_version[5]),
			(ble_pickit.infos.vsd_version[6]),
			ble_pickit.params.pa_lna_enable & 0x01,
			ble_pickit.params.leds_status_enable & 0x01
    };
    manuf_data.company_identifier = 0x01ee;				// Valeo Service company ID
    manuf_data.data.p_data = data_array;
    manuf_data.data.size = sizeof(data_array);

    memset(&init, 0, sizeof(init));

    /*
     * Set of ble_advdata_t (Advertising data)
     */
    init.advdata.name_type               				= BLE_ADVDATA_NO_NAME;
    init.advdata.short_name_len 						= 0;		// use with name_type = BLE_ADVDATA_SHORT_NAME
    init.advdata.include_appearance      				= false;
    init.advdata.flags                   				= BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_more_available.uuid_cnt 			= 0;
    init.advdata.uuids_more_available.p_uuids 			= NULL;
    init.advdata.uuids_complete.uuid_cnt 				= 0;
    init.advdata.uuids_complete.p_uuids  				= NULL;
    init.advdata.uuids_solicited.uuid_cnt 				= 0;
    init.advdata.uuids_solicited.p_uuids 				= NULL;
    init.advdata.p_slave_conn_int->min_conn_interval	= ble_pickit.params.preferred_gap_params.conn_params.min_conn_interval;
    init.advdata.p_slave_conn_int->max_conn_interval	= ble_pickit.params.preferred_gap_params.conn_params.max_conn_interval;
    init.advdata.p_manuf_specific_data					= NULL;
    init.advdata.p_service_data_array					= NULL;
    init.advdata.service_data_count						= 0;
    init.advdata.include_ble_device_addr				= false;
    init.advdata.le_role								= BLE_ADVDATA_ROLE_NOT_PRESENT;
    init.advdata.p_tk_value								= NULL;
    init.advdata.p_sec_mgr_oob_flags 					= NULL;
    init.advdata.p_lesc_data 							= NULL;

    /*
	 * Set of ble_advdata_t (Scan response data)
	 */
    init.srdata.name_type								= BLE_ADVDATA_FULL_NAME;
	init.srdata.p_manuf_specific_data					= &manuf_data;

    /*
     * Set of ble_adv_modes_config_t
     */
    init.config.ble_adv_on_disconnect_disabled			= false;
    init.config.ble_adv_whitelist_enabled 				= false;
    init.config.ble_adv_extended_enabled				= false;

    init.config.ble_adv_fast_enabled  					= true;
    init.config.ble_adv_fast_interval 					= ble_pickit.params.preferred_gap_params.adv_interval;		// in units of 0.625 ms	(20ms to 10240ms)
    init.config.ble_adv_fast_timeout  					= ble_pickit.params.preferred_gap_params.adv_timeout;		// in units of 10 ms

    init.config.ble_adv_slow_enabled					= false;
    init.config.ble_adv_slow_interval 					= 0;			// in units of 0.625 ms (1000ms to 10240ms)
    init.config.ble_adv_slow_timeout  					= 0;			// in units of 10 ms

    init.config.ble_adv_directed_high_duty_enabled 		= false;
    init.config.ble_adv_directed_enabled				= false;
    init.config.ble_adv_directed_interval 				= 0;			// in units of 0.625 ms
    init.config.ble_adv_directed_timeout  				= 0;			// in units of 10 ms

    /*
     * Set of events handler
     */
    init.evt_handler = adv_evt_handler;
    init.error_handler = adv_err_evt_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void services_init(void)
{
    ret_code_t         	err_code;
    nrf_ble_qwr_init_t 	qwr_init = {0};
    ble_msg_init_t 		msg_init;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

	// Initialize MSG Service init structure to zero.
	memset(&msg_init, 0, sizeof(msg_init));

	// Set the msg event handler
	msg_init.evt_handler = on_service_event_handler;

	// Allowing peer to notify, read & write to characteristic
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&msg_init.char_app_security.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&msg_init.char_app_security.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&msg_init.char_app_security.write_perm);

	// Allowing peer to notify, read & write to characteristic
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&msg_init.char_test_security.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&msg_init.char_test_security.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&msg_init.char_test_security.write_perm);

	// Allowing peer to notify, read & write to characteristic
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&msg_init.char_params_security.cccd_write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&msg_init.char_params_security.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&msg_init.char_params_security.write_perm);

	ble_pickit_service_set_link_with_vsd(&ble_pickit);
	err_code = ble_pickit_service_init(&m_msg, &msg_init);
	if (err_code == NRF_ERROR_NO_MEM)
	{
		NRF_LOG_INFO("Service init error.");
	}
	APP_ERROR_CHECK(err_code);
}

static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = &ble_pickit.params.preferred_gap_params.conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = conn_params_evt_handler;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}
