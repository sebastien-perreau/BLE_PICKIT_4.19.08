#include "sdk_common.h"
#include "nrf_log.h"
#include "ble_pickit_board.h"
#include "ble_vsd.h"
#include "ble_pickit_service.h"


static ble_pickit_t * p_vsd;

static void _version(uint8_t *buffer);
static void _transfer_ble_to_uart(uint8_t *buffer);

static void _notif_buffer(uint8_t *buffer);
static void _notif_scenario(uint8_t *buffer);

static uint8_t vsd_send_request(p_function ptr);

void ble_init(ble_pickit_t * p_vsd_params)
{
    p_vsd = p_vsd_params;
}

void ble_stack_tasks()
{
	static uint64_t tick_blink_led_1 = 0;
	static uint64_t tick_blink_led_3 = 0;
	ret_code_t err_code;

	if (p_vsd->params.leds_status_enable)
	{
		if (!p_vsd->status.is_init_done)
		{
			board_led_set(LED_1);
			board_led_set(LED_2);
			board_led_set(LED_3);
		}
		else
		{

			if (p_vsd->status.is_ble_service_has_event)
			{
				tick_blink_led_1 = mGetTick();
				p_vsd->status.is_ble_service_has_event = false;
				board_led_set(LED_1);
			}
			else if (mTickCompare(tick_blink_led_1) > TICK_1MS)
			{
				board_led_clr(LED_1);
			}

			if (p_vsd->status.is_connected_to_a_central)
			{
				board_led_set(LED_2);
			}
			else if (p_vsd->status.is_in_advertising_mode)
			{
				board_led_lat(LED_2, (mGetTick() >> 12)&1);
			}
			else
			{
				board_led_clr(LED_2);
			}

			if ((p_vsd->uart.transmit_in_progress) || (p_vsd->status.is_uart_message_receives))
			{
				tick_blink_led_3 = mGetTick();
				p_vsd->status.is_uart_message_receives = false;
				board_led_set(LED_3);
			}
			else if (mTickCompare(tick_blink_led_3) > TICK_20MS)
			{
				board_led_clr(LED_3);
			}
		}
	}
	else
	{
		board_led_clr(LED_1);
		board_led_clr(LED_2);
		board_led_clr(LED_3);
	}

    err_code = app_uart_get(&p_vsd->uart.buffer[p_vsd->uart.index]);
	if (err_code == NRF_SUCCESS)
	{
		p_vsd->uart.receive_in_progress = true;
		p_vsd->uart.tick = mGetTick();
		p_vsd->uart.index++;
	}

    if (mTickCompare(p_vsd->uart.tick) >= TICK_300US)
    {
        if (	(p_vsd->uart.index == 3) && 		\
                (p_vsd->uart.buffer[0] == 'A') && 	\
                (p_vsd->uart.buffer[1] == 'C') && 	\
                (p_vsd->uart.buffer[2] == 'K'))
        {
            p_vsd->uart.message_type = UART_ACK_MESSAGE;
        }
        else if (	(p_vsd->uart.index == 4) &&	 		\
                    (p_vsd->uart.buffer[0] == 'N') && 	\
                    (p_vsd->uart.buffer[1] == 'A') && 	\
                    (p_vsd->uart.buffer[2] == 'C') && 	\
                    (p_vsd->uart.buffer[3] == 'K'))
        {
            p_vsd->uart.message_type = UART_NACK_MESSAGE;
        }
        else if ((p_vsd->uart.index > 5) && (p_vsd->uart.buffer[1] == 'W'))
        {
            p_vsd->uart.message_type = UART_NEW_MESSAGE;
        }
        else
        {
            p_vsd->uart.message_type = UART_OTHER_MESSAGE;
        }
        p_vsd->uart.index = 0;
        p_vsd->uart.receive_in_progress = false;
    }

    if (p_vsd->uart.message_type == UART_NEW_MESSAGE)
    {
        uint8_t i;
        uint16_t crc_calc, crc_uart;

        p_vsd->status.is_uart_message_receives = true;
        p_vsd->uart.message_type = UART_NO_MESSAGE;

        crc_calc = fu_crc_16_ibm(p_vsd->uart.buffer, p_vsd->uart.buffer[2]+3);
        crc_uart = (p_vsd->uart.buffer[p_vsd->uart.buffer[2]+3] << 8) + (p_vsd->uart.buffer[p_vsd->uart.buffer[2]+4] << 0);

        if (crc_calc == crc_uart)
        {
            p_vsd->incoming_uart_message.id = p_vsd->uart.buffer[0];
            p_vsd->incoming_uart_message.type = p_vsd->uart.buffer[1];
            p_vsd->incoming_uart_message.length = p_vsd->uart.buffer[2];
            for (i = 0 ; i < p_vsd->incoming_uart_message.length ; i++)
            {
                p_vsd->incoming_uart_message.data[i] = p_vsd->uart.buffer[3+i];
            }
            do {} while (app_uart_put('A') != NRF_SUCCESS);
			do {} while (app_uart_put('C') != NRF_SUCCESS);
			do {} while (app_uart_put('K') != NRF_SUCCESS);
            p_vsd->uart.transmit_in_progress = true;
        }
        else
        {
            p_vsd->incoming_uart_message.id = 0x00;
            do {} while (app_uart_put('N') != NRF_SUCCESS);
            do {} while (app_uart_put('A') != NRF_SUCCESS);
			do {} while (app_uart_put('C') != NRF_SUCCESS);
			do {} while (app_uart_put('K') != NRF_SUCCESS);
            p_vsd->uart.transmit_in_progress = true;
        }
        memset(p_vsd->uart.buffer, 0, sizeof(p_vsd->uart.buffer));

        switch (p_vsd->incoming_uart_message.id)
        {
        	case ID_PA_LNA:
        		p_vsd->params.pa_lna_enable = p_vsd->incoming_uart_message.data[0] & 0x01;
        		break;

        	case ID_LED_STATUS:
        		p_vsd->params.leds_status_enable = p_vsd->incoming_uart_message.data[0] & 0x01;
        		break;

			case ID_SET_NAME:
				memcpy(p_vsd->infos.device_name, p_vsd->incoming_uart_message.data, p_vsd->incoming_uart_message.length);
				p_vsd->infos.device_name[p_vsd->incoming_uart_message.length] = '\0';
				break;

            case ID_GET_VERSION:
                p_vsd->flags.send_version = true;
                break;

            case ID_ADV_INTERVAL:
            	p_vsd->params.preferred_gap_params.adv_interval = (p_vsd->incoming_uart_message.data[0] << 8) | (p_vsd->incoming_uart_message.data[1] << 0);
            	break;

            case ID_ADV_TIMEOUT:
            	p_vsd->params.preferred_gap_params.adv_timeout = (p_vsd->incoming_uart_message.data[0] << 8) | (p_vsd->incoming_uart_message.data[1] << 0);
            	break;

            case ID_SOFTWARE_RESET:
            	if ((p_vsd->incoming_uart_message.length == 1) && ((p_vsd->incoming_uart_message.data[0] == RESET_ALL) || (p_vsd->incoming_uart_message.data[0] == RESET_BLE_PICKIT)))
				{
					p_vsd->flags.exec_reset = true;
				}
                break;

            case ID_CHAR_BUFFER:
            	p_vsd->flags.notification_buffer = true;
            	memcpy(p_vsd->characteristic.buffer.data, p_vsd->incoming_uart_message.data, p_vsd->incoming_uart_message.length);
            	p_vsd->characteristic.buffer.length = p_vsd->incoming_uart_message.length;
            	break;

            case ID_CHAR_SCENARIO:
            	p_vsd->flags.notification_scenario = true;
            	p_vsd->characteristic.scenario.index = p_vsd->incoming_uart_message.data[0];
            	break;

            default:
                break;

        }
    }

    if (p_vsd->flags.w > 0)
    {

    	/** Send Serial message over UART */
    	if (p_vsd->flags.send_version)
		{
			if (!vsd_send_request(_version))
			{
				p_vsd->flags.send_version = false;
			}
		}
        else if (p_vsd->flags.transfer_ble_to_uart)
		{
        	if (!vsd_send_request(_transfer_ble_to_uart))
			{
				p_vsd->flags.transfer_ble_to_uart = false;
			}
		}
        else if (p_vsd->flags.exec_reset)
		{
			if (!p_vsd->uart.transmit_in_progress)
			{
				sd_nvic_SystemReset();
			}
		}

    	/** Send NOTIFICATION over BLE */
    	if (p_vsd->flags.notification_buffer)
        {
        	if (!ble_pickit_app_notification_send(_notif_buffer))
        	{
        		p_vsd->flags.notification_buffer = false;
        	}
        }
    	else if (p_vsd->flags.notification_scenario)
        {
        	if (!ble_pickit_app_notification_send(_notif_scenario))
        	{
        		p_vsd->flags.notification_scenario = false;
        	}
        }
    }

}

static void _version(uint8_t *buffer)
{
    uint8_t i = 0;
	uint16_t crc = 0;

	buffer[0] = ID_GET_VERSION;
	buffer[1] = 'N';
	buffer[2] = 7;
	for (i = 0 ; i < 7 ; i++)
	{
		buffer[3+i] = p_vsd->infos.vsd_version[i];
	}
	crc = fu_crc_16_ibm(buffer, buffer[2]+3);
	buffer[buffer[2]+3] = (crc >> 8) & 0xff;
	buffer[buffer[2]+4] = (crc >> 0) & 0xff;
}

static void _transfer_ble_to_uart(uint8_t *buffer)
{
    uint8_t i = 0;
	uint16_t crc = 0;

	buffer[0] = p_vsd->outgoing_uart_message.id;
	buffer[1] = p_vsd->outgoing_uart_message.type;
	buffer[2] = p_vsd->outgoing_uart_message.length;
	for (i = 0 ; i < buffer[2] ; i++)
	{
		buffer[3+i] = p_vsd->outgoing_uart_message.data[i];
	}
	crc = fu_crc_16_ibm(buffer, buffer[2]+3);
	buffer[buffer[2]+3] = (crc >> 8) & 0xff;
	buffer[buffer[2]+4] = (crc >> 0) & 0xff;
}

static void _notif_buffer(uint8_t *buffer)
{
	uint8_t i = 0;

	buffer[0] = ID_CHAR_BUFFER;
	buffer[1] = p_vsd->characteristic.buffer.length;
	for (i = 0 ; i < buffer[1] ; i++)
	{
		buffer[2+i] = p_vsd->characteristic.buffer.data[i];
	}
}

static void _notif_scenario(uint8_t *buffer)
{
	buffer[0] = ID_CHAR_SCENARIO;
	buffer[1] = 1;
	buffer[2] = p_vsd->characteristic.scenario.index;
}

static uint8_t vsd_send_request(p_function ptr)
{
    static state_machine_t sm;
	static uint8_t buffer[256] = {0};

	switch (sm.index)
	{
		case 0:
			sm.index++;
			sm.tick = mGetTick();
        case 1:

            if (!p_vsd->uart.transmit_in_progress && !p_vsd->uart.receive_in_progress)
            {
                sm.index++;
                sm.tick = mGetTick();
            }
            break;

        case 2:
        	if (mTickCompare(sm.tick) >= TICK_400US)
        	{
        		if (!p_vsd->uart.transmit_in_progress && !p_vsd->uart.receive_in_progress)
				{
					sm.index++;
					sm.tick = mGetTick();
				}
        		else
        		{
        			sm.index = 1;
        		}
        	}
        	break;

		case 3:

			(*ptr)(buffer);

			for (uint8_t i = 0 ; i <= (buffer[2]+4) ; i++)
			{
				do {} while (app_uart_put(buffer[i]) != NRF_SUCCESS);
			}
            p_vsd->uart.transmit_in_progress = true;

			sm.index++;
			sm.tick = mGetTick();
			break;

		case 4:

            if (!p_vsd->uart.transmit_in_progress)
            {
                memset(buffer, 0, sizeof(buffer));
                sm.index++;
                sm.tick = mGetTick();
            }
			break;

		case 5:

            if (p_vsd->uart.message_type == UART_ACK_MESSAGE)
            {
                p_vsd->uart.message_type = UART_NO_MESSAGE;
                sm.index = 0;
            }
            else if (p_vsd->uart.message_type == UART_NACK_MESSAGE)
            {
                p_vsd->uart.message_type = UART_NO_MESSAGE;
                sm.index = 3;
            }
            else if (mTickCompare(sm.tick) >= TICK_10MS)
            {
                sm.index = 3;
            }
			break;

		default:
            sm.index = 0;
			break;

	}

	return sm.index;
}

