#include "sdk_common.h"
#include "ble_srv_common.h"
#include "nrf_log.h"
#include "ble_pickit_board.h"
#include "ble_vsd.h"
#include "ble_pickit_service.h"

static ble_pickit_t * p_vsd;
static ble_msg_t * p_msg;

void ble_pickit_service_set_link_with_vsd(ble_pickit_t * p)
{
	p_vsd = p;
}

uint32_t ble_pickit_service_init(ble_msg_t * p, const ble_msg_init_t * p_msg_init)
{
	if (p == NULL || p_msg_init == NULL)
	{
		return NRF_ERROR_NULL;
	}

	uint32_t err_code;
	ble_uuid_t ble_uuid;

	p_msg = p;

	// Initialize service structure
	p->evt_handler	= p_msg_init->evt_handler;
	p->conn_handle	= BLE_CONN_HANDLE_INVALID;

	p->char_app.is_notification_enabled = false;
	p->char_test.is_notification_enabled = false;
	p->char_params.is_notification_enabled = false;

	p->ble_params.change_conn_params_request = false;
	p->ble_params.change_phy_param_request= false;
	p->ble_params.change_mtu_size_params_request = false;

	p->ble_params.notify_all_parameters = false;

	// Add Message Service UUID
	ble_uuid128_t base_uuid = {MESSAGE_SERVICE_UUID_BASE};
	err_code = sd_ble_uuid_vs_add(&base_uuid, &p->uuid_type);
	VERIFY_SUCCESS(err_code);

	ble_uuid.type = p->uuid_type;
	ble_uuid.uuid = MESSAGE_SERVICE_UUID;

	// Add Message Service to the BLE Stack's GATT table
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p->service_handle);

	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = add_characteristic_app_0x1501(p, p_msg_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = add_characteristic_test_0x1502(p, p_msg_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = add_characteristic_params_0x1503(p, p_msg_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	return NRF_SUCCESS;
}

uint32_t add_characteristic_app_0x1501(ble_msg_t * p_msg, const ble_msg_init_t * p_msg_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Set CCCD
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    // Properties displayed to the central during service discovery
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read     = 0;        // We want to read
    char_md.char_props.write    = 1;        // We want to write
    char_md.char_props.notify   = 1;        // We want to notify
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = &cccd_md;
    char_md.p_sccd_md           = NULL;

    // Set the properties (ie accessability of the attribute)
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm   = p_msg_init->char_app_security.read_perm; 	// must correspond to the char_md properties
    attr_md.write_perm  = p_msg_init->char_app_security.write_perm;    // same
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;                             // characteristic stored in the softdevice RAM (and therefore not in the application RAM)
    attr_md.rd_auth     = 0;
    attr_md.wr_auth     = 0;
    attr_md.vlen        = 0;

    // CHAR UUID
    ble_uuid.type = p_msg->uuid_type;
    ble_uuid.uuid = MESSAGE_APP_CHAR_UUID;

    // Set UUID, pointer to attr_md, set size of the characteristic
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid      = &ble_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.init_len    = sizeof(uint8_t);
    attr_char_value.init_offs   = 0;
    attr_char_value.max_len     = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
    attr_char_value.p_value     = NULL;

    // Structure are populated, now we add the characteristic
    err_code = sd_ble_gatts_characteristic_add(p_msg->service_handle, &char_md, &attr_char_value, &p_msg->char_app.handles);

    return err_code;
}

uint32_t add_characteristic_test_0x1502(ble_msg_t * p_msg, const ble_msg_init_t * p_msg_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Set CCCD
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    // Properties displayed to the central during service discovery
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read     = 0;        // We do not want to read
    char_md.char_props.write    = 1;        // We want to write
    char_md.char_props.notify   = 1;        // We want to notify
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = &cccd_md;
    char_md.p_sccd_md           = NULL;
    
    // Set the properties (ie accessability of the attribute)
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm   = p_msg_init->char_test_security.read_perm; 		// must correspond to the char_md properties
    attr_md.write_perm  = p_msg_init->char_test_security.write_perm;    	// same
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;                             		// characteristic stored in the softdevice RAM (and therefore not in the application RAM)
    attr_md.rd_auth     = 0;
    attr_md.wr_auth     = 0;
    attr_md.vlen        = 0;
    
    // CHAR UUID
    ble_uuid.type = p_msg->uuid_type;
    ble_uuid.uuid = MESSAGE_TEST_UUID;

    // Set UUID, pointer to attr_md, set size of the characteristic
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid      = &ble_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.init_len    = sizeof(uint8_t);
    attr_char_value.init_offs   = 0;
    attr_char_value.max_len     = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
    attr_char_value.p_value     = NULL;

    // Structure are populated, now we add the characteristic
    err_code = sd_ble_gatts_characteristic_add(p_msg->service_handle, &char_md, &attr_char_value, &p_msg->char_test.handles);

    return err_code;
}

uint32_t add_characteristic_params_0x1503(ble_msg_t * p_msg, const ble_msg_init_t * p_msg_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // Set CCCD
    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    // Properties displayed to the central during service discovery
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read     = 0;        // We want to read
    char_md.char_props.write    = 1;        // We want to write
    char_md.char_props.notify   = 1;        // We want to notify
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = &cccd_md;
    char_md.p_sccd_md           = NULL;

    // Set the properties (ie accessability of the attribute)
    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm   = p_msg_init->char_params_security.read_perm; 		// must correspond to the char_md properties
    attr_md.write_perm  = p_msg_init->char_params_security.write_perm;    	// same
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;                             // characteristic stored in the softdevice RAM (and therefore not in the application RAM)
    attr_md.rd_auth     = 0;
    attr_md.wr_auth     = 0;
    attr_md.vlen        = 0;

    // CHAR UUID
    ble_uuid.type = p_msg->uuid_type;
    ble_uuid.uuid = MESSAGE_PARAMS_UUID;

    // Set UUID, pointer to attr_md, set size of the characteristic
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid      = &ble_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.init_len    = sizeof(uint8_t);
    attr_char_value.init_offs   = 0;
    attr_char_value.max_len     = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
    attr_char_value.p_value     = NULL;

    // Structure are populated, now we add the characteristic
    err_code = sd_ble_gatts_characteristic_add(p_msg->service_handle, &char_md, &attr_char_value, &p_msg->char_params.handles);

    return err_code;
}



void ble_pickit_throughput_notification_send(ble_msg_t * p_msg)
{
	uint32_t err_code = NRF_SUCCESS;
	uint8_t throughput_data[NRF_SDH_BLE_GATT_MAX_MTU_SIZE] = {0};
	uint16_t _att_payload;
	ble_gatts_hvx_params_t const hvx_param =
	{
		.handle = p_msg->char_test.handles.value_handle,
		.type   = BLE_GATT_HVX_NOTIFICATION,
		.offset = 0,
		.p_len  = &_att_payload,
		.p_data = throughput_data,
	};
	static uint32_t delta_bytes = 0;
	static uint64_t delta_time = 0;

	_att_payload = ((p_msg->throughput.bytes_transmitted + p_msg->att_payload) > p_msg->throughput._end_count) ? (p_msg->throughput._end_count - p_msg->throughput.bytes_transmitted) : p_msg->att_payload;
	if (((_att_payload == 0) || (mTickCompare(p_msg->throughput._start_time_ms) > TICK_1S*60)) && (p_msg->throughput._sm.index == 1))
	{
		p_msg->throughput._sm.index = 2;
	}

	switch (p_msg->throughput._sm.index)
	{
		case 0:
			p_msg->throughput.indice = 0;
			p_msg->throughput.elapsed_time_ms = 0;
			p_msg->throughput.bytes_transmitted = 0;
			p_msg->throughput.current_data_rate_kbps = 0.0;
			p_msg->throughput.average_data_rate_kbps = 0.0;
			p_msg->throughput.min_packets_per_interval_of_conn = 0;
			p_msg->throughput.max_packets_per_interval_of_conn = 0;

			p_msg->throughput._end_count = 1024 * ((p_msg->throughput._type > 2) ? (1024 * ((p_msg->throughput._type > 3) ? 50 : 1)) : 1);
			p_msg->throughput._start_time_ms = mGetTick();
			p_msg->throughput._sm.tick = p_msg->throughput._start_time_ms;
			p_msg->throughput._sm.index++;

			delta_bytes = 0;
			delta_time = 0;

			_att_payload = p_msg->att_payload;

		case 1:
			if (p_msg->conn_handle != BLE_CONN_HANDLE_INVALID)
			{
				while (err_code == NRF_SUCCESS)
				{
					p_msg->throughput.indice++;
					p_msg->throughput.elapsed_time_ms = mTickCompare(p_msg->throughput._start_time_ms) / TICK_1MS;
					p_msg->throughput.bytes_transmitted += _att_payload;
					p_msg->throughput.average_data_rate_kbps = ((float)(p_msg->throughput.bytes_transmitted) / (float)(p_msg->throughput.elapsed_time_ms));
					delta_bytes = p_msg->throughput.bytes_transmitted - delta_bytes;
					delta_time = mTickCompare(delta_time);
					p_msg->throughput.current_data_rate_kbps = ((float)(delta_bytes) / (float)(delta_time / TICK_1MS));

					// Miss ID
					// Miss Length

					throughput_data[0] = p_msg->throughput._type;
					throughput_data[1] = p_msg->throughput._sm.index;
					throughput_data[2] = (p_msg->throughput.indice >> 8);
					throughput_data[3] = (p_msg->throughput.indice >> 0);
					throughput_data[4] = (p_msg->throughput.elapsed_time_ms >> 24);
					throughput_data[5] = (p_msg->throughput.elapsed_time_ms >> 16);
					throughput_data[6] = (p_msg->throughput.elapsed_time_ms >> 8);
					throughput_data[7] = (p_msg->throughput.elapsed_time_ms >> 0);
					throughput_data[8] = (p_msg->throughput.bytes_transmitted >> 24);
					throughput_data[9] = (p_msg->throughput.bytes_transmitted >> 16);
					throughput_data[10] = (p_msg->throughput.bytes_transmitted >> 8);
					throughput_data[11] = (p_msg->throughput.bytes_transmitted >> 0);
					throughput_data[12] = fu_integer_value(p_msg->throughput.current_data_rate_kbps);
					throughput_data[13] = fu_decimal_value(p_msg->throughput.current_data_rate_kbps);
					throughput_data[14] = fu_integer_value(p_msg->throughput.average_data_rate_kbps);
					throughput_data[15] = fu_decimal_value(p_msg->throughput.average_data_rate_kbps);
					throughput_data[16] = p_msg->throughput.min_packets_per_interval_of_conn;
					throughput_data[17] = p_msg->throughput.max_packets_per_interval_of_conn;

					err_code = sd_ble_gatts_hvx(p_msg->conn_handle, &hvx_param);

					if (err_code == NRF_SUCCESS)
					{
						p_msg->char_test.notifications_on_going++;
					}
					else if (err_code == NRF_ERROR_RESOURCES)
					{
						// Wait for BLE_GATTS_EVT_HVN_TX_COMPLETE.
						p_msg->throughput.indice--;
						p_msg->throughput.bytes_transmitted -= _att_payload;
						break;
					}
					else if (err_code != NRF_SUCCESS)
					{
						NRF_LOG_ERROR("sd_ble_gatts_hvx() failed: 0x%x", err_code);
					}

	//				NRF_LOG_INFO("Elapsed time: %d - Data: " NRF_LOG_FLOAT_MARKER " Ko/s", p_msg->throughput.elapsed_time_ms, NRF_LOG_FLOAT(p_msg->throughput.current_data_rate_kbps));
					if (mTickCompare(p_msg->throughput._sm.tick) >= TICK_1S)
					{
						p_msg->throughput._sm.tick = mGetTick();
						NRF_LOG_INFO("Data: " NRF_LOG_FLOAT_MARKER " Ko/s", NRF_LOG_FLOAT(p_msg->throughput.average_data_rate_kbps));
					}
				}
			}

			break;

		case 2:
			if (p_msg->conn_handle != BLE_CONN_HANDLE_INVALID)
			{
				_att_payload = 18;
				p_msg->throughput.elapsed_time_ms = mTickCompare(p_msg->throughput._start_time_ms) / TICK_1MS;
				p_msg->throughput.average_data_rate_kbps = ((float)(p_msg->throughput.bytes_transmitted) / (float)(p_msg->throughput.elapsed_time_ms));
				p_msg->throughput.current_data_rate_kbps = 0.0;

				throughput_data[0] = p_msg->throughput._type;
				throughput_data[1] = p_msg->throughput._sm.index;
				throughput_data[2] = (p_msg->throughput.indice >> 8);
				throughput_data[3] = (p_msg->throughput.indice >> 0);
				throughput_data[4] = (p_msg->throughput.elapsed_time_ms >> 24);
				throughput_data[5] = (p_msg->throughput.elapsed_time_ms >> 16);
				throughput_data[6] = (p_msg->throughput.elapsed_time_ms >> 8);
				throughput_data[7] = (p_msg->throughput.elapsed_time_ms >> 0);
				throughput_data[8] = (p_msg->throughput.bytes_transmitted >> 24);
				throughput_data[9] = (p_msg->throughput.bytes_transmitted >> 16);
				throughput_data[10] = (p_msg->throughput.bytes_transmitted >> 8);
				throughput_data[11] = (p_msg->throughput.bytes_transmitted >> 0);
				throughput_data[12] = fu_integer_value(p_msg->throughput.current_data_rate_kbps);
				throughput_data[13] = fu_decimal_value(p_msg->throughput.current_data_rate_kbps);
				throughput_data[14] = fu_integer_value(p_msg->throughput.average_data_rate_kbps);
				throughput_data[15] = fu_decimal_value(p_msg->throughput.average_data_rate_kbps);
				throughput_data[16] = p_msg->throughput.min_packets_per_interval_of_conn;
				throughput_data[17] = p_msg->throughput.max_packets_per_interval_of_conn;

				err_code = sd_ble_gatts_hvx(p_msg->conn_handle, &hvx_param);

				if (err_code == NRF_SUCCESS)
				{
					p_msg->char_test.notifications_on_going++;
					p_msg->throughput._sm.index++;
				}
				else if (err_code == NRF_ERROR_RESOURCES)
				{
					// Wait for BLE_GATTS_EVT_HVN_TX_COMPLETE.
					break;
				}
				else if (err_code != NRF_SUCCESS)
				{
					NRF_LOG_ERROR("sd_ble_gatts_hvx() failed: 0x%x", err_code);
				}

				if (p_msg->throughput._type == THROUGHPUT_STOP_TEST)
				{
					NRF_LOG_INFO("Test stopped - Results: ");
				}
				else if (p_msg->throughput._type == THROUGHPUT_LAUNCH_TEST_1KB)
				{
					NRF_LOG_INFO("Test 1 Ko finished: ");
				}
				else if (p_msg->throughput._type == THROUGHPUT_LAUNCH_TEST_1MB)
				{
					NRF_LOG_INFO("Test 1 Mo finished: ");
				}
				else
				{
					NRF_LOG_INFO("Test 60 seconds finished: ");
				}
				NRF_LOG_INFO("Number of transmission: %d", p_msg->throughput.indice);
				NRF_LOG_INFO("Data transmitted: %d bytes in %d milliseconds with a throughput average of " NRF_LOG_FLOAT_MARKER " Ko/s (%d Kbps).", p_msg->throughput.bytes_transmitted, p_msg->throughput.elapsed_time_ms, NRF_LOG_FLOAT(p_msg->throughput.average_data_rate_kbps), p_msg->throughput.average_data_rate_kbps*8);
			}

			break;

		default:
			// Do nothing
			break;

	}
}

void ble_pickit_parameters_notification_send()
{
	uint32_t err_code = NRF_SUCCESS;
	uint8_t params_data[14] = {0};
	uint16_t _att_payload = 14;
	ble_gatts_hvx_params_t const hvx_param =
	{
		.handle = p_msg->char_params.handles.value_handle,
		.type   = BLE_GATT_HVX_NOTIFICATION,
		.offset = 0,
		.p_len  = &_att_payload,
		.p_data = params_data,
	};

	if (p_msg->ble_params.notify_all_parameters && (p_msg->conn_handle != BLE_CONN_HANDLE_INVALID))
	{
		p_msg->ble_params.notify_all_parameters = false;

		params_data[0] = 0x00; 	// ID
		params_data[1] = 12;	// Length

		params_data[2] = (uint8_t) (p_vsd->params.current_gap_params.conn_params.min_conn_interval >> 8);
		params_data[3] = (uint8_t) (p_vsd->params.current_gap_params.conn_params.min_conn_interval >> 0);
		params_data[4] = (uint8_t) (p_vsd->params.current_gap_params.conn_params.max_conn_interval >> 8);
		params_data[5] = (uint8_t) (p_vsd->params.current_gap_params.conn_params.max_conn_interval >> 0);
		params_data[6] = (uint8_t) (p_vsd->params.current_gap_params.conn_params.slave_latency >> 8);
		params_data[7] = (uint8_t) (p_vsd->params.current_gap_params.conn_params.slave_latency >> 0);
		params_data[8] = (uint8_t) (p_vsd->params.current_gap_params.conn_params.conn_sup_timeout >> 8);
		params_data[9] = (uint8_t) (p_vsd->params.current_gap_params.conn_params.conn_sup_timeout >> 0);
		params_data[10] = (uint8_t) (p_vsd->params.current_gap_params.phys_params.tx_phys);
		params_data[11] = (uint8_t) (p_vsd->params.current_gap_params.mtu_size_params.max_tx_octets);
		params_data[12] = (uint8_t) (p_vsd->params.current_gap_params.mtu_size_params.max_rx_octets);
		params_data[13] = (uint8_t) (p_vsd->params.pa_lna_enable);

		err_code = sd_ble_gatts_hvx(p_msg->conn_handle, &hvx_param);

		if (err_code == NRF_ERROR_RESOURCES)
		{
			NRF_LOG_INFO("sd_ble_gatts_hvx() failed: error resources");
		}
		else if (err_code != NRF_SUCCESS)
		{
			NRF_LOG_ERROR("ble_pickit_parameters_notification_send - sd_ble_gatts_hvx() failed: 0x%x", err_code);
		}
	}
}

uint8_t ble_pickit_app_notification_send(p_function ptr)
{
	uint8_t ret = 1;

	if (p_msg->char_app.is_notification_enabled && (p_msg->conn_handle != BLE_CONN_HANDLE_INVALID))
	{
		uint32_t err_code = NRF_SUCCESS;
		uint8_t _buffer[256] = {0};
		uint16_t _att_payload = 0;
		ble_gatts_hvx_params_t const hvx_param =
		{
			.handle = p_msg->char_app.handles.value_handle,
			.type   = BLE_GATT_HVX_NOTIFICATION,
			.offset = 0,
			.p_len  = &_att_payload,
			.p_data = _buffer,
		};

		(*ptr)(_buffer);
		_att_payload = _buffer[1]+2;

		err_code = sd_ble_gatts_hvx(p_msg->conn_handle, &hvx_param);

		if (err_code == NRF_ERROR_RESOURCES)
		{
			ret = 1;
			NRF_LOG_INFO("sd_ble_gatts_hvx() failed: error resources");
		}
		else if (err_code != NRF_SUCCESS)
		{
			ret = 0;
			NRF_LOG_ERROR("ble_pickit_parameters_notification_send - sd_ble_gatts_hvx() failed: 0x%x", err_code);
		}
		else if (err_code == NRF_SUCCESS)
		{
			ret = 0;
		}
	}
	else
	{
		ret = 0;
	}

	return ret;
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_msg       Message Service structure.
 * @param[in]   p_msg_evt   Event received from the BLE stack.
 */
static void on_connect(ble_msg_t * p_msg, ble_evt_t const * p_ble_evt)
{
    p_msg->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_msg_evt_t evt;

    evt.evt_type = SERVICE_EVT_CONNECTED;

    p_msg->evt_handler(p_msg, &evt, NULL, 0);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_msg       Message Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_msg_t * p_msg, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_msg->conn_handle = BLE_CONN_HANDLE_INVALID;

    ble_msg_evt_t evt;

    evt.evt_type = SERVICE_EVT_DISCONNECTED;

    p_msg->evt_handler(p_msg, &evt, NULL, 0);
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_msg       Message Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_msg_t * p_msg, ble_evt_t const * p_ble_evt)
{
	ble_msg_evt_t evt;
	ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	// Check if the handle passed with the event matches the Message Value Characteristic handle.
	if (p_evt_write->handle == p_msg->char_app.handles.value_handle)
	{
		evt.evt_type = SERVICE_EVT_APP_WRITE;
		p_msg->evt_handler(p_msg, &evt, p_ble_evt->evt.gatts_evt.params.write.data, p_ble_evt->evt.gatts_evt.params.write.len);
	}
	else if (p_evt_write->handle == p_msg->char_test.handles.value_handle)
	{
		evt.evt_type = SERVICE_EVT_TEST_WRITE;
		p_msg->evt_handler(p_msg, &evt, p_ble_evt->evt.gatts_evt.params.write.data, p_ble_evt->evt.gatts_evt.params.write.len);
	}
	else if (p_evt_write->handle == p_msg->char_params.handles.value_handle)
	{
		evt.evt_type = SERVICE_EVT_PARAMS_WRITE;
		p_msg->evt_handler(p_msg, &evt, p_ble_evt->evt.gatts_evt.params.write.data, p_ble_evt->evt.gatts_evt.params.write.len);
	}


	// Check if the Message value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_msg->char_app.handles.cccd_handle) && (p_evt_write->len == 2))
    {
        // CCCD written, call application event handler
        if (p_msg->evt_handler != NULL)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = SERVICE_EVT_APP_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = SERVICE_EVT_APP_NOTIFICATION_DISABLED;
            }
            p_msg->evt_handler(p_msg, &evt, NULL, 0);
        }
    }
    else if ((p_evt_write->handle == p_msg->char_test.handles.cccd_handle) && (p_evt_write->len == 2))
    {
        // CCCD written, call application event handler
        if (p_msg->evt_handler != NULL)
        {
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = SERVICE_EVT_TEST_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = SERVICE_EVT_TEST_NOTIFICATION_DISABLED;
            }
            p_msg->evt_handler(p_msg, &evt, NULL, 0);
        }
    }
    else if ((p_evt_write->handle == p_msg->char_params.handles.cccd_handle) && (p_evt_write->len == 2))
        {
            // CCCD written, call application event handler
            if (p_msg->evt_handler != NULL)
            {
                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = SERVICE_EVT_PARAMS_NOTIFICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = SERVICE_EVT_PARAMS_NOTIFICATION_DISABLED;
                }
                p_msg->evt_handler(p_msg, &evt, NULL, 0);
            }
        }
}

///**@brief Function for handling the RW Authorize event.
// *
// * @param[in]   p_msg       Message Service structure.
// * @param[in]   p_ble_evt   Event received from the BLE stack.
// */
//static void on_read(ble_msg_t * p_msg, ble_evt_t const * p_ble_evt)
//{
//    ble_gatts_evt_read_t const * p_evt_read = &p_ble_evt->evt.gatts_evt.params.authorize_request.request.read;
//
//    // Check if the handle passed with the event matches the Message Value Characteristic handle.
//    if (p_evt_read->handle == p_msg->char_app_handles.value_handle)
//    {
//        ble_gatts_rw_authorize_reply_params_t p_rw_authorize_reply_params;
//
//        memset(&p_rw_authorize_reply_params, 0, sizeof(&p_rw_authorize_reply_params));
//
//        p_rw_authorize_reply_params.type                       = BLE_GATTS_AUTHORIZE_TYPE_READ;
//        p_rw_authorize_reply_params.params.read.gatt_status    = 0;
//        p_rw_authorize_reply_params.params.read.update         = 0;
//        p_rw_authorize_reply_params.params.read.offset         = 0;
//        p_rw_authorize_reply_params.params.read.len            = sizeof(uint8_t);
//        p_rw_authorize_reply_params.params.read.p_data         = NULL;
//
//        ret_code_t err_code = sd_ble_gatts_rw_authorize_reply(p_msg->conn_handle, &p_rw_authorize_reply_params);
//        if(err_code != NRF_SUCCESS)
//        {
//            NRF_LOG_INFO("READ AUTH RESP ERROR : err_code %d", err_code);
//        }
//        else
//        {
//            NRF_LOG_INFO("READ AUTH RESP SUCCESS");
//        }
//    }
//
//}

static void on_tx_complete(ble_msg_t * p_msg, ble_evt_t const * p_ble_evt)
{
	if (p_msg->char_test.notifications_on_going > 0)
	{
		p_msg->char_test.notifications_on_going--;
		ble_pickit_throughput_notification_send(p_msg);
	}
}

void ble_pickit_service_event_handler( ble_evt_t const * p_ble_evt, void * p_context)
{
	ble_msg_t * p_msg = (ble_msg_t *) p_context;

	if (p_msg == NULL || p_ble_evt == NULL)
	{
		return;
	}

	switch(p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			NRF_LOG_INFO("Event connected.");
			on_connect(p_msg, p_ble_evt);
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			NRF_LOG_INFO("Event disconnected.");
			on_disconnect(p_msg, p_ble_evt);
			break;

		case BLE_GATTS_EVT_WRITE:
			NRF_LOG_INFO("Event write.");
			p_vsd->status.is_ble_service_has_event = true;
			on_write(p_msg, p_ble_evt);
			break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        	NRF_LOG_INFO("Event read.");
        	p_vsd->status.is_ble_service_has_event = true;
//            on_read(p_msg, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
//        	NRF_LOG_INFO("Tx complete.");
        	p_vsd->status.is_ble_service_has_event = true;
            on_tx_complete(p_msg, p_ble_evt);
            break;
            
		default:
			// No implementation needed.
			break;

	}
}
