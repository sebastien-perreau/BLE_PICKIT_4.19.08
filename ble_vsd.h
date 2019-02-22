#ifndef BLE_VSD_H
#define BLE_VSD_H

#include <stdint.h>
#include <stdbool.h>
#include "app_uart.h"
#include "nrf_uart.h"
#include "nrf_uarte.h"

#define ID_PA_LNA					0x01
#define ID_LED_STATUS				0x02
#define ID_SET_NAME					0x03
#define ID_GET_VERSION				0x04
#define ID_ADV_INTERVAL				0x05
#define ID_ADV_TIMEOUT				0x06
#define ID_SOFTWARE_RESET			0xff

#define ID_CHAR_BUFFER              0x30
#define ID_CHAR_SCENARIO            0x40

#define RESET_BLE_PICKIT            0x01
#define RESET_PIC32                 0x02
#define RESET_ALL                   0x03

typedef enum
{
    UART_NO_MESSAGE,
    UART_ACK_MESSAGE,
    UART_NACK_MESSAGE,
    UART_NEW_MESSAGE,
    UART_OTHER_MESSAGE
} BLE_UART_MESSAGE_TYPE;


typedef union
{
    struct
    {
        unsigned 					send_version:1;
        unsigned 					exec_reset:1;
        unsigned 					transfer_ble_to_uart:1;

        unsigned 					notification_buffer:1;
        unsigned 					notification_scenario:1;
    };
    struct
    {
        uint32_t 					w;
    };
} ble_pickit_flags_t;

typedef struct
{
	unsigned 						is_init_done:1;
	unsigned 						is_connected_to_a_central:1;
	unsigned 						is_in_advertising_mode:1;
	unsigned 						is_ble_service_has_event:1;
	unsigned 						is_uart_message_receives:1;

} ble_pickit_status_t;

typedef struct
{
    uint8_t                         data[247];
    uint8_t                         length;
} ble_char_buffer_t;

typedef struct
{
    uint8_t                         index;
} ble_char_scenario_t;

typedef struct
{
    ble_char_buffer_t               buffer;
    ble_char_scenario_t             scenario;
} ble_chars_t;

typedef struct
{
	uint8_t 						id;
	uint8_t 						type;
	uint8_t 						length;
	uint8_t 						data[251];
} ble_serial_message_t;

typedef struct
{
    BLE_UART_MESSAGE_TYPE 			message_type;
	bool 							transmit_in_progress;
	bool 							receive_in_progress;
	uint8_t 						buffer[256];
	uint8_t 						index;
	uint64_t 						tick;
} ble_uart_t;

typedef struct
{
	ble_gap_conn_params_t 			conn_params;
	ble_gap_phys_t					phys_params;
	ble_gap_data_length_params_t	mtu_size_params;
	uint32_t						adv_interval;
	uint32_t						adv_timeout;
} ble_pickit_gap_params;

typedef struct
{
	ble_pickit_gap_params			preferred_gap_params;
	ble_pickit_gap_params			current_gap_params;
	bool							pa_lna_enable;
	bool							leds_status_enable;
} ble_pickit_params;

typedef struct
{
    char 							vsd_version[8];
    char 							device_name[20];
} ble_device_infos_t;

typedef struct
{
	ble_device_infos_t              infos;
	ble_pickit_params				params;
	ble_uart_t                      uart;
	ble_serial_message_t            incoming_uart_message;
	ble_serial_message_t			outgoing_uart_message;
	ble_chars_t						characteristic;
	ble_pickit_flags_t        		flags;
	ble_pickit_status_t				status;
} ble_pickit_t;


#define BLE_START_CONN_PARAMS_INSTANCE()						\
{																\
	.min_conn_interval 	= MSEC_TO_UNITS(15, UNIT_1_25_MS),		\
	.max_conn_interval 	= MSEC_TO_UNITS(15, UNIT_1_25_MS),		\
	.slave_latency 		= 0,									\
	.conn_sup_timeout 	= MSEC_TO_UNITS(4000, UNIT_10_MS),		\
}

#define BLE_START_PHYS_PARAMS_INSTANCE()						\
{																\
	.rx_phys 			= BLE_GAP_PHY_2MBPS,					\
	.tx_phys 			= BLE_GAP_PHY_2MBPS,					\
}

#define BLE_START_MTU_SIZE_PARAMS_INSTANCE()					\
{																\
	.max_tx_octets 		= NRF_SDH_BLE_GATT_MAX_MTU_SIZE + 4U,	\
	.max_rx_octets 		= NRF_SDH_BLE_GATT_MAX_MTU_SIZE + 4U,	\
	.max_tx_time_us 	= BLE_GAP_DATA_LENGTH_AUTO,				\
	.max_rx_time_us 	= BLE_GAP_DATA_LENGTH_AUTO,				\
}

#define BLE_PICKIT_GAP_PARAMS_INSTANCE()						\
{																\
	.conn_params = BLE_START_CONN_PARAMS_INSTANCE(),			\
	.phys_params = BLE_START_PHYS_PARAMS_INSTANCE(),			\
	.mtu_size_params = BLE_START_MTU_SIZE_PARAMS_INSTANCE(),	\
	.adv_interval = MSEC_TO_UNITS(100, UNIT_0_625_MS),			\
	.adv_timeout = 18000,										\
}

#define BLE_PICKIT_PARAMS_INSTANCE()							\
{																\
	.preferred_gap_params = BLE_PICKIT_GAP_PARAMS_INSTANCE(),	\
	.current_gap_params = {{0}, {0}, {0}, 0, 0},				\
	.pa_lna_enable = false,										\
	.leds_status_enable = true,									\
}

#define BLE_DEVICE_INFOS_INSTANCE(_name, _version)       		\
{                                                       		\
    .vsd_version = {_version},                     				\
    .device_name = {_name},                              		\
}

#define BLE_PICKIT_INSTANCE(_name, _version)          			\
{                                                       		\
	.infos = BLE_DEVICE_INFOS_INSTANCE(_name, _version),		\
	.params = BLE_PICKIT_PARAMS_INSTANCE(),						\
	.uart = {0},                                        		\
	.incoming_uart_message = {0},                            	\
	.outgoing_uart_message = {0},                            	\
	.characteristic = {{{0}}, {0}},								\
	.flags = {{0}},                                    			\
	.status = {0},												\
}

#define BLE_PICKIT_DEF(_var, _name, _version)					\
static ble_pickit_t _var = BLE_PICKIT_INSTANCE(_name, _version)

typedef void (*p_function)(uint8_t *buffer);

void ble_init(ble_pickit_t * p_vsd_params);
void ble_stack_tasks();

#endif
