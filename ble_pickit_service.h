#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

// <o> BLE_MSG_BLE_OBSERVER_PRIO  
// <i> Priority with which BLE events are dispatched to the Message Service.
#ifndef BLE_MSG_BLE_OBSERVER_PRIO
#define BLE_MSG_BLE_OBSERVER_PRIO 2
#endif

/**@brief   Macro for defining a ble_msg instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_PICKIT_SERVICE_DEF(_name)      					\
static ble_msg_t _name;										\
NRF_SDH_BLE_OBSERVER(_name ## _obs, BLE_MSG_BLE_OBSERVER_PRIO, ble_pickit_service_event_handler, &_name)

// a7bbdefd-eef2-4a8e-80d4-13a83c8cf46f
#define MESSAGE_SERVICE_UUID_BASE         	{0x6F, 0xF4, 0x8C, 0x3C, 0xa8, 0x13, 0xD4, 0x80, 0x8E, 0x4A, 0xF2, 0xEE, 0xFD, 0xDE, 0xBB, 0xA7}

#define MESSAGE_SERVICE_UUID               	0x1500
#define MESSAGE_APP_CHAR_UUID          		0x1501		// (Notification / Write)
#define MESSAGE_TEST_UUID        			0x1502		// (Notification / Write)
#define MESSAGE_PARAMS_UUID					0x1503		// (Notification / Write)

/**@brief Message Service event type. */
typedef enum
{
	SERVICE_EVT_DISCONNECTED,
	SERVICE_EVT_CONNECTED,

	SERVICE_EVT_APP_NOTIFICATION_ENABLED,
    SERVICE_EVT_APP_NOTIFICATION_DISABLED,
	SERVICE_EVT_TEST_NOTIFICATION_ENABLED,
	SERVICE_EVT_TEST_NOTIFICATION_DISABLED,
	SERVICE_EVT_PARAMS_NOTIFICATION_ENABLED,
	SERVICE_EVT_PARAMS_NOTIFICATION_DISABLED,

	SERVICE_EVT_APP_WRITE,
	SERVICE_EVT_TEST_WRITE,
	SERVICE_EVT_PARAMS_WRITE,
} service_evt_type_t;

/**@brief Message Service event. */
typedef struct
{
	service_evt_type_t evt_type;                                  /**< Type of event. */
} ble_msg_evt_t;

// Forward declaration of the ble_msg_t type.
typedef struct ble_msg_s ble_msg_t;

/**@brief Message Service event handler type. */
typedef void (*ble_msg_evt_handler_t) (ble_msg_t * p_msg, ble_msg_evt_t * p_evt, const uint8_t *buffer, uint8_t length);

/**@brief Message Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_msg_evt_handler_t         evt_handler;             			/**< Event handler to be called for handling events in the Message Service. */

    ble_srv_cccd_security_mode_t  char_app_security;     			/**< Initial security level for Message characteristics attribute */
    ble_srv_cccd_security_mode_t  char_test_security;   			/**< Initial security level for Message characteristics attribute */
    ble_srv_cccd_security_mode_t  char_params_security;     		/**< Initial security level for Message characteristics attribute */
} ble_msg_init_t;


typedef struct
{
	uint16_t					indice;
	uint64_t					elapsed_time_ms;
	uint32_t					bytes_transmitted;
	float 						current_data_rate_kbps;
	float						average_data_rate_kbps;
	uint8_t						min_packets_per_interval_of_conn;
	uint8_t						max_packets_per_interval_of_conn;

	uint8_t						_type;
	uint64_t					_start_time_ms;
	uint32_t					_end_count;
	state_machine_t				_sm;

} _throughput_t;

typedef struct
{
	bool						change_conn_params_request;
	bool						change_phy_param_request;
	bool						change_mtu_size_params_request;

	bool						notify_all_parameters;
} _ble_params_t;

typedef struct
{
	ble_gatts_char_handles_t	handles;							/**< Handles related to the Message characteristic. */
	bool						is_notification_enabled;
	uint8_t						notifications_on_going;				/**< Number of notifications on going to be sent. */
} ble_characteristics_t;


/**@brief Message Service structure. This contains various status information for the service. */
struct ble_msg_s
{
	ble_msg_evt_handler_t     	evt_handler;                  		/**< Event handler to be called for handling events in the Message Service. */

	ble_characteristics_t		char_app;
	ble_characteristics_t		char_test;
	ble_characteristics_t		char_params;

    uint16_t                  	service_handle;               		/**< Handle of Message Service (as provided by the BLE stack). */
    uint16_t                  	conn_handle;                 		/**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                  	uuid_type;

    uint16_t					att_payload;

    _throughput_t				throughput;
    _ble_params_t				ble_params;

};

typedef enum
{
	THROUGHPUT_STOP_TEST		= 1,
	THROUGHPUT_LAUNCH_TEST_1KB 	= 2,
	THROUGHPUT_LAUNCH_TEST_1MB	= 3,
	THROUGHPUT_LAUNCH_TEST_60S	= 4,
} _THROUGHPUT_COMMANDS;

typedef void (*p_function)(uint8_t *buffer);

void ble_pickit_service_set_link_with_vsd(ble_pickit_t * p);
/**@brief Function for initializing the Message Service.
 *
 * @param[out]  p_msg       Message Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_msg_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_pickit_service_init(ble_msg_t * p, const ble_msg_init_t * p_msg_init);

uint32_t add_characteristic_app_0x1501(ble_msg_t * p_msg, const ble_msg_init_t * p_msg_init);
uint32_t add_characteristic_test_0x1502(ble_msg_t * p_msg, const ble_msg_init_t * p_msg_init);
uint32_t add_characteristic_params_0x1503(ble_msg_t * p_msg, const ble_msg_init_t * p_msg_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note 
 *
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 * @param[in]   p_context  Message Service structure.
 */
void ble_pickit_service_event_handler(ble_evt_t const * p_ble_evt, void * p_context);

void ble_pickit_throughput_notification_send(ble_msg_t * p_msg);
void ble_pickit_parameters_notification_send();
uint8_t ble_pickit_app_notification_send(p_function ptr);

