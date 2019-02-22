#ifndef BLE_PICKIT_BOARD_H
#define BLE_PICKIT_BOARD_H

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "nrf_drv_rtc.h"

#define B_BLE_PICKIT
//#define B_PCA10040

#if defined(B_BLE_PICKIT)

#define LEDS_NUMBER 			3
#define BUTTONS_NUMBER			2

/*
 * For the pin 'BOOTLOADER_ACTIVATION' you have to add the following in the Makefile:
 * CFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
 * ASMFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
 * By default, the NFC pins are enabled. NFC can be disabled and GPIOs enabled by defining
 * the CONFIG_NFCT_PINS_AS_GPIOS.
 */
#define BOOTLOADER_ACTIVATION	10

#define LED_1 					20
#define LED_2 					18
#define LED_3 					13
#define LEDS_LIST 				{ LED_1, LED_2, LED_3 }

#define BUTTON_1				26
#define BUTTON_2				27
#define BUTTONS_LIST 			{ BUTTON_1, BUTTON_2 }
#define BUTTON_PULL    			NRF_GPIO_PIN_PULLUP

#define LEDS_ACTIVE_STATE		0
#define BUTTONS_ACTIVE_STATE 	0

#define PA_PIN              	17
#define LNA_PIN             	19
#define PA_LNA_CS_PIN			6

#define RX_PIN_NUMBER  			2
#define TX_PIN_NUMBER  			3
#define CTS_PIN_NUMBER 			UART_PIN_DISCONNECTED	// not used
#define RTS_PIN_NUMBER 			UART_PIN_DISCONNECTED	// not used

#elif defined(B_PCA10040)

#define LEDS_NUMBER 			4
#define BUTTONS_NUMBER			4

/*
 * For the pin 'BOOTLOADER_ACTIVATION' you have to add the following in the Makefile:
 * CFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
 * ASMFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
 * By default, the NFC pins are enabled. NFC can be disabled and GPIOs enabled by defining
 * the CONFIG_NFCT_PINS_AS_GPIOS.
 */
#define BOOTLOADER_ACTIVATION	10

#define LED_1 					17
#define LED_2 					18
#define LED_3 					19
#define LED_4 					20
#define LEDS_LIST 				{ LED_1, LED_2, LED_3, LED_4 }

#define BUTTON_1				13
#define BUTTON_2				14
#define BUTTON_3				15
#define BUTTON_4				16
#define BUTTONS_LIST 			{ BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 }
#define BUTTON_PULL    			NRF_GPIO_PIN_PULLUP

#define LEDS_ACTIVE_STATE		0
#define BUTTONS_ACTIVE_STATE 	0

#define RX_PIN_NUMBER  			2
#define TX_PIN_NUMBER  			3
#define CTS_PIN_NUMBER 			UART_PIN_DISCONNECTED	// not used
#define RTS_PIN_NUMBER 			UART_PIN_DISCONNECTED	// not used

#endif

nrfx_rtc_t						rtc;

#define mGetTick()				(rtc.p_reg->COUNTER)
#define mTickCompare(var)		(mGetTick() - var)

#define TICK_300US				(10)
#define TICK_400US				(13)
#define TICK_1MS				(33)
#define TICK_2MS				(66)
#define TICK_3MS				(98)
#define TICK_4MS				(131)
#define TICK_5MS				(164)
#define TICK_10MS				(328)
#define TICK_20MS				(655)
#define TICK_30MS				(983)
#define TICK_40MS				(1311)
#define TICK_50MS				(1638)
#define TICK_60MS				(1966)
#define TICK_70MS				(2294)
#define TICK_80MS				(2621)
#define TICK_90MS				(2949)
#define TICK_100MS				(3277)
#define TICK_200MS				(6554)
#define TICK_300MS				(9830)
#define TICK_400MS				(13107)
#define TICK_500MS				(16384)
#define TICK_600MS				(19661)
#define TICK_700MS				(22938)
#define TICK_800MS				(26214)
#define TICK_900MS				(29491)
#define TICK_1S					(32768)
#define TICK_2S					(65536)
#define TICK_3S					(98304)
#define TICK_4S					(131072)
#define TICK_5S					(163840)
#define TICK_6S					(196608)
#define TICK_7S					(229376)
#define TICK_8S					(262144)
#define TICK_9S					(294912)
#define TICK_10S				(327680)

typedef struct
{
	uint8_t 		index;
	uint64_t 		tick;
} state_machine_t;

typedef void (*button_handler_t)(uint8_t pin_no, bool button_action);

uint32_t board_init(button_handler_t evt_handler);
void board_led_set(uint32_t led_pin_no);
void board_led_clr(uint32_t led_pin_no);
void board_led_toggle(uint32_t led_pin_no);
void board_led_lat(uint32_t led_pin_no, bool value);
bool board_led_get(uint32_t led_pin_no);
bool board_button_get(uint32_t button_pin_no);
void board_pa_lna_init(bool enable);

uint8_t fu_integer_value(float val);
uint8_t fu_decimal_value(float val);
float fu_float_value(uint8_t integer, uint8_t decimal);
uint16_t fu_crc_16_ibm(uint8_t *buffer, uint16_t length);

#endif
