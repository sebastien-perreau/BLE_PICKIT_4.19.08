#include "sdk_common.h"
#include "ble_pickit_board.h"

#include "nordic_common.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "app_timer.h"
#include "ble.h"

static const uint8_t m_board_led_list[LEDS_NUMBER] = LEDS_LIST;
static const uint8_t m_board_btn_list[BUTTONS_NUMBER] = BUTTONS_LIST;
static button_handler_t m_button_handler;
static uint64_t m_pin_state;
static uint64_t m_pin_transition;
APP_TIMER_DEF(m_timer_id);

static void timer_handler(void * p_context)
{
	uint8_t i;

	for (i = 0; i < BUTTONS_NUMBER; i++)
	{
		uint64_t btn_mask = 1ULL << m_board_btn_list[i];
		if (btn_mask & m_pin_transition)
		{
			m_pin_transition &= ~btn_mask;
			bool pin_is_set = nrf_drv_gpiote_in_is_set(m_board_btn_list[i]);
			if ((m_pin_state & (1ULL << m_board_btn_list[i])) == (((uint64_t)pin_is_set) << m_board_btn_list[i]))
			{

				if (m_button_handler)
				{
					m_button_handler(m_board_btn_list[i], !(pin_is_set));
				}
			}
		}
	}
}

static void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	uint32_t err_code;
	uint64_t pin_mask = 1ULL << pin;

	err_code = app_timer_stop(m_timer_id);
	if (err_code != NRF_SUCCESS)
	{
		return;
	}

	if (!(m_pin_transition & pin_mask))
	{
		if (nrf_drv_gpiote_in_is_set(pin))
		{
			m_pin_state |= pin_mask;
		}
		else
		{
			m_pin_state &= ~(pin_mask);
		}
		m_pin_transition |= (pin_mask);

		err_code = app_timer_start(m_timer_id, 2500, NULL);
	}
	else
	{
		m_pin_transition &= ~pin_mask;
	}
}

uint32_t board_init(button_handler_t evt_handler)
{
	uint32_t err_code;
	uint8_t i;

	m_button_handler = evt_handler;

	nrf_gpio_cfg_output(BOOTLOADER_ACTIVATION);
	nrf_gpio_pin_clear(BOOTLOADER_ACTIVATION);

	for (i = 0; i < LEDS_NUMBER; ++i)
	{
		nrf_gpio_cfg_output(m_board_led_list[i]);
		nrf_gpio_pin_set(m_board_led_list[i]);
	}

	for (i = 0; i < BUTTONS_NUMBER; ++i)
	{
		nrf_gpio_cfg_input(m_board_btn_list[i], BUTTON_PULL);
	}

	if (!nrf_drv_gpiote_is_init())
	{
		err_code = nrf_drv_gpiote_init();
		VERIFY_SUCCESS(err_code);
	}

	nrf_drv_gpiote_in_config_t config;
	config.is_watcher = false;
	config.hi_accuracy = false;
	config.pull = BUTTON_PULL;
	config.sense = NRF_GPIOTE_POLARITY_TOGGLE;

	for (i = 0; i < BUTTONS_NUMBER; ++i)
	{
		err_code = nrf_drv_gpiote_in_init(m_board_btn_list[i], &config, gpiote_event_handler);
		VERIFY_SUCCESS(err_code);
	}

	err_code = app_timer_create(&m_timer_id, APP_TIMER_MODE_SINGLE_SHOT, timer_handler);
	VERIFY_SUCCESS(err_code);

	for (i = 0; i < BUTTONS_NUMBER; i++)
	{
		nrf_drv_gpiote_in_event_enable(m_board_btn_list[i], true);
	}

	return err_code;
}

void board_led_set(uint32_t led_pin_no)
{
	nrf_gpio_pin_clear(led_pin_no);
}

void board_led_clr(uint32_t led_pin_no)
{
	nrf_gpio_pin_set(led_pin_no);
}

void board_led_toggle(uint32_t led_pin_no)
{
	nrf_gpio_pin_toggle(led_pin_no);
}

void board_led_lat(uint32_t led_pin_no, bool value)
{
	(value) ? nrf_gpio_pin_clear(led_pin_no) : nrf_gpio_pin_set(led_pin_no);
}

bool board_led_get(uint32_t led_pin_no)
{
	return (nrf_gpio_pin_out_read(led_pin_no) ? 0 : 1);
}

bool board_button_get(uint32_t button_pin_no)
{
	return (nrf_gpio_pin_read(button_pin_no) ? 0 : 1);
}

void board_pa_lna_init(bool enable)
{
#if defined(B_BLE_PICKIT)

	if (enable)
	{
		ble_opt_t pa_lna_config;
		uint32_t err_code;

		nrf_gpio_cfg_output(PA_LNA_CS_PIN);
		nrf_gpio_cfg_output(PA_PIN);
		nrf_gpio_cfg_output(LNA_PIN);

		nrf_gpio_pin_clear(PA_LNA_CS_PIN);
		nrf_gpio_pin_clear(PA_PIN);
		nrf_gpio_pin_clear(LNA_PIN);

		pa_lna_config.common_opt.pa_lna.pa_cfg.enable = 1;
		pa_lna_config.common_opt.pa_lna.pa_cfg.active_high = 1;
		pa_lna_config.common_opt.pa_lna.pa_cfg.gpio_pin = PA_PIN;

		pa_lna_config.common_opt.pa_lna.lna_cfg.enable = 1;
		pa_lna_config.common_opt.pa_lna.lna_cfg.active_high = 1;
		pa_lna_config.common_opt.pa_lna.lna_cfg.gpio_pin = LNA_PIN;

		pa_lna_config.common_opt.pa_lna.ppi_ch_id_set = 0;
		pa_lna_config.common_opt.pa_lna.ppi_ch_id_clr = 1;
		pa_lna_config.common_opt.pa_lna.gpiote_ch_id = 0;

		NRF_GPIO->DIRSET |= (1 << PA_PIN) | (1 << LNA_PIN) ;

		err_code = sd_ble_opt_set(BLE_COMMON_OPT_PA_LNA, &pa_lna_config);
		APP_ERROR_CHECK(err_code);
	}
#endif
}

uint8_t fu_integer_value(float v)
{
    return (uint8_t) v;
}

uint8_t fu_decimal_value(float v)
{
	uint8_t integer = (uint8_t) v;

    return (uint8_t) ((v - (float) integer) * 100.0);
}

float fu_float_value(uint8_t integer, uint8_t decimal)
{
    return (float) (integer + decimal/100.0);
}

uint16_t fu_crc_16_ibm(uint8_t *buffer, uint16_t length)
{
	uint16_t crc = 0;
	uint16_t l;

	while (length--)
	{
	    crc ^= *buffer++;
	    for (l = 0 ; l < 8 ; l++)
	    {
	        crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
	    }
	}

	return crc;
}
