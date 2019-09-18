/*******************************************************************************
* File Name: cycfg_pins.c
*
* Description:
* Pin configuration
* This file was automatically generated and should not be modified.
* 
********************************************************************************
* Copyright 2017-2019 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
********************************************************************************/

#include "cycfg_pins.h"

#define ioss_0_pin_12_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_11].gpio_pin, \
    .config = GPIO_INPUT_ENABLE | GPIO_PULL_UP_DOWN_NONE, \
    .default_state = GPIO_PIN_OUTPUT_LOW, \
 }
#define ioss_0_pin_13_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_12].gpio_pin, \
    .config = GPIO_INPUT_ENABLE | GPIO_PULL_UP_DOWN_NONE, \
    .default_state = GPIO_PIN_OUTPUT_LOW, \
 }
#define CYBSP_SERIN_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_15].gpio_pin, \
    .config = GPIO_INPUT_ENABLE | GPIO_PULL_UP_DOWN_NONE, \
    .default_state = GPIO_PIN_OUTPUT_LOW, \
 }
#define CYBSP_USER_BTN_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_16].gpio_pin, \
    .config = GPIO_INPUT_ENABLE | GPIO_PULL_DOWN, \
    .default_state = GPIO_PIN_OUTPUT_LOW, \
    .button_pressed_value = GPIO_PIN_OUTPUT_HIGH, \
}
#define CYBSP_INT_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_19].gpio_pin, \
    .config = GPIO_INPUT_ENABLE | GPIO_PULL_UP_DOWN_NONE, \
    .default_state = GPIO_PIN_OUTPUT_LOW, \
 }
#define CYBSP_LED_G_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_3].gpio_pin, \
    .config = GPIO_OUTPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
 }
#define CYBSP_LED_B_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_4].gpio_pin, \
    .config = GPIO_OUTPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
 }
#define CYBSP_INT_DOCI_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_5].gpio_pin, \
    .config = GPIO_INPUT_ENABLE | GPIO_PULL_UP_DOWN_NONE, \
    .default_state = GPIO_PIN_OUTPUT_LOW, \
 }
#define CYBSP_LED_R_config \
{\
    .gpio = (wiced_bt_gpio_numbers_t*)&platform_gpio_pins[PLATFORM_GPIO_6].gpio_pin, \
    .config = GPIO_OUTPUT_ENABLE | GPIO_PULL_UP, \
    .default_state = GPIO_PIN_OUTPUT_HIGH, \
 }

const wiced_platform_gpio_t platform_gpio_pins[] = 
{
	[PLATFORM_GPIO_0] = {WICED_P00, CYBSP_PUART_txd_0_TRIGGER_IN},
	[PLATFORM_GPIO_1] = {WICED_P01, CYBSP_PUART_rts_0_TRIGGER_IN},
	[PLATFORM_GPIO_2] = {WICED_P02, CYBSP_SPI_FLASH_clk_0_TRIGGER_IN},
	[PLATFORM_GPIO_3] = {WICED_P03, WICED_GPIO},
	[PLATFORM_GPIO_4] = {WICED_P04, WICED_GPIO},
	[PLATFORM_GPIO_5] = {WICED_P05, WICED_GPIO},
	[PLATFORM_GPIO_6] = {WICED_P06, WICED_GPIO},
	[PLATFORM_GPIO_7] = {WICED_P08, CYBSP_SPI_FLASH_miso_0_TRIGGER_IN},
	[PLATFORM_GPIO_8] = {WICED_P09, WICED_GPIO},
	[PLATFORM_GPIO_9] = {WICED_P10, CYBSP_SPI_FLASH_mosi_0_TRIGGER_IN},
	[PLATFORM_GPIO_10] = {WICED_P11, WICED_GPIO},
	[PLATFORM_GPIO_11] = {WICED_P12, WICED_GPIO},
	[PLATFORM_GPIO_12] = {WICED_P13, WICED_GPIO},
	[PLATFORM_GPIO_13] = {WICED_P14, WICED_GPIO},
	[PLATFORM_GPIO_14] = {WICED_P15, CYBSP_SPI_FLASH_cs_0_TRIGGER_IN},
	[PLATFORM_GPIO_15] = {WICED_P17, WICED_GPIO},
	[PLATFORM_GPIO_16] = {WICED_P26, WICED_GPIO},
	[PLATFORM_GPIO_17] = {WICED_P27, CYBSP_I2C_LIGHT_scl_0_TRIGGER_IN},
	[PLATFORM_GPIO_18] = {WICED_P28, CYBSP_PUART_rxd_0_TRIGGER_IN},
	[PLATFORM_GPIO_19] = {WICED_P29, WICED_GPIO},
	[PLATFORM_GPIO_20] = {WICED_P32, CYBSP_I2C_LIGHT_sda_0_TRIGGER_IN},
	[PLATFORM_GPIO_21] = {WICED_P37, CYBSP_PUART_cts_0_TRIGGER_IN},
};
const wiced_platform_led_config_t platform_led[] = 
{
	[WICED_PLATFORM_LED_2] = CYBSP_LED_G_config,
	[WICED_PLATFORM_LED_3] = CYBSP_LED_B_config,
	[WICED_PLATFORM_LED_1] = CYBSP_LED_R_config,
};
const size_t led_count = (sizeof(platform_led) / sizeof(wiced_platform_led_config_t));
const wiced_platform_button_config_t platform_button[] = 
{
	[WICED_PLATFORM_BUTTON_1] = CYBSP_USER_BTN_config,
};
const size_t button_count = (sizeof(platform_button) / sizeof(wiced_platform_button_config_t));
const wiced_platform_gpio_config_t platform_gpio[] = 
{
	[WICED_PLATFORM_GPIO_4] = CYBSP_INT_DOCI_config,
	[WICED_PLATFORM_GPIO_1] = ioss_0_pin_12_config,
	[WICED_PLATFORM_GPIO_2] = ioss_0_pin_13_config,
	[WICED_PLATFORM_GPIO_3] = CYBSP_SERIN_config,
	[WICED_PLATFORM_GPIO_5] = CYBSP_INT_config,
};
const size_t gpio_count = (sizeof(platform_gpio) / sizeof(wiced_platform_gpio_config_t));

