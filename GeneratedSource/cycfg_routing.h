/*******************************************************************************
* File Name: cycfg_routing.h
*
* Description:
* Establishes all necessary connections between hardware elements.
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

#if !defined(CYCFG_ROUTING_H)
#define CYCFG_ROUTING_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "cycfg_notices.h"
void init_cycfg_routing(void);
#define init_cycfg_connectivity() init_cycfg_routing()
#define CYBSP_I2C_LIGHT_scl_0_TRIGGER_IN WICED_I2C_1_SCL
#define CYBSP_I2C_LIGHT_sda_0_TRIGGER_IN WICED_I2C_1_SDA
#define CYBSP_PUART_cts_0_TRIGGER_IN WICED_UART_2_CTS
#define CYBSP_PUART_rts_0_TRIGGER_IN WICED_UART_2_RTS
#define CYBSP_PUART_rxd_0_TRIGGER_IN WICED_UART_2_RXD
#define CYBSP_PUART_txd_0_TRIGGER_IN WICED_UART_2_TXD
#define CYBSP_SPI_FLASH_clk_0_TRIGGER_IN WICED_SPI_2_CLK
#define CYBSP_SPI_FLASH_cs_0_TRIGGER_IN WICED_SPI_2_CS
#define CYBSP_SPI_FLASH_miso_0_TRIGGER_IN WICED_SPI_2_MISO
#define CYBSP_SPI_FLASH_mosi_0_TRIGGER_IN WICED_SPI_2_MOSI

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_ROUTING_H */
