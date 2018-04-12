/**
 * @file example.c
 * 
 * @author
 * Angelo Elias Dalzotto (150633@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 * 
 * @copyright
 * Copyright (C) Angelo Elias Dalzotto. 2018. All Rights MIT Licensed.
 * 
 * @brief This is and example code to the uBlox7 GPS library.
 * It initializes the GPS module with pedestrian mode, deactivating RMC, VTG,
 * GSA, GLL and GSV, setting the output of the messages to 1Hz.
 * The output of the GPS is directly written to the serial monitor.
*/

/**
 * Pin assignment:
 * - UART_NUM_1:
 *      GPIO16: RXD
 *      GPIO17: TXD
 *      GPIO5:   EN
 *      GPIO18: PPS
 */

#include <stdio.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include "ublox7/ublox7.h"

#define UB7_RXD GPIO_NUM_17
#define UB7_TXD GPIO_NUM_16
#define UB7_EN GPIO_NUM_5
#define UB7_PPS GPIO_NUM_18

void gps_task()
{
    ub7_config_t gps;
    ESP_ERROR_CHECK(ub7_init(&gps, UART_NUM_1, UB7_BAUD_9600, UB7_TXD, UB7_RXD, UB7_EN));
    ESP_ERROR_CHECK(ub7_set_nav_mode(&gps, UB7_MODE_PEDESTRIAN));
    ESP_ERROR_CHECK(ub7_set_message(&gps, UB7_MSG_RMC, false));
    ESP_ERROR_CHECK(ub7_set_message(&gps, UB7_MSG_VTG, false));
    ESP_ERROR_CHECK(ub7_set_message(&gps, UB7_MSG_GSA, false));
    ESP_ERROR_CHECK(ub7_set_message(&gps, UB7_MSG_GLL, false));
    ESP_ERROR_CHECK(ub7_set_message(&gps, UB7_MSG_GSV, false));
    ESP_ERROR_CHECK(ub7_set_output_rate(&gps, UB7_OUTPUT_1HZ));
    
    uint8_t data;
    while (1){
        // Read data from the UART
        if(uart_read_bytes(UART_NUM_1, &data, 1, 20/portTICK_RATE_MS))
            printf("%c", data);
        // Write data back to the Monitor
    }
}

void app_main() {
    xTaskCreate(gps_task, "gps test", 2048, NULL, 10, NULL);
}

/**
 * MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/