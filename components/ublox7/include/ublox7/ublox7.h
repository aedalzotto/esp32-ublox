/**
 * @file ublox7.h
 * 
 * @author
 * Angelo Elias Dalzotto (150633@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 * 
 * @copyright
 * Copyright (C) Angelo Elias Dalzotto. 2018. All Rights MIT Licensed.
 * 
 * @brief This library was created to interface the VK2828U7G5LF (UBX7) GPS
 * module with ESP32 using the IDF-SDK. It includes functions to the UART
 * interface with the programmer's desired parameters and to configure the GPS
 * module output and options. It does not include parser functions yet.
*/

#ifndef UBLOX7_H 
#define UBLOX7_H 

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/**
 * @brief Navigation mode enum.
 * 
 * @details
 *     - Portable (automatic)
 *     - Stationary (time-only)
 *     - Pedestrian/Automotive/Sea
 *     - Airborne (1G, 2G and 4G max modes)
 */
typedef enum _ub7_nav_mode {
    UB7_MODE_PORTABLE,
    UB7_MODE_STATIONARY = 2,
    UB7_MODE_PEDESTRIAN,
    UB7_MODE_AUTOMOTIVE,
    UB7_MODE_SEA,
    UB7_MODE_AIR_1G_MAX,
    UB7_MODE_AIR_2G_MAX,
    UB7_MODE_AIR_4G_MAX
} ub7_nav_mode_t;

/**
 * @brief Baud rate enum.
 * 
 * @details
 *     - 9600 bps is the default
 *     - Currently no baud change supported
 */
typedef enum _ub7_baud_rate_t {
    UB7_BAUD_4800,
    UB7_BAUD_9600,
    UB7_BAUD_19200,
    UB7_BAUD_38400,
    UB7_BAUD_57600,
    UB7_BAUD_115200
} ub7_baud_rate_t;

/**
 * @brief Message output rate enum.
 */
typedef enum _ub7_output_rate_t {
    UB7_OUTPUT_1HZ,
    UB7_OUTPUT_2HZ,
    UB7_OUTPUT_5HZ
} ub7_output_rate_t;

/**
 * @brief NMEA Messages enum.
 */
typedef enum _ub7_message_t {
    UB7_MSG_DTM = 0x0A,
    UB7_MSG_GBS = 0x09,
    UB7_MSG_GGA = 0x00,
    UB7_MSG_GLL = 0x01,
    UB7_MSG_GRS = 0x06,
    UB7_MSG_GSA = 0x02,
    UB7_MSG_GST = 0x07,
    UB7_MSG_GSV = 0x03,
    UB7_MSG_RMC = 0x04,
    UB7_MSG_VTG = 0x05,
    UB7_MSG_ZDA = 0x08
} ub7_message_t;

/**
 * @brief GPS configuration structure.
 * 
 * @details Please do NOT change the structure directly.
 */
typedef struct _ub7_config_t {
    uart_port_t uart_num;
    uart_config_t uart_conf;
} ub7_config_t;

/**
 * @brief Initializes the serial communication for the GPS.
 * 
 * @details PPS pin function not implemented yet.
 * 
 * @param *this is the address of GPS configuration structure.
 * @param uart_num is the ESP32 UART port number.
 * @param baud is one of the possible GPS baud rates.
 * @param tx_pin is the ESP32 TX Pin (Connect to GPS RX).
 * @param rx_pin is the ESP32 RX Pin (Connect to GPS TX).
 * @param en_pin is the GPS enable pin. Manual set to low to power off.
 * 
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t ub7_init(ub7_config_t *this, uart_port_t uart_num,
					ub7_baud_rate_t baud, gpio_num_t tx_pin, 
                        gpio_num_t rx_pin, gpio_num_t en_pin);

/**
 * @brief Sets the GPS navigation mode.
 * 
 * @param *this is the address of the GPS configuration structure.
 * @param mode is the uBlox navigation mode.
 * 
 * @return
 *     - ESP_OK                Success
 *     - ESP_FAIL              Not all bytes sent
 *     - ESP_FAIL              Parameter error
 *     - ESP_ERR_TIMEOUT       Not all bytes read
 *     - ESP_ERR_TIMEOUT       No ACK received within ACK_TIMEOUT
 *     - ESP_INVALID_RESPONSE  NAK received
 *     - ESP_ERR_INVALID_CRC   Checksum for the wrong message received
 */
esp_err_t ub7_set_nav_mode(ub7_config_t *this, ub7_nav_mode_t mode);

/**
 * @brief Sets the GPS message output rate.
 * 
 * @param *this is the address of the GPS configuration structure.
 * @param rate is the uBlox output message frequency.
 * 
 * @return
 *     - ESP_OK                Success
 *     - ESP_ERR_INVALID_ARG   Invalid rate parameter
 *     - ESP_FAIL              Not all bytes sent
 *     - ESP_FAIL              Parameter error
 *     - ESP_ERR_TIMEOUT       Not all bytes read
 *     - ESP_ERR_TIMEOUT       No ACK received within ACK_TIMEOUT
 *     - ESP_INVALID_RESPONSE  NAK received
 *     - ESP_ERR_INVALID_CRC   Checksum for the wrong message received
 */
esp_err_t ub7_set_output_rate(ub7_config_t *this, ub7_output_rate_t rate);

/**
 * @brief Sets the GPS messages to be output.
 * 
 * @param *this is the address of the GPS configuration structure.
 * @param rate is the NMEA message.
 * @param active is if the message will be output.
 * 
 * @return
 *     - ESP_OK                Success
 *     - ESP_FAIL              Not all bytes sent
 *     - ESP_FAIL              Parameter error
 *     - ESP_ERR_TIMEOUT       Not all bytes read
 *     - ESP_ERR_TIMEOUT       No ACK received within ACK_TIMEOUT
 *     - ESP_INVALID_RESPONSE  NAK received
 *     - ESP_ERR_INVALID_CRC   Checksum for the wrong message received
 */
esp_err_t ub7_set_message(ub7_config_t *this, ub7_message_t message, 
                                                            bool active);

#endif

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