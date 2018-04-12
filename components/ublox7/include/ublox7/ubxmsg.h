/**
 * @file ubxmsg.h
 * 
 * @author
 * Angelo Elias Dalzotto (150633@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 * 
 * @copyright
 * Copyright (C) Angelo Elias Dalzotto. 2018. All Rights MIT Licensed.
 * 
 * @brief This is the "private" header file for the uBlox7 library.
 * It includes the configuration bytes definitions and some functions.
 * Please do NOT include this file.
*/

#ifndef UBXMSG_H
#define UBXMSG_H

#include <stdint.h>
#include <esp_err.h>
#include "ublox7.h"

/*!< Timeout for the ACK message */
#define ACK_TIMEOUT 1500

/*!< UBX Protocol headers */
#define UBX_HDR_A 0xB5
#define UBX_HDR_B 0x62

/*!< UBX Message classes */
#define CLS_CFG  0x06
#define CFG_MSG  0x01
#define CFG_NAV5 0x24
#define CLS_ACK  0x05

/*!< NMEA Configuration class */
#define NMEA_STD 0xF0

/*!< ACK Message IDs */
#define ACK_ACK 0x01
#define ACK_NAK 0x00

/*!< CFG Message IDs */
#define CFG_PRT  0x00
#define CFG_RATE 0x08

/**
 * @brief Gets the acknowledge of configuration message.
 * 
 * @param *this is the address of GPS configuration structure.
 * @param cls_id is the class to be acknowledged.
 * @param msg_id is the message to be acknowledged.
 * 
 * @return
 *     - ESP_OK                Success
 *     - ESP_FAIL              Parameter error
 *     - ESP_ERR_TIMEOUT       Not all bytes read
 *     - ESP_ERR_TIMEOUT       No ACK received within ACK_TIMEOUT
 *     - ESP_INVALID_RESPONSE  NAK received
 *     - ESP_ERR_INVALID_CRC   Checksum for the wrong message received
 */
esp_err_t ack_status(ub7_config_t *this, uint8_t cls_id, uint8_t msg_id);

/**
 * @brief Milliseconds passed since task started running.
 * 
 * @return Milliseconds passed since task started running.
 */
uint32_t millis();

/**
 * @brief Adds checksum to the message to be sent.
 * 
 * @details 8-Bit Fletcher Algorithm excluding UBX header.
 */
void add_checksum(uint8_t *message, uint8_t size, uint8_t *CK_A, uint8_t *CK_B);

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