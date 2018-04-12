/**
 * @file ublox7.c
 * 
 * @author
 * Angelo Elias Dalzotto (150633@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 * 
 * @copyright
 * Copyright (C) Angelo Elias Dalzotto. 2018. All Rights MIT Licensed.
 * 
 * @brief This is the source code to the uBlox7 GPS library for ESP32.
*/

#include <driver/gpio.h>
#include "ublox7/ublox7.h"
#include "ublox7/ubxmsg.h"

esp_err_t ub7_init(ub7_config_t *this, uart_port_t uart_num, 
					ub7_baud_rate_t baud, gpio_num_t tx_pin, 
						gpio_num_t rx_pin, gpio_num_t en_pin)
{

	gpio_set_direction(en_pin, GPIO_MODE_OUTPUT);
	gpio_set_level(en_pin, true);

	this->uart_num = uart_num;

	uint32_t baud_rate;
	switch(baud){
	case UB7_BAUD_4800:
		baud_rate = 4800;
		break;
	case UB7_BAUD_19200:
		baud_rate = 19200;
		break;
	case UB7_BAUD_38400:
		baud_rate = 38400;
		break;
	case UB7_BAUD_57600:
		baud_rate = 57600;
		break;
	case UB7_BAUD_115200:
		baud_rate = 115200;
		break;
	case UB7_BAUD_9600:
	default:
		baud_rate = 9600;
		break;
	}

    this->uart_conf.baud_rate = baud_rate;
    this->uart_conf.data_bits = UART_DATA_8_BITS;
    this->uart_conf.parity    = UART_PARITY_DISABLE;
    this->uart_conf.stop_bits = UART_STOP_BITS_1;
    this->uart_conf.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

	uart_param_config(uart_num, &(this->uart_conf));
	uart_set_pin(uart_num, tx_pin, rx_pin, 
					UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	return uart_driver_install(uart_num, 2048, 0, 0, NULL, 0);

}

esp_err_t ub7_set_nav_mode(ub7_config_t *this, ub7_nav_mode_t mode)
{
	uint8_t nav_msg[44] = {UBX_HDR_A, UBX_HDR_B, CLS_CFG, CFG_NAV5, 
                                    /* payload size */ 0x24, 0x00,
                                    /* mask */ 0xFF, 0xFF,
                                    mode, /* auto 2D-3D */ 0x03,
                                    /* fixedAlt */ 0x00, 0x00, 0x00, 0x00,
                                    /* fixedAltVar */ 0x10, 0x27, 0x00, 0x00,
                                    /* minElev */ 0x05, /* drLimit */ 0x00,
                                    /* pDop */ 0xFA, 0x00,
                                    /* tDop */ 0xFA, 0x00,
                                    /* pAcc */ 0x64, 0x00,
                                    /* tAcc */ 0x2C, 0x01,
                                    /* staticHoldThresh */ 0x00,
                                    /* dgpsTimeOut */ 0x00,
                                    /* cnoThreshNumSVs */ 0x00,
                                    /* cnoThresh */ 0x00,
                                    /* reserved */ 0x00, 0x00, 0x00, 0x00, 0x00,
                                    /* reserved */ 0x00, 0x00, 0x00, 0x00, 0x00,
                                    /* chksum */ 0x00, 0x00};

	add_checksum(nav_msg, sizeof(nav_msg), 
					&nav_msg[sizeof(nav_msg)-2], &nav_msg[sizeof(nav_msg)-1]);
	
	if(uart_write_bytes(this->uart_num, (char*)nav_msg,
						 sizeof(nav_msg)) != sizeof(nav_msg))
		return ESP_FAIL;

	return ack_status(this, CLS_CFG, CFG_NAV5);

}

esp_err_t ub7_set_output_rate(ub7_config_t *this, ub7_output_rate_t rate)
{
	uint8_t output_vec[2];

	switch(rate){
	case UB7_OUTPUT_1HZ:
		output_vec[0] = 0xE8;
		output_vec[1] = 0x03;
		break;
	case UB7_OUTPUT_2HZ:
		output_vec[0] = 0xF4;
		output_vec[1] = 0x01;
		break;
	case UB7_OUTPUT_5HZ:
		output_vec[0] = 0xC8;
		output_vec[1] = 0x00;
		break;
	default:
		return ESP_ERR_INVALID_ARG;
	}

	uint8_t out_msg[14] = {UBX_HDR_A, UBX_HDR_B, CLS_CFG, CFG_RATE, 0x06, 0x00,
							output_vec[0], output_vec[1],
							/* navRate always 1 */ 0x01, 0x00,
							/* timeRef UTC */ 0x01, 0x00,
							/* checksum */ 0x00, 0x00};

	add_checksum(out_msg, 14, &out_msg[12], &out_msg[13]);

	if(uart_write_bytes(this->uart_num, (char*)out_msg,
						sizeof(out_msg)) != sizeof(out_msg))
		return ESP_FAIL;

	return ack_status(this, CLS_CFG, CFG_RATE);

}

esp_err_t ub7_set_message(ub7_config_t *this, ub7_message_t message,
															bool active)
{
	uint8_t msg_vec[11] = {UBX_HDR_A, UBX_HDR_B, CLS_CFG, CFG_MSG, 0x03, 0x00,
							NMEA_STD, message, active, 0x00, 0x00};
	add_checksum(msg_vec, sizeof(msg_vec),
					&msg_vec[sizeof(msg_vec)-2], &msg_vec[sizeof(msg_vec)-1]);

	if(uart_write_bytes(this->uart_num, (char*)msg_vec,
							sizeof(msg_vec)) != sizeof(msg_vec))
		return ESP_FAIL;

	return ack_status(this, CLS_CFG, CFG_MSG);
}

/*
esp_err_t ub7_set_baud_rate(ub7_config_t *this, ub7_baud_rate_t baud)
{
	uint8_t baud_vec[3];
	uint32_t baud_rate;
	switch(baud){
		case UB7_BAUD_4800:
			baud_vec[0] = 0xC0;
			baud_vec[1] = 0x12;
			baud_vec[2] = 0x00;
			baud_rate = 4800;
			break;
		case UB7_BAUD_9600:
			baud_vec[0] = 0x80;
			baud_vec[1] = 0x25;
			baud_vec[2] = 0x00;
			baud_rate = 9600;
			break;
		case UB7_BAUD_19200:
			baud_vec[0] = 0x00;
			baud_vec[1] = 0x4B;
			baud_vec[2] = 0x00;
			baud_rate = 19200;
			break;
		case UB7_BAUD_38400:
			baud_vec[0] = 0x00;
			baud_vec[1] = 0x96;
			baud_vec[2] = 0x00;
			baud_rate = 38400;
			break;
		case UB7_BAUD_57600:
			baud_vec[0] = 0x00;
			baud_vec[1] = 0xE1;
			baud_vec[2] = 0x00;
			baud_rate = 57600;
			break;
		case UB7_BAUD_115200:
			baud_vec[0] = 0x00;
			baud_vec[1] = 0xC2;
			baud_vec[2] = 0x01;
			baud_rate = 115200;
			break;
		default:
			return ESP_ERR_INVALID_ARG;
	}
*/

//	uint8_t prt_msg[32] = {UBX_HDR_A, UBX_HDR_B, CLS_CFG, CFG_PRT, 0x14, 0x00,
//							/* uart */ 0x01, /* reserved0 */ 0x00,
//							/* txReady off */ 0x00, 0x00,
//							/* 8N1 No parity */ 0xD0, 0x08, 0x00, 0x00,
//							baud_vec[0], baud_vec[1], baud_vec[2], 0x00,
//							/* inProtoMask all */ 0x07, 0x00,
//							/* outProtoMask all */ 0x07, 0x00,
//							/* flags no ext timeout */ 0x00, 0x00, 
//							/* reserved5 */ 0x00, 0x00, 
//							/* checksum */ 0x00, 0x00};

/*
	add_checksum(prt_msg, 32, &prt_msg[30], &prt_msg[31]);

	if(uart_write_bytes(this->uart_num, (char*)prt_msg,
						sizeof(prt_msg)) != sizeof(prt_msg))
		return ESP_FAIL;

	esp_err_t ret;

	uint32_t old_baud = this->uart_conf.baud_rate;
	this->uart_conf.baud_rate = baud_rate;

	uart_driver_delete(this->uart_num);
	uart_param_config(this->uart_num, &(this->uart_conf));
	return uart_driver_install(this->uart_num, 2048, 0, 0, NULL, 0);
	

 	ret = ack_status(this, CLS_CFG, CFG_PRT);
	if(ret){
		uart_set_baudrate(this->uart_num, this->baud);
		return ret;
	}


	return ESP_OK;

}
*/

esp_err_t ack_status(ub7_config_t *this, uint8_t cls_id, uint8_t msg_id)
{
	esp_err_t ret;
	uint8_t data = 0;
	uint8_t i = 0;
	size_t size = 0;
	uint8_t ack_pkt[10] = {UBX_HDR_A, UBX_HDR_B, CLS_ACK, ACK_ACK, 0x02, 0x00,
							cls_id, msg_id, 0x00, 0x00};

	uint32_t then = millis();
	while(i<10 && millis() - then < ACK_TIMEOUT){
		ret = uart_get_buffered_data_len(this->uart_num, &size);
		if(ret)
			return ret;
		if(size){
			if(!uart_read_bytes(this->uart_num, &data, 1, 20/portTICK_RATE_MS))
				return ESP_ERR_TIMEOUT;

			if(data == ack_pkt[i])
				i++;
			else if(i > 2){
				ack_pkt[i] = data;
				i++;
			}
		}
	}
	if(millis() - then >= 1500)
		return ESP_ERR_TIMEOUT;
	if(!ack_pkt[3])
		return ESP_ERR_INVALID_RESPONSE;	

	uint8_t CK_A = 0;
	uint8_t CK_B = 0;
	add_checksum(ack_pkt, 10, &CK_A, &CK_B);

	if(cls_id == ack_pkt[6] && msg_id == ack_pkt[7] && 
					CK_A == ack_pkt[8] && CK_B == ack_pkt[9])
		return ESP_OK;
	else
		return ESP_ERR_INVALID_CRC;
}

void add_checksum(uint8_t *message, uint8_t size, uint8_t *CK_A, uint8_t *CK_B)
{
	for(uint8_t i = 2; i < size-2; i++){
		*CK_A = *CK_A + message[i];
		*CK_B = *CK_B + *CK_A;
	}
}

inline uint32_t millis()
{
	return xTaskGetTickCount() * portTICK_PERIOD_MS;
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