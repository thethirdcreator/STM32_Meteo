/*
 * MHZ_19B.h
 *
 *  Created on: Jul 21, 2023
 *      Author: gnaruts
 */

#ifndef INC_MHZ_19B_H_
#define INC_MHZ_19B_H_

#include "stm32f0xx_hal.h"

#define MHZ19_RX_BUFF_SIZE 9
//commands
uint8_t mhz19_cmd_read_co2[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
uint8_t mhz19_cmd_set_detection_range_2000[] = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x07, 0xD0, 0x8F};
uint8_t mhz19_cmd_set_detection_range_3000[] = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x0B, 0xB8, 0xA3};
uint8_t mhz19_cmd_set_detection_range_5000[] = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x13, 0x88, 0xCB};
uint8_t mhz19_cmd_calibrate_zero[] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
uint8_t mhz19_cmd_self_calib_on[] = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6};
uint8_t mhz19_cmd_self_calib_off[] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86};

enum MHZ19_STATE
{
	MHZ19_OK,
	MHZ19_ERROR
};

typedef struct MHZ19
{
	UART_HandleTypeDef huart;
	uint8_t detection_range;
	uint8_t bAllowAutoCalib;
	uint8_t rx_buff[MHZ19_RX_BUFF_SIZE];

};

void MHZ19_Get_CO2();
void MHZ19_Set_Range();
void MHZ19_Calib_Zero();
void MHZ19_Init(struct MHZ19 hMHZ19,UART_HandleTypeDef huart, uint8_t detection_range, uint8_t bAllowAutoCalib);

void MHZ19_Init(struct MHZ19 hMHZ19,UART_HandleTypeDef huart, uint8_t detection_range, uint8_t bAllowAutoCalib)
{
	hMHZ19.huart = huart;
	hMHZ19.detection_range = detection_range;
	hMHZ19.bAllowAutoCalib = bAllowAutoCalib;

	HAL_UART_Transmit_IT(&hMHZ19.huart, hMHZ19.rx_buff, MHZ19_RX_BUFF_SIZE);


}
#endif /* INC_MHZ_19B_H_ */
