#ifndef __HDC1080_H
#define __HDC1080_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_i2c.h"

//#define bHDC1080UseFloatCalculation

#define         HDC_1080_ADDR                            0x40
#define         Configuration_register_addr              0x02
#define         Temperature_register_addr                0x00
#define         Humidity_register_addr                   0x01

#define HDC1080_HRES 0x8
#define HDC1080_TRES 0x10
#define HDC1080_BTST 0x11
#define HDC1080_MODE 0x12
#define HDC1080_HEAT 0x13
#define HDC1080_RST  0x15


typedef struct __HDC1080_Typedef
{
	I2C_HandleTypeDef hi2c;
	enum
	{
	  Temperature_Resolution_14_bit,
	  Temperature_Resolution_11_bit
	}Temp_Res;
	enum
	{
	  Humidity_Resolution_14_bit,
	  Humidity_Resolution_11_bit,
	  Humidity_Resolution_8_bit
	}Humi_Res;

	unsigned int bUseHeater : 1;
	unsigned int bAquireInSequence : 1;

	int32_t temp;
	uint32_t humi;

}HDC1080_Typedef;

void hdc1080_init(struct __HDC1080_Typedef*);

/*
void hdc1080_init(I2C_HandleTypeDef*,Temp_Res Temperature_Resolution_x_bit,HDC1080_Humi_Res Humidity_Resolution_x_bit);
#ifdef bHDC1080UseFloatCalculation
uint8_t hdc1080_start_measurement(I2C_HandleTypeDef*,float*, uint8_t*);
#else
uint8_t hdc1080_start_measurement(I2C_HandleTypeDef*,int32_t*, uint8_t*);
void hdc1080_get_t(I2C_HandleTypeDef*,int32_t*);
void hdc1080_get_t(I2C_HandleTypeDef*,uint8_t*);
#endif
*/

#endif

