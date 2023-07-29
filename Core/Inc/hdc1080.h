#ifndef __HDC1080_H
#define __HDC1080_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_i2c.h"

//#define bHDC1080UseFloatCalculation

#define         HDC_1080_ADDR                            0x40
#define         Configuration_register_addr              0x02
#define         Temperature_register_addr                0x00
#define         Humidity_register_addr                   0x01

#define HDC1080_CONF_REG_DEFAULT_VALUE 0x1000

#define HDC1080_CONF_HRES 8
#define HDC1080_CONF_TRES 10
#define HDC1080_CONF_BTST 11
#define HDC1080_CONF_MODE 12
#define HDC1080_CONF_HEAT 13
#define HDC1080_CONF_RST  15


typedef struct __HDC1080_Typedef
{
	I2C_HandleTypeDef *hi2c;
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
	enum
	{
		NotUseHeater,
		UseHeater
	}Heater;
	enum
	{
		SingleAquisition,
		AquireInSequence
	}Mode;

	int32_t temp;
	uint32_t humi;

}HDC1080_Typedef;

void hdc1080_init(struct __HDC1080_Typedef*);
void hdc1080_Measure(struct __HDC1080_Typedef*);

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

