#ifndef __HDC1080_H
#define __HDC1080_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_i2c.h"

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


#endif

