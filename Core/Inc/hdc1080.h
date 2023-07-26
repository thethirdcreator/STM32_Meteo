#ifndef __HDC1080_H
#define __HDC1080_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_i2c.h"

#define         HDC_1080_ADDR                            0x40
#define         Configuration_register_addr              0x02
#define         Temperature_register_addr                0x00
#define         Humidity_register_addr                   0x01


typedef enum
{
  Temperature_Resolution_14_bit = 0,
  Temperature_Resolution_11_bit
}Temp_Res;

typedef enum
{
  Humidity_Resolution_14_bit = 0,
  Humidity_Resolution_11_bit = 1,
  Humidity_Resolution_8_bit =2
}Humi_Res;

void hdc1080_init(I2C_HandleTypeDef* hi2c_x,Temp_Res Temperature_Resolution_x_bit,Humi_Res Humidity_Resolution_x_bit);
uint8_t hdc1080_start_measurement(I2C_HandleTypeDef* hi2c_x,float* temperature, uint8_t* humidity);
#endif

