#include "hdc1080.h"


void hdc1080_init(struct __HDC1080_Typedef* HDC1080ptr)
{
	uint16_t config_Reg = HDC1080_CONF_REG_DEFAULT_VALUE;
	uint8_t HDC1080_TX_Buff[2];

	config_Reg = (HDC1080ptr->Humi_Res<<HDC1080_CONF_HRES)|(HDC1080ptr->Temp_Res<<HDC1080_CONF_TRES)|(HDC1080ptr->Heater<<HDC1080_CONF_HEAT)|(HDC1080ptr->Mode<<HDC1080_CONF_MODE);
	HDC1080_TX_Buff[0] = (uint8_t)(config_Reg>>8);
	HDC1080_TX_Buff[1] = (uint8_t)config_Reg;

	HAL_I2C_Mem_Write(HDC1080ptr->hi2c,HDC_1080_ADDR<<1,Configuration_register_addr,I2C_MEMADD_SIZE_8BIT,HDC1080_TX_Buff,sizeof(config_Reg),10);
}

void hdc1080_Measure(struct __HDC1080_Typedef* HDC1080ptr)
{
	uint8_t HDC1080_TX_Buff[2];
	uint8_t HDC1080_RX_Buff[4];

	HDC1080_TX_Buff[0] = Temperature_register_addr;
	HAL_I2C_Master_Transmit(HDC1080ptr->hi2c,HDC_1080_ADDR<<1,HDC1080_TX_Buff,1,10);

	HAL_Delay(50);

	HAL_I2C_Master_Receive(HDC1080ptr->hi2c,HDC_1080_ADDR<<1,HDC1080_RX_Buff,4,50);

	HDC1080ptr->temp = ((((HDC1080_RX_Buff[0]<<8)|HDC1080_RX_Buff[1])*1650)/65535)-400;
	HDC1080ptr->humi = ((((HDC1080_RX_Buff[2]<<8)|HDC1080_RX_Buff[3])*1000/65535));
}
