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

	HAL_Delay(15);

	HAL_I2C_Master_Receive(HDC1080ptr->hi2c,HDC_1080_ADDR<<1,HDC1080_RX_Buff,4,10);

	HDC1080ptr->temp = ((((HDC1080_RX_Buff[0]<<8)|HDC1080_RX_Buff[1])*1650)/65535)-400;
	HDC1080ptr->humi=(uint8_t)((((HDC1080_RX_Buff[0]<<8)|HDC1080_RX_Buff[1])*100/65536));
}

/*void hdc1080_init(I2C_HandleTypeDef* hi2c_x,HDC1080_Temp_Res Temperature_Resolution_x_bit,HDC1080_Humi_Res Humidity_Resolution_x_bit)
{
	 Temperature and Humidity are acquired in sequence, Temperature first
	 * Default:   Temperature resolution = 14 bit,
	 *            Humidity resolution = 14 bit


	 Set the acquisition mode to measure both temperature and humidity by setting Bit[12] to 1
	uint16_t config_reg_value=0x1000;
	uint8_t data_send[2];

	if(Temperature_Resolution_x_bit == Temperature_Resolution_11_bit)
	{
		config_reg_value |= (1<<10); //11 bit
	}

	switch(Humidity_Resolution_x_bit)
	{
	case Humidity_Resolution_11_bit:
		config_reg_value|= (1<<8);
		break;
	case Humidity_Resolution_8_bit:
		config_reg_value|= (1<<9);
		break;
	default:
		break;
	}

	data_send[0]= (config_reg_value>>8);
	data_send[1]= (config_reg_value&0x00ff);

	HAL_I2C_Mem_Write(hi2c_x,HDC_1080_ADDR<<1,Configuration_register_addr,I2C_MEMADD_SIZE_8BIT,data_send,2,1000);
}*/

/*
void hdc1080_get_t(I2C_HandleTypeDef* hi2c_x,int32_t* temperature)
{

}
void hdc1080_get_t(I2C_HandleTypeDef* hi2c_x,uint8_t* humidity)
{

}

uint8_t hdc1080_start_measurement(I2C_HandleTypeDef* hi2c_x,float* temperature, uint8_t* humidity)
{
	uint8_t receive_data[4];
	uint16_t temp_x,humi_x;
	uint8_t send_data = Temperature_register_addr;
	int32_t temp_int = 0;

	HAL_I2C_Master_Transmit(hi2c_x,HDC_1080_ADDR<<1,&send_data,1,1000);

	 Delay here 15ms for conversion compelete.
	 * Note: datasheet say maximum is 7ms, but when delay=7ms, the read value is not correct

	HAL_Delay(15);

	 Read temperature and humidity
	HAL_I2C_Master_Receive(hi2c_x,HDC_1080_ADDR<<1,receive_data,4,1000);

	temp_x =((receive_data[0]<<8)|receive_data[1]);
	humi_x =((receive_data[2]<<8)|receive_data[3]);

	*temperature=((temp_x/65536.0)*165.0)-40.0;
	temp_int = ((temp_x*1650)/65535)-400;
	*humidity=(uint8_t)((humi_x/65536.0)*100.0);

	return 0;

}
*/
