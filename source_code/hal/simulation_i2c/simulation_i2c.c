#include "simulation_i2c.h"


static void i2c_delay()
{
	uint32_t i = 20;
	while(--i);
}



static void simulation_i2c_start()
{
	i2c_sda_1();
	i2c_scl_1();
	i2c_delay();
	i2c_sda_0();
	i2c_delay();
	i2c_scl_0();
	i2c_delay();
}


static void simulation_i2c_stop()
{
	i2c_sda_0();
	i2c_scl_1();
	i2c_delay();
	i2c_sda_1();
	i2c_delay();
}


void simulation_i2c_init()
{
	GPIO_InitType GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(I2C_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pins  = I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OutType = GPIO_OutType_OD;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;
    GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz;
    GPIO_Init(I2C_GPIO_SCL_PROT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pins  = I2C_SDA_PIN;
    GPIO_Init(I2C_GPIO_SDA_PROT, &GPIO_InitStructure);

	
	simulation_i2c_stop();
	//i2c_sck_0();
	//i2c_sda_0();
}


static void i2c_ack()
{
	i2c_sda_0();
	i2c_delay();
	i2c_scl_1();
	i2c_delay();
	i2c_scl_0();
	i2c_delay();
	i2c_sda_1();
}

static void i2c_nack()
{
	i2c_sda_1();
	i2c_delay();
	i2c_scl_1();
	i2c_delay();
	i2c_scl_0();
	i2c_delay();
}

static uint8_t read_ack()
{
	uint8_t ack;
	
	i2c_sda_1();
	i2c_delay();
	i2c_scl_1();
	i2c_delay();
	ack = i2c_sda_read();
	i2c_scl_0();
	i2c_delay();
	return ack;
}


static void simulation_i2c_sendbyte(uint8_t data)
{
	uint8_t i;
	for(i = 0;i < 8;i++)
	{
		if(data & 0x80)
		{
			i2c_sda_1();
		}
		else
		{
			i2c_sda_0();
		}
		i2c_delay();
		i2c_scl_1();
		i2c_delay();
		i2c_scl_0();
		i2c_delay();
		data <<= 1;
		if(i == 7)
		{
			i2c_sda_1();
		}
		
		
	}
	
}

static uint8_t simulation_i2c_readbyte(uint8_t ack)
{
	uint8_t i;
	uint8_t data = 0;
	for(i = 0;i < 8;i++)
	{
		data <<= 1;
		i2c_scl_1();
		i2c_delay();
		data |= i2c_sda_read();
		i2c_scl_0();
		i2c_delay();

	}
	if(ack == 0)
	{
		i2c_nack();
	}
	else
	{
		i2c_ack();
	}
	return data;
}


int simulation_i2c_writereg(uint8_t addr, uint8_t reg ,uint8_t data)
{
	simulation_i2c_start();
	simulation_i2c_sendbyte((addr << 1) | 0x00);
	read_ack();
	simulation_i2c_sendbyte(reg);
	read_ack();
	simulation_i2c_sendbyte(data);
	read_ack();
	simulation_i2c_stop();
	return 0;
}

int simulation_i2c_writeregs(uint8_t addr, uint8_t reg ,uint8_t len,uint8_t *data)
{
	uint8_t i;
	simulation_i2c_start();
	simulation_i2c_sendbyte((addr << 1) | 0x00);
	read_ack();
	simulation_i2c_sendbyte(reg);
	read_ack();
	for(i = 0;i < len;i++)
	{
		simulation_i2c_sendbyte(*data);
		data++;
		read_ack();
	}
	simulation_i2c_stop();
	return 0;
}


int simulation_i2c_readregs(uint8_t addr, uint8_t reg ,uint8_t len,uint8_t *data)
{
	uint8_t i;
	simulation_i2c_start();
	simulation_i2c_sendbyte((addr << 1) | 0x00);
	read_ack();
	simulation_i2c_sendbyte(reg);
	read_ack();
	
	simulation_i2c_start();
	simulation_i2c_sendbyte((addr << 1) | 0x01);
	read_ack();
	for(i = 0;i < (len - 1);i++)
	{
		*data = simulation_i2c_readbyte(1);
		data++;
	}
	*data = simulation_i2c_readbyte(0);
	simulation_i2c_stop();
	return 0;
}






