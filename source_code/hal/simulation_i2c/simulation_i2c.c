 #include "simulation_i2c.h"


static void SimulationI2C_Delay(SimulationI2C_t * i2c) 
{
	uint32_t i = i2c->DelayCount;
	while(--i);
}



static uint8_t SimulationI2C_Start(SimulationI2C_t * i2c)
{


	i2c->SDASet(1);
	SimulationI2C_Delay(i2c);
	i2c->SCLSet(1);
	SimulationI2C_Delay(i2c);
	i2c->SDASet(0);
	SimulationI2C_Delay(i2c);
	i2c->SCLSet(0);
	SimulationI2C_Delay(i2c);

}


static void SimulationI2C_Stop(SimulationI2C_t * i2c)
{

	i2c->SDASet(0);
	SimulationI2C_Delay(i2c);
	i2c->SCLSet(1);
	SimulationI2C_Delay(i2c);
	i2c->SDASet(1);
	SimulationI2C_Delay(i2c);

}

/*
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
	//i2c->SDASet(0);
}
*/

static void SimulationI2C_Ack(SimulationI2C_t * i2c)
{

	i2c->SDASet(0);
	SimulationI2C_Delay(i2c);
	i2c->SCLSet(1);
	SimulationI2C_Delay(i2c);
	i2c->SCLSet(0);
	SimulationI2C_Delay(i2c);
	i2c->SDASet(1);


}

static void SimulationI2C_NAck(SimulationI2C_t * i2c)
{
	i2c->SDASet(1);
	SimulationI2C_Delay(i2c);
	i2c->SCLSet(1);
	SimulationI2C_Delay(i2c);
	i2c->SCLSet(0);
	SimulationI2C_Delay(i2c);
}

static uint8_t SimulationI2C_ReadAck(SimulationI2C_t * i2c)
{
	uint8_t ack;

	i2c->SDADirSet(E_SIMULATION_I2C_SDA_RX);	
	i2c->SCLSet(1);
	SimulationI2C_Delay(i2c);
	ack = i2c->SDARead();
	i2c->SCLSet(0);
	i2c->SDADirSet(E_SIMULATION_I2C_SDA_TX);
	SimulationI2C_Delay(i2c);
	
	
	return ack;
}


static void SimulationI2C_SendByte(SimulationI2C_t * i2c, uint8_t data)
{
	uint8_t i;
	for(i = 0;i < 8;++i)
	{
		if(data & 0x80)
		{
			i2c->SDASet(1);
		}
		else
		{
			i2c->SDASet(0);
		}
		SimulationI2C_Delay(i2c);
		i2c->SCLSet(1);
		SimulationI2C_Delay(i2c);
		i2c->SCLSet(0);
		SimulationI2C_Delay(i2c);
		data <<= 1;
		if(i == 7)
		{
			i2c->SDASet(1);
		}
		
		
	}
	
}

static uint8_t SimulationI2C_ReadByte(SimulationI2C_t * i2c, uint8_t ack)
{
	uint8_t i;
	uint8_t data = 0;

	i2c->SDADirSet(E_SIMULATION_I2C_SDA_RX);
	
	for(i = 0;i < 8;++i)
	{
		data <<= 1;
		i2c->SCLSet(1);
		SimulationI2C_Delay(i2c);
		data |= i2c->SDARead();
		i2c->SCLSet(0);
		SimulationI2C_Delay(i2c);

	}

	i2c->SDADirSet(E_SIMULATION_I2C_SDA_TX);
	
	if(ack == 0)
	{
		SimulationI2C_NAck();
	}
	else
	{
		SimulationI2C_Ack();
	}
	return data;
}



E_SIMULATION_I2C_ERROR SimulationI2C_SendData(SimulationI2C_t * i2c, uint8_t addr, uint8_t * reg, uint32_t regLen, uint8_t *data, uint8_t dataLen)
{
	uint32_t i;
	uint8_t ack;

	if((NULL == i2c)
		|| (NULL == i2c->SCLSet)
		|| (NULL == i2c->SDASet)
	)
	{
		return E_SIMULATION_I2C_ERROR_NULL;
	}
	
	SimulationI2C_Start(i2c);
	SimulationI2C_SendByte((addr << 1) | 0x00);
	
	ack = SimulationI2C_ReadAck(i2c);
	if(1 == ack)
	{
		return E_SIMULATION_I2C_ERROR_NACK;
	}



	for(i = 0;i < regLen;++i)
	{
		SimulationI2C_SendByte(i2c, *reg);
		
		ack = SimulationI2C_ReadAck(i2c);
		if(1 == ack)
		{
			return E_SIMULATION_I2C_ERROR_NACK;
		}

		reg++;
	}
	
	
	for(i = 0;i < dataLen;++i)
	{
		SimulationI2C_SendByte(i2c, *data);
		
		ack = SimulationI2C_ReadAck(i2c);
		if(1 == ack)
		{
			return E_SIMULATION_I2C_ERROR_NACK;
		}

		data++;
	}


	SimulationI2C_Stop(i2c);
	
	return E_SIMULATION_I2C_ERROR_OK;
}


E_SIMULATION_I2C_ERROR SimulationI2C_ReadData(SimulationI2C_t * i2c, uint8_t addr, uint8_t * reg, uint8_t regLen, uint8_t *data, uint8_t dataLen)
{
	uint32_t i;
	uint8_t ack;

	if((NULL == i2c)
		|| (NULL == i2c->SCLSet)
		|| (NULL == i2c->SDASet)
		|| (NULL == i2c->SDADirSet)
		|| (NULL == i2c->SDARead)
	)
	{
		return E_SIMULATION_I2C_ERROR_NULL;
	}

	if(0 != regLen)
	{
		SimulationI2C_Start(i2c);
		SimulationI2C_SendByte((addr << 1) | 0x00);

		for(i = 0;i < regLen;++i)
		{
			SimulationI2C_SendByte(i2c, *reg);
			
			ack = SimulationI2C_ReadAck(i2c);
			if(1 == ack)
			{
				return E_SIMULATION_I2C_ERROR_NACK;
			}
			reg++;
		}
	
	}
	
	
	SimulationI2C_Start(i2c);
	SimulationI2C_SendByte((addr << 1) | 0x01);
	
	ack = SimulationI2C_ReadAck(i2c);
	if(1 == ack)
	{
		return E_SIMULATION_I2C_ERROR_NACK;
	}

		
	for(i = 0;i < (len - 1);++i)
	{
		*data = SimulationI2C_ReadByte(i2c, 1);
		data++;
	}
	*data = SimulationI2C_ReadByte(i2c, 0);
	SimulationI2C_Stop(i2c);
	
	return E_SIMULATION_I2C_ERROR_OK;
}






