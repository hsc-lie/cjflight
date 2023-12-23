 #include "simulation_i2c.h"


static void SimulationI2CDelay(SimulationI2C_t * i2c) 
{
	uint32_t i = i2c->DelayCount;
	while(--i);
}



static uint8_t SimulationI2CStart(SimulationI2C_t * i2c)
{


	i2c->SDASet(1);
	SimulationI2CDelay(i2c);
	i2c->SCLSet(1);
	SimulationI2CDelay(i2c);
	i2c->SDASet(0);
	SimulationI2CDelay(i2c);
	i2c->SCLSet(0);
	SimulationI2CDelay(i2c);

}


static void SimulationI2CStop(SimulationI2C_t * i2c)
{

	i2c->SDASet(0);
	SimulationI2CDelay(i2c);
	i2c->SCLSet(1);
	SimulationI2CDelay(i2c);
	i2c->SDASet(1);
	SimulationI2CDelay(i2c);

}


static void SimulationI2CAck(SimulationI2C_t * i2c)
{

	i2c->SDASet(0);
	SimulationI2CDelay(i2c);
	i2c->SCLSet(1);
	SimulationI2CDelay(i2c);
	i2c->SCLSet(0);
	SimulationI2CDelay(i2c);
	i2c->SDASet(1);


}

static void SimulationI2CNAck(SimulationI2C_t * i2c)
{
	i2c->SDASet(1);
	SimulationI2CDelay(i2c);
	i2c->SCLSet(1);
	SimulationI2CDelay(i2c);
	i2c->SCLSet(0);
	SimulationI2CDelay(i2c);
}

static uint8_t SimulationI2CReadAck(SimulationI2C_t * i2c)
{
	uint8_t ack;

	i2c->SDADirSet(SIMULATION_I2C_SDA_RX);	
	i2c->SCLSet(1);
	SimulationI2CDelay(i2c);
	ack = i2c->SDARead();
	i2c->SCLSet(0);
	i2c->SDADirSet(SIMULATION_I2C_SDA_TX);
	SimulationI2CDelay(i2c);
	
	
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
		SimulationI2CDelay(i2c);
		i2c->SCLSet(1);
		SimulationI2CDelay(i2c);
		i2c->SCLSet(0);
		SimulationI2CDelay(i2c);
		data <<= 1;
		if(i == 7)
		{
			i2c->SDASet(1);
		}
		
		
	}
	
}

static uint8_t SimulationI2CReadByte(SimulationI2C_t * i2c, uint8_t ack)
{
	uint8_t i;
	uint8_t data = 0;

	i2c->SDADirSet(SIMULATION_I2C_SDA_RX);
	
	for(i = 0;i < 8;++i)
	{
		data <<= 1;
		i2c->SCLSet(1);
		SimulationI2CDelay(i2c);
		data |= i2c->SDARead();
		i2c->SCLSet(0);
		SimulationI2CDelay(i2c);

	}

	i2c->SDADirSet(SIMULATION_I2C_SDA_TX);
	
	if(ack == 0)
	{
		SimulationI2CNAck(i2c);
	}
	else
	{
		SimulationI2CAck(i2c);
	}
	return data;
}



SIMULATION_I2C_ERROR_t SimulationI2CSendData(SimulationI2C_t * i2c, uint8_t addr, uint8_t * reg, uint32_t regLen, uint8_t *data, uint8_t dataLen)
{
	uint32_t i;
	uint8_t ack;

	if((NULL == i2c)
		|| (NULL == i2c->SCLSet)
		|| (NULL == i2c->SDASet)
	)
	{
		return SIMULATION_I2C_ERROR_NULL;
	}
	
	SimulationI2CStart(i2c);
	SimulationI2C_SendByte(i2c, (addr << 1) | 0x00);
	
	ack = SimulationI2CReadAck(i2c);
	if(1 == ack)
	{
		return SIMULATION_I2C_ERROR_NACK;
	}



	for(i = 0;i < regLen;++i)
	{
		SimulationI2C_SendByte(i2c, *reg);
		
		ack = SimulationI2CReadAck(i2c);
		if(1 == ack)
		{
			return SIMULATION_I2C_ERROR_NACK;
		}

		reg++;
	}
	
	
	for(i = 0;i < dataLen;++i)
	{
		SimulationI2C_SendByte(i2c, *data);
		
		ack = SimulationI2CReadAck(i2c);
		if(1 == ack)
		{
			return SIMULATION_I2C_ERROR_NACK;
		}

		data++;
	}


	SimulationI2CStop(i2c);
	
	return SIMULATION_I2C_ERROR_OK;
}


SIMULATION_I2C_ERROR_t SimulationI2CReadData(SimulationI2C_t * i2c, uint8_t addr, uint8_t * reg, uint8_t regLen, uint8_t *data, uint8_t dataLen)
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
		return SIMULATION_I2C_ERROR_NULL;
	}

	if(0 != regLen)
	{
		SimulationI2CStart(i2c);
		SimulationI2C_SendByte(i2c, (addr << 1) | 0x00);
		
		ack = SimulationI2CReadAck(i2c);
		if(1 == ack)
		{
			return SIMULATION_I2C_ERROR_NACK;
		}

		for(i = 0;i < regLen;++i)
		{
			SimulationI2C_SendByte(i2c, *reg);
			
			ack = SimulationI2CReadAck(i2c);
			if(1 == ack)
			{
				return SIMULATION_I2C_ERROR_NACK;
			}
			reg++;
		}
	
	}
	
	
	SimulationI2CStart(i2c);
	SimulationI2C_SendByte(i2c, (addr << 1) | 0x01);
	
	ack = SimulationI2CReadAck(i2c);
	if(1 == ack)
	{
		return SIMULATION_I2C_ERROR_NACK;
	}

		
	for(i = 0;i < (dataLen - 1);++i)
	{
		*data = SimulationI2CReadByte(i2c, 1);
		data++;
	}
	*data = SimulationI2CReadByte(i2c, 0);
	SimulationI2CStop(i2c);
	
	return SIMULATION_I2C_ERROR_OK;
}

