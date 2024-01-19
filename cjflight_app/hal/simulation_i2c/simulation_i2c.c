 #include "simulation_i2c.h"

/*
 * @函数名  SimulationI2CDelay
 * @用  途  模拟I2C延时
 * @参  数  i2c:模拟I2C
 * @返回值
*/
static inline void SimulationI2CDelay(SimulationI2C_t *i2c) 
{
	register uint32_t i = i2c->DelayCount;
	while(--i);
}

/*
 * @函数名  SimulationI2CStart
 * @用  途  模拟I2C发送起始信号
 * @参  数  i2c:模拟I2C
 * @返回值
*/
static void SimulationI2CStart(SimulationI2C_t *i2c)
{
	i2c->SDASet1();
	SimulationI2CDelay(i2c);
	i2c->SCLSet1();
	SimulationI2CDelay(i2c);
	i2c->SDASet0();
	SimulationI2CDelay(i2c);
	i2c->SCLSet0();
	SimulationI2CDelay(i2c);
}

/*
 * @函数名  SimulationI2CStop
 * @用  途  模拟I2C发送停止信号
 * @参  数  i2c:模拟I2C
 * @返回值
*/
static void SimulationI2CStop(SimulationI2C_t *i2c)
{
	i2c->SDASet0();
	SimulationI2CDelay(i2c);
	i2c->SCLSet1();
	SimulationI2CDelay(i2c);
	i2c->SDASet1();
	SimulationI2CDelay(i2c);
}

/*
 * @函数名  SimulationI2CAck
 * @用  途  模拟I2C发送应答
 * @参  数  i2c:模拟I2C
 * @返回值
*/
static void SimulationI2CAck(SimulationI2C_t *i2c)
{
	i2c->SDASet0();
	SimulationI2CDelay(i2c);
	i2c->SCLSet1();
	SimulationI2CDelay(i2c);
	i2c->SCLSet0();
	SimulationI2CDelay(i2c);
	i2c->SDASet1();
}

/*
 * @函数名  SimulationI2CNAck
 * @用  途  模拟I2C发送非应答
 * @参  数  i2c:模拟I2C
 * @返回值
*/
static void SimulationI2CNAck(SimulationI2C_t *i2c)
{
	i2c->SDASet1();
	SimulationI2CDelay(i2c);
	i2c->SCLSet1();
	SimulationI2CDelay(i2c);
	i2c->SCLSet0();
	SimulationI2CDelay(i2c);
}

/*
 * @函数名  SimulationI2CReceiveAck
 * @用  途  模拟I2C接收从机应答
 * @参  数  i2c:模拟I2C
 * @返回值  读取到的从机应答值
 *          0:应答
 *          1:非应答
*/
static uint8_t SimulationI2CReceiveAck(SimulationI2C_t *i2c)
{
	uint8_t ack;

	i2c->SDASetRX();	
	i2c->SCLSet1();
	SimulationI2CDelay(i2c);
	ack = i2c->SDARead();
	i2c->SCLSet0();
	i2c->SDASetTX();
	SimulationI2CDelay(i2c);
	
	return ack;
}

/*
 * @函数名  SimulationI2CSendByte
 * @用  途  模拟I2C发送一个字节数据
 * @参  数  i2c:模拟I2C
 *          data:发送的数据
 * @返回值
*/
static void SimulationI2CSendByte(SimulationI2C_t *i2c, uint8_t data)
{
	uint8_t i;

	for(i = 0;i < 8;++i)
	{
		if(data & 0x80)
		{
			i2c->SDASet1();
		}
		else
		{
			i2c->SDASet0();
		}

		SimulationI2CDelay(i2c);
		i2c->SCLSet1();
		SimulationI2CDelay(i2c);
		i2c->SCLSet0();
		SimulationI2CDelay(i2c);
		data <<= 1;
	}

	i2c->SDASet1();
}

/*
 * @函数名  SimulationI2CReceiveByte
 * @用  途  模拟I2C接收一个字节数据
 * @参  数  i2c:模拟I2C
 *          ack:主机应答值 0:应答 1:非应答
 * @返回值  接收到的数据
*/
static uint8_t SimulationI2CReceiveByte(SimulationI2C_t *i2c, uint8_t ack)
{
	uint8_t i;
	uint8_t data = 0;

	i2c->SDASetRX();
	
	for(i = 0;i < 8;++i)
	{
		data <<= 1;
		i2c->SCLSet1();
		SimulationI2CDelay(i2c);
		data |= i2c->SDARead();
		i2c->SCLSet0();
		SimulationI2CDelay(i2c);
	}

	i2c->SDASetTX();
	
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

/*
 * @函数名  SimulationI2CSendData
 * @用  途  模拟I2C写入数据
 * @参  数  i2c:模拟I2C
 *          addr:从机地址
 *          reg:寄存器值(或是命令)
 *          regLen:寄存器值长度(或是命令长度)
 *          data:要写入的数据
 *          dataLen:写入的数据的长度
 * @返回值  错误状态值
*/
SIMULATION_I2C_ERROR_t SimulationI2CWriteData(SimulationI2C_t *i2c, uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t *data, uint8_t dataLen)
{
	uint8_t *p;
	uint8_t *pEnd;
	uint8_t ack;

	if((NULL == i2c->SCLSet0)
	   || (NULL == i2c->SCLSet1)
	   || (NULL == i2c->SDASet0)
	   || (NULL == i2c->SDASet1)
	   || (NULL == i2c->SDASetTX)
	   || (NULL == i2c->SDASetRX)
	   || (NULL == i2c->SDARead)
	)
	{
		return SIMULATION_I2C_ERROR_NULL;
	}
	
	SimulationI2CStart(i2c);
	SimulationI2CSendByte(i2c, (addr << 1) | 0x00);
	
	ack = SimulationI2CReceiveAck(i2c);
	if(1 == ack)
	{
		return SIMULATION_I2C_ERROR_NACK;
	}

	pEnd = reg + regLen;
	for(p = reg;p < pEnd;++p)
	{
		SimulationI2CSendByte(i2c, *p);
		ack = SimulationI2CReceiveAck(i2c);
		if(1 == ack)
		{
			return SIMULATION_I2C_ERROR_NACK;
		}
	}
	
	pEnd = data + dataLen;
	for(p = data;p < pEnd;++p)
	{
		SimulationI2CSendByte(i2c, *p);
		ack = SimulationI2CReceiveAck(i2c);
		if(1 == ack)
		{
			return SIMULATION_I2C_ERROR_NACK;
		}
	}

	SimulationI2CStop(i2c);
	
	return SIMULATION_I2C_ERROR_OK;
}

/*
 * @函数名  SimulationI2CReadData
 * @用  途  模拟I2C读取数据
 * @参  数  i2c:模拟I2C
 *          addr:从机地址
 *          reg:寄存器值(或是命令)
 *          regLen:寄存器值长度(或是命令长度)
 *          data:读取到的数据
 *          dataLen:要读取数据的长度
 * @返回值  错误状态值
*/
SIMULATION_I2C_ERROR_t SimulationI2CReadData(SimulationI2C_t *i2c, uint8_t addr, uint8_t *reg, uint8_t regLen, uint8_t *data, uint8_t dataLen)
{
	uint8_t *p;
	uint8_t *pEnd;
	uint8_t ack;

	if((NULL == i2c->SCLSet0)
	   || (NULL == i2c->SCLSet1)
	   || (NULL == i2c->SDASet0)
	   || (NULL == i2c->SDASet1)
	   || (NULL == i2c->SDASetTX)
	   || (NULL == i2c->SDASetRX)
	   || (NULL == i2c->SDARead)
	)
	{
		return SIMULATION_I2C_ERROR_NULL;
	}

	if(0 != regLen)
	{
		SimulationI2CStart(i2c);
		SimulationI2CSendByte(i2c, (addr << 1) | 0x00);
		
		ack = SimulationI2CReceiveAck(i2c);
		if(1 == ack)
		{
			return SIMULATION_I2C_ERROR_NACK;
		}

		pEnd = reg + regLen;
		for(p = reg;p < pEnd;++p)
		{
			SimulationI2CSendByte(i2c, *p);	
			ack = SimulationI2CReceiveAck(i2c);
			if(1 == ack)
			{
				return SIMULATION_I2C_ERROR_NACK;
			}
		}
	}
	
	SimulationI2CStart(i2c);
	SimulationI2CSendByte(i2c, (addr << 1) | 0x01);
	
	ack = SimulationI2CReceiveAck(i2c);
	if(1 == ack)
	{
		return SIMULATION_I2C_ERROR_NACK;
	}

	pEnd = data + dataLen - 1;
	for(p = data;p < pEnd;++p)
	{
		*p = SimulationI2CReceiveByte(i2c, 1);
	}
	*p = SimulationI2CReceiveByte(i2c, 0);
	SimulationI2CStop(i2c);
	
	return SIMULATION_I2C_ERROR_OK;
}
