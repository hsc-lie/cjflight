#### 介绍
四旋翼飞控
主控MCU使用雅特力的AT32F421
RTOS使用freeRTOS

#### 软件架构
#app:应用层
dal：设备抽象层
dev：设备层
hal：硬件抽象层
hardware：硬件层

comomon：一些通用的模块
protocol_stack：通讯协议栈

freeRTOS：存放freeRTOS V9源码
Libraries：存放芯片片内外设驱动库

user：暂未整理的代码



