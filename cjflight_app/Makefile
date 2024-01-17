TARGET=cjflight

#编译中间生成文件路径
OBJ_DIR=build/obj
#编译输出文件路径
OUT_DIR=build/out

#设置创建文件夹指命令
MKDIR=mkdir -p
#设置删除命令
RM=rm -rf

#设置交叉编译
CC=arm-none-eabi-gcc
AS=arm-none-eabi-gcc -x assembler-with-cpp
SZ=arm-none-eabi-size
#生成hex文件的指令
HEX=arm-none-eabi-objcopy -O ihex
#生成bin文件的指令
BIN=arm-none-eabi-objcopy -O binary -S

#0:Release
#1:Debug
DEBUG=1

#设置C标准
C_STD=-std=c99

#优化等级
ifeq ($(DEBUG), 0)
	OPT=-O3
else
	OPT=-O0 -g -gdwarf-2
endif

#CPU类型
CPU=-mthumb -mcpu=cortex-m4 -march=armv7e-m 
#FPU类型
#-mfpu=fpv4-sp-d16
FPU=
#浮点计算的方式 
#hard 硬件
#soft 软件 
FLOAT-ABI=-mfloat-abi=soft
#MCU相关参数
MCU=$(CPU) $(FPU) $(FLOAT-ABI)

#C全局宏
C_DEFINES=\
	AT32F421C8T7 \
	USE_STDPERIPH_DRIVER \

C_DEFINE_FLAGS=$(addprefix -D, $(C_DEFINES))

#编译参数
ASM_FLAGS=$(MCU) $(OPT) -Wall -MMD -MP -fdata-sections -ffunction-sections -fdiagnostics-color=always
C_FLAGS=$(MCU) $(C_STD) $(OPT) -Wall -MMD -MP -fdata-sections -ffunction-sections -fdiagnostics-color=always

#静态库
LIBS=

#gcc库相关
#链接时需要放在其他.o文件后面
#因为链接时.o文件里对库的依赖是往后搜索
#-lc -lm -lnosys #-specs=nosys.specs -specs=nano.specs -specs=rdimon.specs -specs=rdpmon.specs
LIB_FLAGS=-specs=nano.specs -lc -lm -lnosys $(addprefix -l, $(LIBS))

#链接脚本
LINKER_FILE = ./bsp/at32f4/linker/AT32F421x8_FLASH.ld

#链接参数
LINKER_FLAGS=$(MCU) -T$(LINKER_FILE) -Wl,-Map=$(OUT_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -fdiagnostics-color=always

#头文件路径
INCLUDE_DIRS =\
	app \
	app/control \
	app/filter \
	app/pid \
	app/quaternion \
	app/remote \
	app/sensors \
	common \
	common/ring_queue \
	protocol_stack/ibus \
	dev/led \
	dev/bmp280 \
	dev/motor \
	dev/mpu6050 \
	dev/spl06 \
	freeRTOS \
	freeRTOS/include \
	freeRTOS/port \
	hal/gpio \
	hal/usart \
	hal/timer \
	hal/i2c \
	hal/simulation_i2c \
	bsp/at32f4 \
	bsp/at32f4/board/rcc \
	bsp/at32f4/board/interrupt \
	bsp/at32f4/board/dma \
	bsp/at32f4/board/gpio \
	bsp/at32f4/board/usart \
	bsp/at32f4/board/timer \
	bsp/at32f4/cmsis \
	bsp/at32f4/at32f4_lib/inc \

INCLUDE_FLAGS=$(addprefix -I, $(INCLUDE_DIRS))

#需要编译的汇编
ASM_SOURCES=./bsp/at32f4/startup/startup_at32f421c8t7.s

#freertos的c文件
FREERTOS_SOURCES =\
	./freeRTOS/port/port.c \
	./freeRTOS/MemMang/heap_4.c \
	./freeRTOS/croutine.c \
	./freeRTOS/event_groups.c \
	./freeRTOS/list.c \
	./freeRTOS/queue.c \
	./freeRTOS/stream_buffer.c \
	./freeRTOS/tasks.c \
	./freeRTOS/timers.c \

#c文件
C_SOURCES=\
	$(shell find ./app ./common ./protocol_stack ./dev ./hal ./bsp -name *.c) \
	$(FREERTOS_SOURCES)

#汇编编译的.s文件
OBJS+=$(addprefix $(OBJ_DIR)/, $(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(dir $(ASM_SOURCES))
#C编译的.o文件
OBJS+=$(addprefix $(OBJ_DIR)/, $(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(dir $(C_SOURCES))


all:$(OUT_DIR)/$(TARGET).elf $(OUT_DIR)/$(TARGET).hex $(OUT_DIR)/$(TARGET).bin

$(OUT_DIR)/%.hex: $(OUT_DIR)/%.elf | $(OUT_DIR) 
	$(HEX) $< $@

$(OUT_DIR)/%.bin: $(OUT_DIR)/%.elf | $(OUT_DIR) 
	$(BIN) $< $@

$(OUT_DIR)/$(TARGET).elf: $(OBJS) Makefile | $(OUT_DIR) 
	$(CC) $(LINKER_FLAGS) $(OBJS) $(LIB_FLAGS) -o $@
	$(SZ) $@

$(OBJ_DIR)/%.o:%.c Makefile | $(OBJ_DIR)
	$(CC) -c $(C_DEFINE_FLAGS) $(INCLUDE_FLAGS) $(C_FLAGS) $< -o $@

$(OBJ_DIR)/%.o:%.s Makefile | $(OBJ_DIR)
	$(AS) -c $(C_DEFINE_FLAGS) $(INCLUDE_FLAGS) $(ASM_FLAGS) $< -o $@
	
$(OBJ_DIR):
	$(MKDIR) $(OBJ_DIR)

$(OUT_DIR):
	$(MKDIR) $(OUT_DIR)

-include $(addprefix $(OBJ_DIR)/, $(notdir $(C_SOURCES:.c=.d))) 

.PHONY:clean print

clean:
	$(RM) $(OBJ_DIR)
	$(RM) $(OUT_DIR)

print:
#echo $(C_SOURCES)
#@echo OBJS = $(OBJS)
#@echo VPATH = $(VPATH)
#@echo C_FLAGS = $(C_FLAGS)
#@echo $(addprefix $(OBJ_DIR)/, $(notdir $(C_SOURCES:.o=.d))) 
