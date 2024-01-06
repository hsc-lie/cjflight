TARGET = cjflight

#编译文件路径
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj
OUT_DIR = $(BUILD_DIR)/out

#设置交叉编译
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
SZ = arm-none-eabi-size
#生成hex文件的指令
HEX = arm-none-eabi-objcopy -O ihex
#生成bin文件的指令
BIN = arm-none-eabi-objcopy -O binary -S

#删除指令
RM = rm -rf

#优化等级
OPT = -O0 -g -gdwarf-2

#CPU类型
CPU = -mthumb -mcpu=cortex-m4 -march=armv7e-m 
#FPU类型
#-mfpu=fpv4-sp-d16
FPU = 
#浮点计算的方式 
#hard 硬件
#soft 软件 
FLOAT-ABI = -mfloat-abi=soft
#MCU相关参数
MCU = $(CPU) $(FPU) $(FLOAT-ABI)

#C全局宏
C_DEFINE = -DAT32F421C8T7 -DUSE_STDPERIPH_DRIVER

#头文件路径
INCLUDE_DIR =\
	-I app \
	-I app/control \
	-I app/filter \
	-I app/pid \
	-I app/quaternion \
	-I app/remote \
	-I common \
	-I common/ring_queue \
	-I protocol_stack/ibus \
	-I dal/led \
	-I dal/remote \
	-I dev/bmp280 \
	-I dev/motor \
	-I dev/mpu6050 \
	-I dev/spl06 \
	-I freeRTOS \
	-I freeRTOS/include \
	-I freeRTOS/port \
	-I hal/gpio \
	-I hal/usart \
	-I hal/timer \
	-I hal/i2c \
	-I hal/simulation_i2c \
	-I bsp/at32f4 \
	-I bsp/at32f4/board/rcc \
	-I bsp/at32f4/board/interrupt \
	-I bsp/at32f4/board/dma \
	-I bsp/at32f4/board/gpio \
	-I bsp/at32f4/board/usart \
	-I bsp/at32f4/board/timer \
	-I bsp/at32f4/cmsis \
	-I bsp/at32f4/at32f4_lib/inc \

#需要编译的汇编
ASM_SOURCES = ./bsp/at32f4/startup/startup_at32f421c8t7.s

#freertos的c文件
FREERTOS_SOURCES =\
	./freeRTOS/port/port.c \
	./freeRTOS/MemMang/heap_4.c \
	./freeRTOS/list.c \
	./freeRTOS/queue.c \
	./freeRTOS/tasks.c
	

#c文件
C_SOURCES =\
	$(shell find ./app ./common ./protocol_stack ./dal ./dev ./hal ./bsp -name *.c) \
	$(FREERTOS_SOURCES)


ASM_FLAGS = $(MCU) $(OPT) -Wall -fdata-sections -ffunction-sections -MMD -MP -fdiagnostics-color=always
C_FLAGS = $(MCU) $(INCLUDE_DIR) $(C_DEFINE) $(OPT) -Wall -fdata-sections -ffunction-sections -fdiagnostics-color=always

#链接脚本
LINKER_FILE = ./bsp/at32f4/linker/AT32F421x8_FLASH.ld

#gcc库相关
#链接时需要放在其他.o文件后面
#因为链接时.o文件里对库的依赖是往后搜索
#-lc -lm -lnosys #-specs=nosys.specs -specs=nano.specs -specs=rdimon.specs -specs=rdpmon.specs
LIB_FLAGS = -specs=nano.specs -lc -lm -lnosys
#额外的库
LIBS =

#链接参数
LD_FLAGS = $(MCU) -T$(LINKER_FILE) -Wl,-Map=$(OUT_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -fdiagnostics-color=always

#汇编编译的.s文件
OBJS += $(addprefix $(OBJ_DIR)/, $(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(dir $(ASM_SOURCES))
#C编译的.o文件
OBJS += $(addprefix $(OBJ_DIR)/, $(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(dir $(C_SOURCES))


all:$(OUT_DIR)/$(TARGET).elf $(OUT_DIR)/$(TARGET).hex $(OUT_DIR)/$(TARGET).bin

$(OUT_DIR)/%.hex: $(OUT_DIR)/%.elf | $(OUT_DIR) 
	$(HEX) $< $@

$(OUT_DIR)/%.bin: $(OUT_DIR)/%.elf | $(OUT_DIR) 
	$(BIN) $< $@

$(OUT_DIR)/$(TARGET).elf: $(OBJS) Makefile | $(OUT_DIR) 
	$(CC) $(LD_FLAGS) $(OBJS) $(LIBS) $(LIB_FLAGS) -o $@
	$(SZ) $@

$(OBJ_DIR)/%.o:%.c Makefile | $(OBJ_DIR)
#$(CC) -c $(C_FLAGS) -Wa,-a,-ad,-alms=$(OBJ_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
	$(CC) -c $(C_FLAGS) $< -o $@

$(OBJ_DIR)/%.o:%.s Makefile | $(OBJ_DIR)
	$(AS) -c $(ASM_FLAGS) $< -o $@
	
$(OBJ_DIR):$(BUILD_DIR)
	mkdir -p $(OBJ_DIR)

$(OUT_DIR):$(BUILD_DIR)
	mkdir -p $(OUT_DIR)

$(BUILD_DIR):
	mkdir -p $@

-include $(addprefix $(OBJ_DIR)/, $(notdir $(C_SOURCES:.c=.d))) 

.PHONY: printf clean

printf:
#echo $(C_SOURCES)
#@echo OBJS = $(OBJS)
#@echo VPATH = $(VPATH)
#@echo C_FLAGS = $(C_FLAGS)
	@echo $(addprefix $(OBJ_DIR)/, $(notdir $(C_SOURCES:.o=.d))) 


clean:
	$(RM) $(OBJ_DIR)
	$(RM) $(OUT_DIR)
