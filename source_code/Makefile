TARGET = cai_ji_flight

DEBUG = 1

OPT = -Og

BUILD_DIR = build

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size


HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

CPU = -mcpu=cortex-m4 

FPU = 

#FLOAT-ABI = -mfloat-abi=hard
FLOAT-ABI = -mfloat-abi=soft
#FLOAT-ABI = -mfloat-abi=softfp
#FLOAT-ABI = 

MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

ASM_DEFINE = 

#C_DEFINE = -DAT32F421C8T7 -DUSE_STDPERIPH_DRIVER -DUSE_FULL_ASSERT
C_DEFINE = -DAT32F421C8T7 -DUSE_STDPERIPH_DRIVER

C_INCLUDE = -I app \
		 -I app/control \
		 -I app/filter \
		 -I app/pid \
		 -I app/quaternion \
		 -I app/remote_task \
		 -I common \
		 -I common/circular_queue \
		 -I dal/led \
		 -I dal/remote \
		 -I dev/bmp280 \
		 -I dev/motor \
		 -I dev/mpu6050 \
		 -I dev/spl06 \
		 -I freeRTOS \
		 -I freeRTOS/include \
		 -I freeRTOS/GCC \
		 -I hal/i2c \
		 -I hal/pwm \
		 -I hal/simulation_i2c \
		 -I hal/usart \
		 -I hardware/dma \
		 -I hardware/gpio \
		 -I hardware/rcc \
		 -I hardware/timer \
		 -I hardware/usart \
		 -I protocol_stack/ibus \
		 -I CMSIS \
		 -I lib/inc


ASM_SOURCES = ./startup/gcc/startup_at32f421c8t7.s

FREERTOS_SOURCES = ./freeRTOS/GCC/port.c \
				   ./freeRTOS/list.c \
				   ./freeRTOS/queue.c \
				   ./freeRTOS/tasks.c \
				   ./freeRTOS/MemMang/heap_3.c


C_SOURCES = $(shell find ./app ./common ./CMSIS ./dal ./dev ./hal ./hardware ./lib ./protocol_stack -name "*.c") \
			$(FREERTOS_SOURCES)


ASM_FLAGS = $(MCU) $(ASM_DEFINE) $(AS_INCLUDE) $(OPT) -Wall -fdata-sections -ffunction-sections
C_FLAGS = $(MCU) $(C_DEFINE) $(C_INCLUDE) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
	C_FLAGS += -g -gdwarf-2
endif

C_FLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
#C_FLAGS += -MMD -MP

#链接脚本
LDSCRIPT = ./linker/gcc/AT32F421x8_FLASH.ld

#C标准库相关
LIBS = -lc -lm -lnosys
#额外的库
LIBDIR =

#链接参数
LD_FLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections




#OBJS = $(foreach src, $(C_SOURCES), $(patsubst %.c, %.o, $(BUILD_DIR)$(notdir $(src))))
OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJS += $(addprefix $(BUILD_DIR)/, $(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

#VPATH = $(foreach src, $(C_SOURCES), $(dir $(src)))

all:$(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


$(BUILD_DIR):
	mkdir $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS) Makefile
	$(CC) $(OBJS) $(LD_FLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.o:%.c Makefile | $(BUILD_DIR)
	$(CC) -c $(C_FLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
#(CC) -c $(C_FLAGS) $< -o $@

$(BUILD_DIR)/%.o:%.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASM_FLAGS) $< -o $@


.PHONY: show clean debug

show:
#echo $(C_SOURCES)
#@echo OBJS = $(OBJS)
#@echo VPATH = $(VPATH)
	@echo C_FLAGS = $(C_FLAGS)

clean:
	rm -rf build

debug:
	JLinkGDBServer -if SWD -device -AT32F421C8T7

