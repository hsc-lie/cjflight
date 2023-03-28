PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc


CFLAGS = -I app \
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

GLOBAL_DEFINE = -DAT32F421C8T7 -DUSE_STDPERIPH_DRIVER

ASM_SOURCES = ./startup/gcc/startup_at32f421c8t7.s

FREERTOS_SOURCES = ./freeRTOS/list.c \
				   ./freeRTOS/queue.c \
				   ./freeRTOS/tasks.c \
				   ./freeRTOS/MemMang/heap_3.c


C_SOURCES = $(shell find ./app ./common ./dal ./dev ./hal ./hardware ./lib ./protocol_stack -name "*.c") \
			$(FREERTOS_SOURCES)

BUILD_DIR = 

OBJS = $(foreach src, $(C_SOURCES), $(patsubst %.c, %.o, $(BUILD_DIR)$(notdir $(src))))


VPATH = $(foreach src, $(C_SOURCES), $(dir $(src)))

elf:$(OBJS)
	$(CC) $^ -o $@



#%.o:$(C_SOURCES)%.c
%.o:%.c
#mkdir build
#$(CC) -c $< -o $@ $(CFLAGS)
	$(CC) -c $< -o $@ $(CFLAGS) $(GLOBAL_DEFINE)

.PHONY: show clean

show:
#echo $(C_SOURCES)
#@echo $(OBJS)
	@echo $(VPATH)

clean:
	rm -f *.o 



