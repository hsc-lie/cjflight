cmake_minimum_required(VERSION 3.28.0)

set(CMAKE_SYSTEM_NAME Generic)

project(cjflight ASM C)

add_executable(${PROJECT_NAME})

#是否开启更详细的编译过程信息显示
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# 彩色日志输出；
set(CMAKE_COLOR_DIAGNOSTICS TRUE)

#设置生成目标的文件类型
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

#设置生成目标的目录
set(EXECUTABLE_OUTPUT_PATH "./out")

#设置交叉编译器
set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc.exe")
set(CMAKE_C_COMPILER "arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILE "arm-none-eabi-g++.exe")
SET(AS "arm-none-eabi-as.exe")
SET(AR "arm-none-eabi-ar.exe")
set(OBJCOPY "arm-none-eabi-objcopy.exe")
SET(OBJDUMP "arm-none-eabi-objdump.exe")
set(SIZE "arm-none-eabi-size.exe")

#C标准
set(CMAKE_C_STANDARD 99)

#C标准库相关
#-specs=nosys.specs -specs=nano.specs -specs=rdimon.specs -specs=rdpmon.specs
set(LIB_FLAG "-specs=nano.specs -specs=rdimon.specs")

#内核
set(CPU "-mcpu=cortex-m4 -mthumb")

#FPU类型
set(FPU "")

#浮点计算的方式
#hard 硬件
#soft 软件
set(FLOAT_ABI "-mfloat-abi=soft")


#启动文件
set(STARTUP_FILE "./startup/startup_at32f421c8t7.s")
#链接文件
set(LINK_FILE "../linker/AT32F421x8_FLASH.ld")
#CPU相关的全局宏
set(CPU_DEFINE "AT32F421C8T7")

#全局宏定义
ADD_DEFINITIONS(
    -D ${CPU_DEFINE}
    -D USE_STDPERIPH_DRIVER
)

#头文件路径
include_directories(
    .
    app
	app/control
	app/filter
	app/pid
	app/quaternion
	app/remote
    freeRTOS
	freeRTOS/include
	freeRTOS/port
	common
	common/circular_queue
	dal/led
	dal/remote
	dev/bmp280
	dev/motor
	dev/mpu6050
	dev/spl06
	hal/i2c
	hal/pwm
	hal/simulation_i2c
	hal/usart
	hardware/dma
	hardware/gpio
	hardware/rcc
	hardware/timer
	hardware/usart
	protocol_stack/ibus
	CMSIS
	at32f4_lib/inc
)


target_sources(${PROJECT_NAME}
    PRIVATE
    ${STARTUP_FILE}
)


#应用层
add_subdirectory(app)

#freeRTOS
add_subdirectory(freeRTOS)

#通信协议栈
add_subdirectory(protocol_stack)
#通用库
add_subdirectory(common)

#设备抽象层
add_subdirectory(dal)
#设备层
add_subdirectory(dev)
#硬件抽象层
add_subdirectory(hal)
#硬件层
add_subdirectory(hardware)

#CMSIS
add_subdirectory(CMSIS)
#AT32外设库
add_subdirectory(at32f4_lib)



#汇编编译参数
set(CMAKE_ASM_FLAGS "-Wall -O0 -g -gdwarf-2 -fdata-sections -ffunction-sections ${CPU} ${FPU} ${FLOAT_ABI} -x assembler-with-cpp")
#C编译参数
set(CMAKE_C_FLAGS   "-Wall -O0 -g -gdwarf-2 -fdata-sections -ffunction-sections ${CPU} ${FPU} ${FLOAT_ABI}")
#链接参数
set(CMAKE_EXE_LINKER_FLAGS "-T${LINK_FILE} ${LIB_FLAG} ${LIBS} -Wl,-Map=${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.map,--cref -Wl,--gc-sections")

add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${SIZE} ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf
    COMMAND ${OBJCOPY} -Obinary ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.bin
    COMMAND ${OBJCOPY} -Oihex ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.hex
)

