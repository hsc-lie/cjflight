cmake_minimum_required(VERSION 3.28.0)

set(CMAKE_SYSTEM_NAME Generic)

project(cjflight ASM C)

add_executable(${PROJECT_NAME})

#是否开启更详细的编译过程信息显示
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# 彩色日志输出；
set(CMAKE_COLOR_DIAGNOSTICS TRUE)

#设置生成目标的文件类型
set(CMAKE_EXECUTABLE_SUFFIX .elf)

#设置生成目标的目录
set(EXECUTABLE_OUTPUT_PATH out)

#设置交叉编译器
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILE arm-none-eabi-g++)
set(OBJCOPY arm-none-eabi-objcopy)
set(SIZE arm-none-eabi-size)

#内核
set(CPU -mcpu=cortex-m4 -mthumb)

#FPU类型
set(FPU "")

#浮点计算的方式
#hard 硬件
#soft 软件
set(FLOAT_ABI -mfloat-abi=soft)

#C标准
set(C_STD -std=c99)

#启动文件
set(STARTUP_FILE ./startup/startup_at32f421c8t7.s)
#链接文件
set(LINKER_FILE ../linker/AT32F421x8_FLASH.ld)
#CPU相关的全局宏
set(CPU_DEFINE AT32F421C8T7)

#全局宏定义
target_compile_definitions(${PROJECT_NAME}
	PRIVATE
	${CPU_DEFINE}
	USE_STDPERIPH_DRIVER
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
set(CMAKE_ASM_FLAGS "-x assembler-with-cpp")
#编译参数
target_compile_options(${PROJECT_NAME}
	PRIVATE
	${CPU} ${FPU} ${FLOAT_ABI} ${C_STD} -Wall -Og -g -gdwarf-2 -fdata-sections -ffunction-sections
)


#垃圾玩意
#链接时-lc -lm -lnosys不懂怎么调到.o文件后面
#只能先这样了
#-lc -lm -lnosys
target_link_libraries(${PROJECT_NAME} c m nosys)

#链接参数
# -specs=nosys.specs -specs=nano.specs -specs=rdimon.specs -specs=rdpmon.specs
target_link_options(${PROJECT_NAME}
	PRIVATE
	${CPU} ${FPU} ${FLOAT_ABI}
	-T${LINKER_FILE} -specs=nano.specs -Wl,-Map=${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.map,--cref -Wl,--gc-sections
)


add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${SIZE} ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf
    COMMAND ${OBJCOPY} -Obinary ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.bin
    COMMAND ${OBJCOPY} -Oihex ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.hex
)

