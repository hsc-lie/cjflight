cmake_minimum_required(VERSION 3.28.0)

set(CMAKE_SYSTEM_NAME Generic)

project(cjflight ASM C)

add_executable(${PROJECT_NAME})

#是否开启更详细的编译过程信息显示
SET(CMAKE_VERBOSE_MAKEFILE ON)

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



#C标准
set(C_STD -std=c99)

#全局宏定义
target_compile_definitions(${PROJECT_NAME}
	PRIVATE
)


target_sources(${PROJECT_NAME}
    PRIVATE
    ${STARTUP_FILE}
)


#应用层
add_subdirectory(app)
#通信协议栈
add_subdirectory(protocol_stack)
#通用库
add_subdirectory(common)
#freeRTOS
add_subdirectory(freeRTOS)
#设备抽象层
add_subdirectory(dal)
#设备层
add_subdirectory(dev)
#硬件抽象层
add_subdirectory(hal)
#BSP
add_subdirectory(bsp)


#汇编编译参数
set(CMAKE_ASM_FLAGS "-x assembler-with-cpp")
#编译参数
target_compile_options(${PROJECT_NAME}
	PRIVATE
	${C_STD} -Wall -O0 -g -gdwarf-2 -fdata-sections -ffunction-sections
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
	-specs=nano.specs -Wl,-Map=${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.map,--cref -Wl,--gc-sections
)


add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${SIZE} ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf
    COMMAND ${OBJCOPY} -Obinary ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.bin
    COMMAND ${OBJCOPY} -Oihex ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.hex
)

