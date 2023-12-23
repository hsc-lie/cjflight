#CMSIS
add_subdirectory(cmsis)
#AT32外设库
add_subdirectory(at32f4_lib)
#板级相关
add_subdirectory(board)

#内核
set(CPU -mthumb -mcpu=cortex-m4 -march=armv7e-m)

#FPU类型
#-mfpu=fpv4-sp-d16
set(FPU "")

#浮点计算的方式
#hard 硬件
#soft 软件
set(FLOAT_ABI -mfloat-abi=soft)

#启动文件
set(STARTUP_FILE startup/startup_at32f421c8t7.s)
#链接文件
set(LINKER_FILE AT32F421x8_FLASH.ld)
#CPU相关的全局宏
set(CPU_DEFINE AT32F421C8T7)


#全局宏定义
target_compile_definitions(${PROJECT_NAME}
	PUBLIC
	${CPU_DEFINE}
)

#编译参数
target_compile_options(${PROJECT_NAME}
    PUBLIC
    ${CPU} ${FPU} ${FLOAT_ABI} 
)

#头文件路径
target_include_directories(${PROJECT_NAME}
    PUBLIC
    .
)

#源文件
target_sources(${PROJECT_NAME}
    PUBLIC
    ${STARTUP_FILE}
    at32f4xx_it.c
)

#链接文件路径
target_link_directories(${PROJECT_NAME}
    PUBLIC
    linker
)

#链接参数
target_link_options(${PROJECT_NAME}
    PUBLIC
    ${CPU} ${FPU} ${FLOAT_ABI}
	-T ${LINKER_FILE}
)




