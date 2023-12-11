target_sources(${PROJECT_NAME}
    PRIVATE
    rcc/bsp_rcc.c
    dma/bsp_dma.c
    gpio/bsp_gpio.c
    usart/bsp_usart.c
    timer/bsp_timer.c
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    dma
	gpio
	rcc
	timer
	usart
)