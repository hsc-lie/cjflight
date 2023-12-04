target_sources(${PROJECT_NAME}
    PRIVATE
    rcc/rcc_cfg.c
    dma/dma_cfg.c
    gpio/gpio_cfg.c
    usart/usart_cfg.c
    timer/timer_cfg.c
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    dma
	gpio
	rcc
	timer
	usart
)