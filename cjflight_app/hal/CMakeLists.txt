target_sources(${PROJECT_NAME}
    PRIVATE
    dev/dev.c
    gpio/gpio_dev.c
    usart/usart_dev.c
    i2c/i2c_dev.c
    simulation_i2c/simulation_i2c.c
    timer/timer_dev.c
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    dev
    gpio
    usart
    i2c
    simulation_i2c
	timer
)

