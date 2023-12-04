target_sources(${PROJECT_NAME}
    PRIVATE
    usart/usart_hal.c
    usart/usart_hal_cfg.c
    simulation_i2c/simulation_i2c.c
    simulation_i2c/simulation_i2c_cfg.c
    i2c/i2c_hal.c
    i2c/i2c_hal_cfg.c
    pwm/pwm_hal.c
    pwm/pwm_hal_cfg.c
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    i2c
	pwm
	simulation_i2c
	usart
)

