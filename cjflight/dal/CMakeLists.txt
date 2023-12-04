target_sources(${PROJECT_NAME}
    PRIVATE
    led/led.c
    led/led_cfg.c
    remote/remote_data.c
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    led
	remote
)