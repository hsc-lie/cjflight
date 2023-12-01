target_sources(${PROJECT_NAME}
    PRIVATE

    bmp280/bmp280.c
    bmp280/bmp280_cfg.c

    spl06/spl06.c
    spl06/spl06_cfg.c

    motor/motor.c
    motor/motor_cfg.c

    mpu6050/mpu6050.c
    mpu6050/mpu6050_cfg.c
)
