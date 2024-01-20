aux_source_directory("src" HAL_SOURCE_FILES)

target_sources(${PROJECT_NAME}
    PRIVATE
    ${HAL_SOURCE_FILES}
    simulation_i2c/simulation_i2c.c
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    include
    simulation_i2c
)

