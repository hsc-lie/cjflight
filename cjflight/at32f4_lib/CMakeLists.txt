aux_source_directory("./src" AT32F4_LIB_SOURCE_FILES)

target_sources(${PROJECT_NAME}
    PRIVATE
    ${AT32F4_LIB_SOURCE_FILES}
)

#message(${AT32F4_LIB_SOURCE_FILES})
