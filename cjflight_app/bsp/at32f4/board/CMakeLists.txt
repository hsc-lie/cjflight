aux_source_directory("src" BOARD_SOURCE_FILES)

target_sources(${PROJECT_NAME}
    PRIVATE
    ${BOARD_SOURCE_FILES}
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    include
)