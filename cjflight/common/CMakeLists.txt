target_sources(${PROJECT_NAME}
    PRIVATE
    common.c
    circular_queue/circular_queue.c
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    .
	circular_queue
)