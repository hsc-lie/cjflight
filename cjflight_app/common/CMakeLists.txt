target_sources(${PROJECT_NAME}
    PRIVATE
    common.c
    ring_queue/ring_queue.c
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    .
	ring_queue
)