target_sources(${PROJECT_NAME}
    PRIVATE
    port/port.c
    MemMang/heap_4.c
    croutine.c
    event_groups.c
    list.c
    queue.c
    stream_buffer.c
    tasks.c
    timers.c
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    .
	include
	port
)
