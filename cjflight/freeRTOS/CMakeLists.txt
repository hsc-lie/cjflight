target_sources(${PROJECT_NAME}
    PRIVATE
    port/port.c
    MemMang/heap_4.c
    list.c
    queue.c
    tasks.c
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    .
	include
	port
)
