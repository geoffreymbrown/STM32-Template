set(CMAKE_C_FLAGS "-O1 -g")

set(CMAKE_ASM_FLAGS "-g")

set(DEMO_FILES
        main.c
)
add_executable(demo.elf ${DEMO_FILES})
