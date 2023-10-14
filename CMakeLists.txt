cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(my_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(my_program main.c)

pico_enable_stdio_usb(my_program 1)

pico_add_extra_outputs(my_program)

target_link_libraries(my_program	pico_stdlib
									pico_multicore)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")
