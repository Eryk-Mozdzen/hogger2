cmake_minimum_required(VERSION 3.16)

project(bridge)

add_executable(${PROJECT_NAME}
	"main.c"
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-O3
	-std=c17
	-Wall
	-Wextra
	-Wpedantic
)

target_link_libraries(${PROJECT_NAME} PRIVATE
	zmq
)
