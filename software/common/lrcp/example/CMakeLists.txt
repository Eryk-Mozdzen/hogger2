cmake_minimum_required(VERSION 3.16)

project(example)

add_executable(${PROJECT_NAME}
	"src/main.c"
	"src/iostream.c"
	"../stream.c"
	"../frame.c"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"include"
	"../.."
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-O0
	-g
	-Wall
	-Wextra
	-Wpedantic
)
