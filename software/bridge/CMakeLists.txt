cmake_minimum_required(VERSION 3.16)

project(bridge)

add_executable(${PROJECT_NAME}
	"main.c"
	"../common/cmp/cmp.c"
)

target_include_directories(${PROJECT_NAME} PRIVATE
    "../common"
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
	cjson
)
