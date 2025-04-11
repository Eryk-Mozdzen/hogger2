cmake_minimum_required(VERSION 3.16)

project(bridge)

find_package(ZeroMQ REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(${PROJECT_NAME}
	"main.cpp"
)

target_include_directories(${PROJECT_NAME} PRIVATE
    "../common"
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-O3
	-std=c++17
	-Wall
	-Wextra
	-Wpedantic
)

target_link_libraries(${PROJECT_NAME} PRIVATE
	zmq
	nlohmann_json::nlohmann_json
)
