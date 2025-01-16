cmake_minimum_required(VERSION 3.16)

project(station)

set(CMAKE_AUTOMOC ON)

find_package(PkgConfig REQUIRED)
find_package(Qt6 COMPONENTS
	Widgets
	Network
REQUIRED)

pkg_check_modules(UDEV REQUIRED libudev)

add_executable(${PROJECT_NAME}
	"src/main.cpp"
	"src/Window.cpp"
	"src/Network.cpp"
	"src/JoystickWidget.cpp"
	"src/Joystick.cpp"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"src"
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-O3
	-Wall
	-Wextra
	-Wpedantic
)

target_link_libraries(${PROJECT_NAME}
	udev
	Qt6::Widgets
	Qt6::Network
)
