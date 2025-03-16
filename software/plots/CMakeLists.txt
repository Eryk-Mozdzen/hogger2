cmake_minimum_required(VERSION 3.16)

project(plots)

set(CMAKE_AUTOMOC ON)

find_package(Qt6 COMPONENTS
	Widgets
	Network
	PrintSupport
	OpenGL
REQUIRED)

add_executable(${PROJECT_NAME}
	"qcustomplot/qcustomplot.cpp"

	"src/main.cpp"
	"src/Window.cpp"
	"src/LiveChart.cpp"
	"src/Subscriber.cpp"
)

set_source_files_properties("qcustomplot/qcustomplot.cpp" PROPERTIES COMPILE_FLAGS
	"-w"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"qcustomplot"
	"src"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
	QCUSTOMPLOT_USE_OPENGL
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-O3
	-Wall
	-Wextra
	-Wpedantic
)

target_link_libraries(${PROJECT_NAME}
	zmq
	Qt6::Widgets
	Qt6::Network
	Qt6::PrintSupport
	Qt6::OpenGL
)
