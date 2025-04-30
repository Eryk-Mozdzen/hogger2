cmake_minimum_required(VERSION 3.16)

project(simulation)

find_package(drake REQUIRED)

add_executable(${PROJECT_NAME}
	"src/main.cpp"
	"src/generators/TrajectoryGenerator.cpp"
	"src/generators/Circle.cpp"
	"src/generators/Lemniscate.cpp"
	"src/controllers/JPTDDynamic.cpp"
	"src/controllers/JPTDDynamic1D.cpp"
	"src/controllers/Naive.cpp"
	"src/Model.cpp"

	"../common/control/generators.c"
	"../common/control/jptd_dynamic.c"
	"../common/control/jptd_dynamic_1d.c"
	"../common/control/naive.c"
)

target_include_directories(${PROJECT_NAME} PRIVATE
	"src"
	"../common"
)

target_compile_options(${PROJECT_NAME} PRIVATE
	-O3
	-Wall
	-Wextra
	-Wpedantic
)

target_link_libraries(${PROJECT_NAME}
	drake::drake
)
