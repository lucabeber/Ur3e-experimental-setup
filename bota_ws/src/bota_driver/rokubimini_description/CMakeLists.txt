cmake_minimum_required(VERSION 3.5.1)
project(rokubimini_description)

if (NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)

set(dependencies sensor_msgsm std_msgs xacro)

install(DIRECTORY meshes/
		DESTINATION share/${PROJECT_NAME}/meshes
		)

install(DIRECTORY urdf/
		DESTINATION share/${PROJECT_NAME}/urdf
		)

ament_export_dependencies(${dependencies})
ament_package()