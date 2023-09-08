cmake_minimum_required(VERSION 3.5.1)
project(rokubimini_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Reading.msg"
        "srv/GetSensorConfiguration.srv"
        "srv/SetSensorConfiguration.srv"
        "srv/FirmwareUpdateEthercat.srv"
        "srv/FirmwareUpdateSerial.srv"
        "srv/ResetWrench.srv"
        DEPENDENCIES builtin_interfaces std_msgs sensor_msgs geometry_msgs
        )

ament_export_dependencies(rosidl_default_generators geometry_msgs sensor_msgs std_msgs)
ament_package()
