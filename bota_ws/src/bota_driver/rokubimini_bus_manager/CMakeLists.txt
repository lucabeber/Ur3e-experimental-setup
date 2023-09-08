cmake_minimum_required(VERSION 3.5.1)
project(rokubimini_bus_manager)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rokubimini REQUIRED)
find_package(bota_node REQUIRED)

include_directories(include)

set(dependencies "rclcpp" "rokubimini" "bota_node")

add_library(
        ${PROJECT_NAME}
        src/${PROJECT_NAME}/BusManager.cpp
)

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        )

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
