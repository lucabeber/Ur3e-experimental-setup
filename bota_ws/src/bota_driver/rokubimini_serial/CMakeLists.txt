cmake_minimum_required(VERSION 3.5.1)
project(rokubimini_serial)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()
#if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic)
#endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rokubimini REQUIRED)
find_package(rokubimini_bus_manager REQUIRED)
find_package(rokubimini_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen3 REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
)

set(dependencies rokubimini rokubimini_bus_manager rokubimini_msgs)

add_library(
        ${PROJECT_NAME}
        src/${PROJECT_NAME}/RokubiminiSerial.cpp
        src/${PROJECT_NAME}/RokubiminiSerialImpl.cpp
        src/${PROJECT_NAME}/RokubiminiSerialBusManager.cpp
        src/${PROJECT_NAME}/RokubiminiSerialCommunication.cpp
)

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
        )

target_link_libraries(${PROJECT_NAME}
        ${EIGEN3_LIBRARIES}
        ${Boost_LIBRARIES}
        )

add_executable(${PROJECT_NAME}_bus_manager_node src/${PROJECT_NAME}/${PROJECT_NAME}_bus_manager_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_bus_manager_node
        ${dependencies}
        )

target_link_libraries(${PROJECT_NAME}_bus_manager_node
        ${PROJECT_NAME}
        )

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_bus_manager_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        )

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
