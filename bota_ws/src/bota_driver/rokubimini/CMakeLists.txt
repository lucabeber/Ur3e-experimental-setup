cmake_minimum_required(VERSION 3.5.1)
project(rokubimini)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
)

set(dependencies "rclcpp" "sensor_msgs" "geometry_msgs" "diagnostic_updater")

add_library(
        ${PROJECT_NAME}
        src/${PROJECT_NAME}/calibration/SensorCalibration.cpp
        src/${PROJECT_NAME}/configuration/Configuration.cpp
        src/${PROJECT_NAME}/configuration/ForceTorqueFilter.cpp
        src/${PROJECT_NAME}/configuration/SensorConfiguration.cpp
        src/${PROJECT_NAME}/fsm/StateEnum.cpp
        src/${PROJECT_NAME}/Reading.cpp
        src/${PROJECT_NAME}/Rokubimini.cpp
        src/${PROJECT_NAME}/Statusword.cpp
        src/${PROJECT_NAME}/calibration/ForceTorqueCalibration.cpp
)

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
        )

target_link_libraries(${PROJECT_NAME}
        ${EIGEN3_LIBRARIES}
        ${Boost_LIBRARIES}
        )

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        )

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        )

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_${PROJECT_NAME}
            test/main.cpp
            test/TestConfiguration.cpp
            test/TestReading.cpp
            test/TestStateEnum.cpp
            test/TestStatusWord.cpp
            test/TestRokubimini.cpp)
    ament_target_dependencies(test_${PROJECT_NAME}
            ${dependencies})
    target_link_libraries(test_${PROJECT_NAME}
            ${PROJECT_NAME})
endif ()

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
add_clang_tooling(
    TARGET ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
    )
endif (cmake_clang_tools_FOUND)


ament_export_dependencies(${dependencies} Boost)
ament_export_include_directories(include ${Boost_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_package()
