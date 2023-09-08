cmake_minimum_required(VERSION 3.5.1)
project(bota_worker)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
)

set(dependencies "rclcpp")

add_library(${PROJECT_NAME}
        src/Rate.cpp
        src/Worker.cpp
        src/WorkerManager.cpp
        )

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
        )

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        )

##########
## Test ##
##########
if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_${PROJECT_NAME}
            test/${PROJECT_NAME}_test.cpp
            test/RateTest.cpp)
    ament_target_dependencies(test_${PROJECT_NAME}
            ${dependencies})
    target_link_libraries(test_${PROJECT_NAME}
            ${PROJECT_NAME})
endif ()

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        PATTERN ".svn" EXCLUDE
        )
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} pthread)
ament_package()
