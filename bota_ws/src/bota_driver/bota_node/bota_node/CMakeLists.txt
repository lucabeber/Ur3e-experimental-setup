cmake_minimum_required(VERSION 3.5.1)
project(bota_node)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bota_worker REQUIRED)
find_package(bota_signal_handler REQUIRED)

include_directories(include)

set(dependencies "rclcpp" "bota_worker" "bota_signal_handler")

add_library(${PROJECT_NAME}
        src/Node.cpp
        )

ament_target_dependencies(${PROJECT_NAME}
        ${dependencies}
        )

##########
## Test ##
##########
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    add_default_clang_tooling()
endif (cmake_clang_tools_FOUND)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_${PROJECT_NAME}
            test/EmptyTests.cpp
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    ament_target_dependencies(test_${PROJECT_NAME} ${dependencies})
    target_link_libraries(test_${PROJECT_NAME}
            ${PROJECT_NAME})
    if (cmake_code_coverage_FOUND)
        add_gtest_coverage()
    endif (cmake_code_coverage_FOUND)
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
