cmake_minimum_required(VERSION 3.5.1)
project(bota_signal_handler)

set(CMAKE_CXX_STANDARD 14)

if (CMAKE_COMPILER_IS_GNUCXX)
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7.0)
        add_definitions(-faligned-new)
    endif ()
endif ()

find_package(ament_cmake REQUIRED)

###########
## Build ##
###########

include_directories(include)

add_library(
        ${PROJECT_NAME}
        src/SignalHandler.cpp
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
    ament_target_dependencies(test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
            ${PROJECT_NAME})
#    if (cmake_code_coverage_FOUND)
#        add_gtest_coverage()
#    endif (cmake_code_coverage_FOUND)
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
