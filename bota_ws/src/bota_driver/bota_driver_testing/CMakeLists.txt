cmake_minimum_required(VERSION 3.5.1)
project(bota_driver_testing)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

option(BUILD_HW_TESTS "Build the hardware tests" OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ros_testing REQUIRED)
find_package(bota_node REQUIRED)
find_package(rokubimini REQUIRED)
find_package(rokubimini_serial REQUIRED)
find_package(rokubimini_ethercat REQUIRED)
find_package(rokubimini_bus_manager REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rokubimini_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

set(dependencies bota_node rokubimini rokubimini_serial rokubimini_ethercat rokubimini_bus_manager rcl_interfaces geometry_msgs sensor_msgs rokubimini_msgs)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  if(BUILD_HW_TESTS)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    add_ros_test(
        launch/reset_service_zero_ethercat.test.py
    )
    add_ros_test(
        launch/reset_service_zero_serial.test.py
    )
    add_ros_test(
        launch/reset_service_ethercat.test.py
    )
    add_ros_test(
        launch/reset_service_serial.test.py
    )
    add_ros_test(
        launch/signal_quality_50_hz_ethercat.test.py
    )
    add_ros_test(
        launch/signal_quality_50_hz_serial.test.py
    )
    add_ros_test(
        launch/signal_quality_1000_hz_ethercat.test.py
    )
    add_ros_test(
        launch/signal_quality_1000_hz_serial.test.py
    )
    add_ros_test(
        launch/signal_quality_imu.test.py
    )
    add_ros_test(
        launch/ft_offsets_ethercat.test.py
    )
    add_ros_test(
        launch/ft_offsets_serial.test.py
    )

    ament_add_gtest_executable(${PROJECT_NAME}_boot
            src/main.cpp
            src/TestBotaDriverBoot.cpp)
    ament_target_dependencies(${PROJECT_NAME}_boot
            ${dependencies}
            )

    ament_add_gtest_executable(${PROJECT_NAME}_reset_service
            src/main.cpp
            src/TestBotaDriverResetService.cpp)
    ament_target_dependencies(${PROJECT_NAME}_reset_service
            ${dependencies}
            )

    ament_add_gtest_executable(${PROJECT_NAME}_signal_quality
            src/main.cpp
            src/TestBotaDriverSignalQuality.cpp)
    ament_target_dependencies(${PROJECT_NAME}_signal_quality
            ${dependencies}
            )

    ament_add_gtest_executable(${PROJECT_NAME}_signal_quality_imu
            src/main.cpp
            src/TestBotaDriverSignalQualityImu.cpp)
    ament_target_dependencies(${PROJECT_NAME}_signal_quality_imu
            ${dependencies}
            )

    ament_add_gtest_executable(${PROJECT_NAME}_ft_offsets
            src/main.cpp
            src/TestBotaDriverFTOffsets.cpp)
    ament_target_dependencies(${PROJECT_NAME}_ft_offsets
            ${dependencies}
            )

    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

    install(TARGETS ${PROJECT_NAME}_boot DESTINATION lib/${PROJECT_NAME})
    install(TARGETS ${PROJECT_NAME}_reset_service DESTINATION lib/${PROJECT_NAME})
    install(TARGETS ${PROJECT_NAME}_signal_quality DESTINATION lib/${PROJECT_NAME})
    install(TARGETS ${PROJECT_NAME}_signal_quality_imu DESTINATION lib/${PROJECT_NAME})
    install(TARGETS ${PROJECT_NAME}_ft_offsets DESTINATION lib/${PROJECT_NAME})

  endif()
endif()

ament_export_dependencies(${dependencies})
ament_package()
