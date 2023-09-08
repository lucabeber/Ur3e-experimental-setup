#!/bin/bash
export ROS_HOSTNAME="localhost"
cd $HOME/catkin_ws && catkin build bota_driver_testing --no-deps -j1 -v --cmake-args -DBUILD_HW_TESTS=ON --catkin-make-args tests
source $HOME/catkin_ws/devel/setup.bash
rostest bota_driver_testing bota_driver_testing_hz_128_async.test && rostest bota_driver_testing bota_driver_testing_hz_128_sync.test && rostest bota_driver_testing bota_driver_testing_hz_256_async.test && rostest bota_driver_testing bota_driver_testing_hz_256_sync.test && rostest bota_driver_testing bota_driver_testing_hz_512_async.test && rostest bota_driver_testing bota_driver_testing_hz_512_sync.test && rostest bota_driver_testing bota_driver_testing_hz_64_async.test && rostest bota_driver_testing bota_driver_testing_hz_64_sync.test

cd $HOME/.ros && catkin_test_results
if [ $? -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi
