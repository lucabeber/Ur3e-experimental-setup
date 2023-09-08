#!/bin/bash
export ROS_HOSTNAME="localhost"
cd $HOME/ros2_ws && colcon build --packages-select bota_driver_testing --ament-cmake-args -DBUILD_HW_TESTS=ON --parallel-worker 1
source $HOME/ros2_ws/install/setup.bash
launch_test src/bota_driver/bota_driver_testing/launch/bota_driver_testing_boot_async.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi
launch_test src/bota_driver/bota_driver_testing/launch/bota_driver_testing_boot_sync.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi
