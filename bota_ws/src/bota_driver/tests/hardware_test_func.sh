#!/bin/bash
export ROS_HOSTNAME="localhost"
cd $HOME/ros2_ws && colcon build --packages-select bota_driver_testing --ament-cmake-args -DBUILD_HW_TESTS=ON --parallel-worker 1
source $HOME/ros2_ws/install/setup.bash
# Reset service zero
launch_test src/bota_driver/bota_driver_testing/launch/reset_service_zero_serial.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi
launch_test src/bota_driver/bota_driver_testing/launch/reset_service_zero_ethercat.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi

# Reset service
launch_test src/bota_driver/bota_driver_testing/launch/reset_service_serial.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi
launch_test src/bota_driver/bota_driver_testing/launch/reset_service_ethercat.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi

# Signal quality
launch_test src/bota_driver/bota_driver_testing/launch/signal_quality_50_hz_serial.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi
launch_test src/bota_driver/bota_driver_testing/launch/signal_quality_1000_hz_serial.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi
launch_test src/bota_driver/bota_driver_testing/launch/signal_quality_50_hz_ethercat.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi
launch_test src/bota_driver/bota_driver_testing/launch/signal_quality_1000_hz_ethercat.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi

# signal quality IMU
launch_test src/bota_driver/bota_driver_testing/launch/signal_quality_imu.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi

# ft offsets
launch_test src/bota_driver/bota_driver_testing/launch/ft_offsets_serial.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi
launch_test src/bota_driver/bota_driver_testing/launch/ft_offsets_ethercat.test.py --junit-xml output.xml
NUMFAILURES=$(xmllint --xpath 'string(/testsuites/@failures)' output.xml)
NUMERRORS=$(xmllint --xpath 'string(/testsuites/@errors)' output.xml)
if [ $NUMFAILURES -ne 0 ] || [ $NUMERRORS -ne 0 ]; then
    RED='\033[0;31m'
    echo -e "${RED}Hardware test failed."
    exit 1
fi