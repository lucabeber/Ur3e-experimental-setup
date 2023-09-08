#! /bin/bash
export $ROS_DISTRO
export $OS_CODE_NAME
export $OS_NAME
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/catkin_ws/devel/setup.bash
# remove cmake_code_coverage and cmake_clang_tools
blacklist=("cmake_code_coverage" "cmake_clang_tools")

CATKIN_WS=/root/catkin_ws
cd $CATKIN_WS
for file in $(find -name "package.xml"); do
	for word in ${blacklist[@]}; do
		if grep -q $word $file; then
			sed -e "s/<test_depend>$word<\/test_depend>//g" -i.backup $file
		fi
	done
done

mkdir $CATKIN_WS/packages

packages=(bota_signal_handler bota_worker bota_node rokubimini rokubimini_bus_manager rokubimini_msgs rokubimini_serial rokubimini_ethercat bota_driver)

rosdeps_file="/etc/ros/rosdep/sources.list.d/51-my-packages.list"
sudo rm $rosdeps_file
for package_name in ${packages[@]}; do
	roscd $package_name && rm -rf debian/ obj-x86_64-linux-gnu/ && bloom-generate rosdebian --os-name $OS_NAME --os-version $OS_CODE_NAME --ros-distro $ROS_DISTRO && fakeroot debian/rules binary && dpkg -i "../ros-$ROS_DISTRO-${package_name//_/-}"*.deb && cp "../ros-$ROS_DISTRO-${package_name//_/-}"*.deb $CATKIN_WS/packages && touch rosdep.yaml && echo "$package_name:" >rosdep.yaml && echo "  ubuntu: [ros-$ROS_DISTRO-${package_name//_/-}]" >>rosdep.yaml && sudo echo "yaml file://$(pwd)/rosdep.yaml" >>$rosdeps_file && rosdep update --rosdistro=$ROS_DISTRO
done
