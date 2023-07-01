#! /bin/zsh
# Script to build ros1_bridge package and all the messages inside BRIGDE_WS.
# Due to [this issue](https://github.com/ros2/ros1_bridge/issues/329) we need to
# remove the ros-noetic-controller-manager-msgs package from the package list, then 

# be sure that packages we want to be bridged are built, but not the bridge itself
echo "Building messages to be bridged"
cd ${BRIDGE_WS} && colcon build --symlink-install --packages-skip ros1_bridge

# remove the controller-manager-msgs package
sudo dpkg -r --force-depends ros-noetic-controller-manager-msgs

# source ros1
echo "Sourcing RO1"
source ${ROS1_INSTALL_PATH}/setup.zsh
# source ros2
echo "Sourcing RO2"
source ${ROS2_INSTALL_PATH}/setup.zsh
# source devel workspaces
echo "Sourcing Devel"
source ${CATKIN_WS}/devel/local_setup.zsh
# source install workspaces
echo "Sourcing Install"
source ${COLCON_WS}/install/local_setup.zsh
# show the sourced workspaces
echo $CMAKE_PREFIX_PATH | tr ':' '\n'
# Now jump into the BRIDGE_WS and build the bridge
zsh -c "cd ${BRIDGE_WS} && colcon build --symlink-install --packages-select ros1_bridge \
--cmake-force-configure --event-handlers console_direct+ --cmake-args -DBUILD_TESTING=FALSE"

# reinstall the controller-manager-msgs package
sudo apt --fix-broken install -y