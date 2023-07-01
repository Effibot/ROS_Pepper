#!/bin/zsh
set -e

# build ROS1 workspace
echo "Building Noetic Workspace"
env -i SHELL=/bin/zsh zsh -c "source ${ROS1_INSTALL_PATH}/setup.zsh \
&& source ${CATKIN_WS}/devel/setup.zsh \
&& cd ${CATKIN_WS} && catkin build"

# build ROS2 workspace
echo "Building Foxy Workspace"
env -i SHELL=/bin/zsh zsh -c "source ${ROS2_INSTALL_PATH}/setup.zsh \
&& cd ${COLCON_WS} && source ${COLCON_WS}/install/setup.zsh \
&& colcon build --merge-install --symlink-install --event-handlers console_direct+ \
--cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On' \
-Wall -Wextra -Wpedantic"
        
