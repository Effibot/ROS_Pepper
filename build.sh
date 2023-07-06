#!/bin/zsh
set -e

# build ROS1 workspace
echo "Building Noetic Workspace"
env -i SHELL=/bin/zsh zsh -c "source /opt/ros/noetic/setup.zsh; source ~/catkin_ws/devel/setup.zsh \
&& cd ${CATKIN_WS} && catkin build"

# build ROS2 workspace
echo "Building Foxy Workspace"
env -i SHELL=/bin/zsh zsh -c "source /opt/ros/foxy/setup.zsh; source ~/colcon_ws/install/setup.zsh \
&& colcon build --merge-install --symlink-install --event-handlers console_direct+ \
--cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On' \
-Wall -Wextra -Wpedantic"
        
