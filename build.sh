#!/bin/zsh
set -e

# build ROS1 workspace
echo "Building Noetic Workspace"
env -i SHELL=/bin/zsh zsh -c "source /opt/ros/noetic/setup.zsh; if [ -e \"~/catkin_ws/devel\" ]; then source ~/catkin_ws/devel/setup.zsh; fi \
&& cd ${CATKIN_WS} && catkin build"

# build ROS2 workspace
echo "Building Foxy Workspace"
env -i SHELL=/bin/zsh zsh -c "source /opt/ros/foxy/setup.zsh; if [ -e \"~/colcon_ws/install\" ]; then source ~/colcon_ws/install/setup.zsh; fi;  \
&& colcon build --merge-install --symlink-install --event-handlers console_direct+ \
--cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On' \
-Wall -Wextra -Wpedantic"
        
