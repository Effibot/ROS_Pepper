# ROS-WORKSPACE

> based on althack/ros2-vscode-template [GitHub](https://github.com/athackst/vscode_ros2_workspace)

## Usage

Repository for a `ROS1(noetic)/ROS2(foxy) workspace` with `vscode` settings. Using `remote container`, once you open the workspace in vscode, it will automatically build the workspace and install the vscode extensions. Feel free to modify the `Dockerfiles` and other stuffs to your needs.

## Prerequisites

To use the an NVIDIA GPU, you need to install the `NVIDIA Container Toolkit` and setup a proper runtime.
If you want to use intel GPU or software rendering, edit the `.devcontainer/devcontainer.json` file properly.

## How to use

The `workspace` folder and its content (`catkin_ws`, `colcon_ws` and `bridge_ws`) are mounted to the container.
You can use the `catkin_ws` and `colcon_ws` folders as you would normally do with ROS1 and ROS2 workspaces.
If you want to use the ROS1/ROS2 bridge, remember to source the bridge `setup.zsh` file in order to add the bridge packages to the ROS environment.

You can find useful aliases in the `aliases.zsh` file in order to build and source the environment easily.

>the `setup.sh` file

This file is used to update ros repositories, install missing depencencies and makes symbolic links from the `workspace` folder to the `home` folder. It is executed when the container is built. With this trick, you can modify the `workspace` folder from the host machine and the changes will be reflected inside container workspace.

>the `build.sh` file

This file is used to build the workspace. At the time of container creation, it's execution is useless because we do the same thing inside the `Dockerfiles`. It is useful when you want to build the workspace in a second moment or after you add something.

>the `make_bridge.sh` file

This file is used to build the ROS1/ROS2 bridge. It executes the same steps as described in the [official documentation](https://github.com/ros2/ros1_bridge) and apply the workaround described in the [issue #329](https://github.com/ros2/ros1_bridge/issues/329) in order to make the bridge work with ROS2 foxy.

> building pipepline

The pipeline that users should follow to build the workspace is the following:
1 run `./setup.sh` to update ros repositories, install missing depencencies and makes symbolic links.
2 run `./build.sh` to build both `catkin workspace` and `colcon workspace`. This is necessary to let the bridge find the packages that need to be bridged.
3 run `./make_bridge.sh` to build the bridge.
