#!/bin/zsh
set -e

create_missing_links() {
    local source_dir="$1"
    local destination_dir="$2"

    # Check if the source directory is empty
    if [ -z "$(ls -A "$source_dir")" ]; then
        echo "Source ($source_dir) directory is empty. No links will be created."
        return
    fi

    # Iterate over each file in the source directory
    for file in "$source_dir"/*; do
        # Get the file name
        local filename=$(basename "$file")
        
        # Check if a symbolic link already exists for the file in the destination directory
        # or if a file with the same name exists in the destination directory
        if [ ! -e "$destination_dir/$filename" ]; then
            # Create a symbolic link in the destination directory if it doesn't exist
            ln -s "$file" "$destination_dir/$filename"
            echo "Created symbolic link for $source_dir/$filename on $destination_dir"
        else
            echo "Package or link already exists for $source_dir/$filename on $destination_dir"
        fi
    done
}

# Create symlinks for catkin_ws
create_missing_links "${workspaces}/catkin_ws/src" "${CATKIN_WS}/src/"
# Create symlinks for colcon_ws
create_missing_links "${workspaces}/colcon_ws/src" "${COLCON_WS}/src/"
# Create symlinks for bridge_ws
create_missing_links "${workspaces}/bridge_ws/src" "${BRIDGE_WS}/src/"

# update lists
echo "\nUpdating lists\n"
sudo apt-get update >> /dev/null
echo "\nUpdating ROS lists\n"
rosdep update --include-eol-distros >> /dev/null

echo "\nInstalling ROS1 dependencies\n"
# assert that ros1 dependencies are downloaded, and install them
env -i SHELL=/bin/zsh zsh -c "source ${ROS1_INSTALL_PATH}/setup.zsh \
&& cd ${CATKIN_WS} && vcs import < src/ros1.repos src\
&& rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS1_DISTRO}"

echo "\nInstalling ROS2 dependencies\n"
# assert that ros2 dependencies are downloaded, and install them
env -i SHELL=/bin/zsh zsh -c "source ${ROS2_INSTALL_PATH}/setup.zsh \
&& cd ${COLCON_WS} && vcs import < src/ros2.repos src\
&& rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS2_DISTRO}"

echo "\nInstalling BRIDGE dependencies\n"
# assert that bridge dependencies are downloaded, and install them
env -i SHELL=/bin/zsh zsh -c "cd ${BRIDGE_WS} && vcs import < src/bridge.repos src\
&& rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS2_DISTRO}"
