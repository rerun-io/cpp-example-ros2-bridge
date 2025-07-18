#!/bin/bash
# Convenience script to view URDF files with Rerun in ROS2 environment

if [ $# -eq 0 ]; then
    echo "Usage: $0 <urdf_file_path>"
    echo "Example: $0 package://go2_robot_sdk/urdf/go2.urdf"
    echo "         $0 install/go2_robot_sdk/share/go2_robot_sdk/urdf/go2.urdf"
    exit 1
fi

URDF_PATH="$1"

# Change to the ROS workspace directory
cd "$(dirname "$0")/../humble_ws" || exit 1

# Source the ROS2 environment
source ./install/local_setup.bash

# If it's a package:// URI, convert it to a relative path
if [[ "$URDF_PATH" == package://* ]]; then
    # Extract package name and path from package://package_name/path
    PACKAGE_PATH="${URDF_PATH#package://}"
    PACKAGE_NAME="${PACKAGE_PATH%%/*}"
    RELATIVE_PATH="${PACKAGE_PATH#*/}"
    URDF_PATH="install/${PACKAGE_NAME}/share/${PACKAGE_NAME}/${RELATIVE_PATH}"
fi

echo "Loading URDF: $URDF_PATH"
rerun "$URDF_PATH"
