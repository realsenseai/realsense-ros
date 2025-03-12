#!/bin/bash
# This script reads a JSON configuration file (hardware.json) and launches selected cameras based on their names.
# Only cameras with names in the allowed list will be processed.
# Usage: ./launch_cameras.sh /path/to/hardware.json
#
# Note: This script requires jq to be installed (e.g., sudo apt-get install jq).

# Check if required environment variables are set
if [ -z "$DATA" ] || [ -z "$BUCKET_VERSION" ] || [ -z "$ROBOT_HWID" ]; then
    echo "Error: Required environment variables are not set."
    echo "Please ensure DATA, BUCKET_VERSION, and ROBOT_HWID are set."
    exit 1
fi

# Check if jq is installed
if ! command -v jq &> /dev/null; then
    echo "Error: jq is not installed. Please install jq and try again."
    exit 1
fi

# Construct the JSON file path using environment variables
json_file="$DATA/$BUCKET_VERSION/config/$ROBOT_HWID/hardware.json"
# json_file="$HOME/hardware.json"

# Verify that the JSON file exists
if [ ! -f "$json_file" ]; then
    echo "Error: File $json_file does not exist."
    exit 1
fi

echo "Reading camera configurations from $json_file..."

# Define the list of allowed camera names
# allowed_cameras=("beam_camera_left" "beam_camera_right" "shoulder_camera_left" "shoulder_camera_right" "wrist_camera_left" "wrist_camera_right")
# allowed_cameras=("shoulder_camera_left" "wrist_camera_left")

allowed_cameras=("wrist_camera_left" "wrist_camera_right")

# Function to check if a camera is in the allowed list
is_allowed() {
    local cam="$1"
    for allowed in "${allowed_cameras[@]}"; do
        if [[ "$cam" == "$allowed" ]]; then
            return 0
        fi
    done
    return 1
}

tmux new-session -d -s camera_sessions
pane_count=0

# Loop through all cameras defined in the JSON file
for camera in $(jq -r '.cameras | keys[]' "$json_file"); do
    # Only process cameras that are in the allowed list
    if ! is_allowed "$camera"; then
        continue
    fi

    # Extract the serial number and exposure (if provided) for each allowed camera
    serial=$(jq -r ".cameras.\"$camera\".serial_number" "$json_file")
    exposure=$(jq -r ".cameras.\"$camera\".exposure // empty" "$json_file")
    height=$(jq -r ".cameras.\"$camera\".height // empty" "$json_file")
    width=$(jq -r ".cameras.\"$camera\".width // empty" "$json_file")
    
    echo "Launching camera: $camera"
    echo "  Serial Number: $serial"
    if [ -n "$exposure" ]; then
        echo "  Exposure: $exposure"
    fi
    if [ -n "$height" ]; then
        echo "  Height: $height"
    fi
    if [ -n "$width" ]; then
        echo "  Width: $width"
    fi
    # Do not have to worry about auto-exposure because it on by default
    ros2s="source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash"
    launch_command="$ros2s; ros2 launch realsense2_camera rs_launch.py serial_no:=_$serial camera_name:=$camera rgb_camera.color_profile:=${width}x${height}x30 align_depth.enable:=true hole_filling_filter.enable:=true"
    # spatial_filter.enable:=true
    #  initial_rest:=true"
    # colorizer.enable:=true 
    ((pane_count++))
    echo "Pane count: $pane_count"
    if [ "$pane_count" -eq 1 ]; then
        tmux send-keys -t camera_sessions "$launch_command" C-m
    else
        tmux split-window -t camera_sessions "$launch_command"
    fi

    # Ensure that the window is tiled for better visibility
    tmux select-layout -t camera_sessions tiled
    #
    # For demonstration, we'll simulate a launch with a sleep command.
    # sleep 1
    
done

echo "All specified camera launch commands executed."
