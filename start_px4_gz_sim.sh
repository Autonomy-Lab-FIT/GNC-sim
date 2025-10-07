#!/bin/bash
#
# PX4-ROS2-MATLAB Bridge Launcher (Fixed Flow)
# Starts all components for the PX4-MATLAB bridge with proper menu flow
#

# Colors and configuration
GREEN='\033[0;32m'; BLUE='\033[0;34m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
export ROS_DOMAIN_ID=1

# Simple OS detection with display configuration
if grep -qi microsoft /proc/version 2>/dev/null; then
    OS_TYPE="WSL"
    export DISPLAY=:0
    echo -e "${GREEN}Running on Windows (WSL) - DISPLAY set to :0${NC}"
else
    OS_TYPE="LINUX"
    export DISPLAY=:1
    echo -e "${GREEN}Running on Linux - DISPLAY set to :1${NC}"
fi

# Directories
ALGORITHMS_SCRIPT_DIR="/root/sim_ws/GNC_algorithms"
CORE_SCRIPT_DIR="/root/GNC_core"
WORLDS_DIR="$ALGORITHMS_SCRIPT_DIR/gazebo_sim/worlds"
LOGS_DIR="$ALGORITHMS_SCRIPT_DIR/logs"

# Drone and world definitions
DRONE_OPTIONS=(
    "gz_x500:Standard X500 quadcopter - no sensors"
    "gz_x500_lidar_2d:X500 with 2D LiDAR sensor"
    "gz_x500_mono_cam:X500 with monocular camera"
    "gz_x500_vision:X500 with forward-facing vision camera"
    "gz_x500_depth:X500 with RGB-D depth camera"
    "gz_standard_vtol:Standard VTOL QuadPlane"
    "gz_rc_cessna:RC Cessna aircraft"
)

DRONE_ALIASES=(
    "x500:gz_x500" "vision:gz_x500_vision" "depth:gz_x500_depth"
    "mono:gz_x500_mono_cam" "camera:gz_x500_mono_cam" "lidar:gz_x500_lidar_2d"
    "vtol:gz_standard_vtol" "cessna:gz_rc_cessna" "plane:gz_rc_cessna"
)

# Utility functions
print_header() { 
    echo -e "\n${BLUE}==================================================="
    echo -e "${BLUE}$1"
    echo -e "${BLUE}===================================================${NC}"
}

print_success() { echo -e "${GREEN}[✓] $1${NC}"; }
print_warning() { echo -e "${YELLOW}[!] $1${NC}"; }
print_error() { echo -e "${RED}[✗] $1${NC}"; }
check_process() { 
    case "$1" in
        "ros_gz_bridge_sensors")
            pgrep -f "parameter_bridge" >/dev/null 2>&1
            ;;
        *)
            pgrep -f "$1" >/dev/null 2>&1
            ;;
    esac
}

# Safe cleanup
cleanup_processes() {
    print_header "Stopping existing processes"
    pgrep -f "bidirectional_bridge.py" >/dev/null && { pkill -f "bidirectional_bridge.py"; print_success "Killed bidirectional bridge"; } || print_warning "No bidirectional bridge found"
    pgrep -f "MicroXRCEAgent" >/dev/null && { pkill -f "MicroXRCEAgent"; print_success "Killed XRCE-DDS Agent"; } || print_warning "No XRCE-DDS Agent found"
    pgrep -f "px4_sitl" >/dev/null && { pkill -f "px4_sitl"; print_success "Killed PX4 SITL"; } || print_warning "No PX4 SITL found"
    pgrep -f "QGroundControl" >/dev/null && { pkill -f "QGroundControl"; print_success "Killed QGroundControl"; } || print_warning "No QGroundControl found"
    pgrep -f "ros_gz_bridge" >/dev/null && { pkill -f "ros_gz_bridge"; print_success "Killed ROS-Gazebo bridge"; } || print_warning "No ROS-Gazebo bridge found"
    pgrep -f "gz " >/dev/null && { pkill -f "gz "; print_success "Killed Gazebo"; } || print_warning "No Gazebo found"
    sleep 2
    print_success "Process cleanup completed"
}

# Fixed menu function
show_selection_menu() {
    local title="$1"; shift; local options=("$@")
    echo "====================================================================="
    echo "                    $title"
    echo "====================================================================="
    echo ""
    for i in "${!options[@]}"; do
        local desc="${options[i]#*:}"
        echo "  $((i+1))) $desc"
    done
    echo ""
    echo "====================================================================="
}

# Get selection
get_selection() {
    local input="$1"; shift; local options=("$@")
    
    if [[ "$input" =~ ^[0-9]+$ ]] && [ "$input" -ge 1 ] && [ "$input" -le "${#options[@]}" ]; then
        echo "${options[$((input-1))]%%:*}"
        return
    fi
    
    for option in "${options[@]}"; do
        local name="${option%%:*}"
        if [ "$input" = "$name" ]; then
            echo "$name"
            return
        fi
    done
    
    if [ "${options[0]}" = "${DRONE_OPTIONS[0]}" ]; then
        for alias in "${DRONE_ALIASES[@]}"; do
            if [ "$input" = "${alias%%:*}" ]; then
                echo "${alias#*:}"
                return
            fi
        done
    fi
    
    echo "${options[0]%%:*}"
}

# Get available worlds
get_world_options() {
    local worlds=()
    if [ -d "$WORLDS_DIR" ]; then
        while IFS= read -r world; do
            [ -n "$world" ] && worlds+=("$world:$world")
        done < <(ls "$WORLDS_DIR"/*.sdf 2>/dev/null | xargs -n 1 basename -s .sdf | sort)
    fi
    [ ${#worlds[@]} -eq 0 ] && worlds=("default:default")
    printf '%s\n' "${worlds[@]}"
}

# Fixed process management
start_process() {
    local name="$1" command="$2" logfile="$3" wait_msg="$4" wait_time="$5" visible="$6"
    
    check_process "$name" && { print_warning "$name is already running"; return 0; }
    
    local abs_logfile="$LOGS_DIR/$(basename "$logfile")"
    
    if [ "$visible" = "visible" ] && [ -n "$DISPLAY" ]; then
        if command -v gnome-terminal >/dev/null; then
            print_success "Starting $name in new terminal"
            gnome-terminal --title="$name" -- bash -c "$command; echo 'Process exited. Press Enter to close.'; read" &
        else
            visible=""
        fi
    fi
    
    if [ "$visible" != "visible" ]; then
        print_success "Starting $name in background (logged to $abs_logfile)"
        {
            nohup bash -c "$command" > "$abs_logfile" 2>&1 &
            echo $! > "$LOGS_DIR/${name}.pid"
        }
    fi
    
    if [ -n "$wait_time" ]; then
        echo -n "$wait_msg"
        for ((i=0; i<wait_time; i++)); do echo -n "."; sleep 1; done
        echo -e " ${GREEN}done${NC}"
    fi
    
    sleep 1  # Give process time to start
    if check_process "$name"; then
        print_success "$name started successfully"
        return 0
    else
        print_error "$name failed to start (check log: $abs_logfile)"
        return 1
    fi
}

# Setup environment
setup_environment() {
    print_header "Setting up environment"
    mkdir -p "$LOGS_DIR"
    
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_fastdds_config.xml
    export QT_QPA_PLATFORM=xcb
    export GZ_CONFIG_PATH=/usr/share/gz
    export GZ_SIM_RESOURCE_PATH=/usr/share/gz/gz-sim8
    export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
    # export DISPLAY=:0
    [ -f /opt/ros/jazzy/setup.bash ] && { source /opt/ros/jazzy/setup.bash; print_success "Sourced ROS2 Jazzy"; } || print_warning "ROS2 Jazzy setup not found"
    [ -f "$CORE_SCRIPT_DIR/internal/px4_ros2_ws/install/setup.bash" ] && { source "$CORE_SCRIPT_DIR/internal/px4_ros2_ws/install/setup.bash"; print_success "Sourced PX4 ROS2 workspace"; } || print_warning "PX4 ROS2 workspace not found"
}

setup_sensor_bridge() {
    local drone="$1" world="$2"
    local base_config="$CORE_SCRIPT_DIR/internal/ros_gz_bridge/sensor_bridge.yaml"
    local sensor_config="/tmp/sensor_bridge_${world}.yaml"
    
    if [[ "$drone" == *"lidar"* ]]; then
        print_header "Setting up LiDAR sensor bridge"
        # Extract LIDAR_SENSORS section - from section marker to next section marker
        sed -n '/# LIDAR_SENSORS/,/# CAMERA_SENSORS/p' "$base_config" | \
        sed '/# CAMERA_SENSORS/d' | \
        grep -v "section:" | \
        sed "s|/world/default/|/world/${world}/|g" > "$sensor_config"
        
    elif [[ "$drone" == *"camera"* ]] || [[ "$drone" == *"mono_cam"* ]]; then
        print_header "Setting up camera sensor bridge"
        # Extract CAMERA_SENSORS section - from section marker to end of file
        sed -n '/# CAMERA_SENSORS/,$p' "$base_config" | \
        grep -v "section:" | \
        sed "s|/world/default/|/world/${world}/|g" > "$sensor_config"
        
    else
        return 0
    fi
    
    sleep 5
    start_process "ros_gz_bridge_sensors" "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$sensor_config" "sensor_bridge.log" "Starting bridge" 3
    rm -f "$sensor_config" &
}

# Display status
display_status() {
    print_header "System Status"
    local ip_address=$(hostname -I | awk '{print $1}')
    local all_running=true
    
    local processes=("px4_sitl:PX4 SITL" "MicroXRCEAgent:XRCE-DDS Agent" "bidirectional_bridge.py:Bidirectional Bridge" "QGroundControl.AppImage:QGroundControl")
    
    for process_info in "${processes[@]}"; do
        local pattern="${process_info%%:*}"
        local name="${process_info#*:}"
        if check_process "$pattern"; then
            print_success "$name is running"
        else
            print_error "$name is NOT running"
            all_running=false
        fi
    done
    
    if [ "$all_running" = true ]; then
        print_header "All components running successfully!"
        echo -e "Container IP: ${GREEN}$ip_address${NC} | Bridge Port: ${GREEN}8766${NC}"
        echo -e "Selected: ${GREEN}$SELECTED_DRONE${NC} in ${GREEN}$SELECTED_WORLD${NC} world"
        echo ""
        print_success "All processes running in background"
    else
        print_header "Some components failed - check logs in $LOGS_DIR/"
    fi
    
    echo ""
    echo -e "${BLUE}Management Commands:${NC}"
    echo -e "  View logs: ${YELLOW}tail -f $LOGS_DIR/<process>.log${NC}"
    echo -e "  Stop all: ${YELLOW}${ALGORITHMS_SCRIPT_DIR}/kill_px4_processes.sh${NC}"
}

# Main execution
main() {
    cleanup_processes
    setup_environment
    
    # Drone selection
    if [ $# -gt 0 ] && [ -n "$1" ]; then
        SELECTED_DRONE=$(get_selection "$1" "${DRONE_OPTIONS[@]}")
        print_success "Using drone from command line: $SELECTED_DRONE"
    else
        show_selection_menu "Available Drone Models" "${DRONE_OPTIONS[@]}"
        echo ""
        read -p "Select Drone Models (1-${#DRONE_OPTIONS[@]}) [default: 1]: " choice
        choice=${choice:-1}
        
        # Fixed validation - check if choice is within valid range
        if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le "${#DRONE_OPTIONS[@]}" ]; then
            SELECTED_DRONE=$(get_selection "$choice" "${DRONE_OPTIONS[@]}")
            print_success "Selected: $SELECTED_DRONE"
        else
            echo "Invalid selection. Using default..."
            SELECTED_DRONE=$(get_selection "1" "${DRONE_OPTIONS[@]}")
            print_success "Selected: $SELECTED_DRONE"
        fi
    fi
    
    # World selection
    world_options=($(get_world_options))
    if [ $# -gt 1 ] && [ -n "$2" ]; then
        SELECTED_WORLD=$(get_selection "$2" "${world_options[@]}")
        print_success "Using world from command line: $SELECTED_WORLD"
    else
        show_selection_menu "Available Gazebo Worlds" "${world_options[@]}"
        echo ""
        read -p "Select Gazebo Worlds (1-${#world_options[@]}) [default: 1]: " world_choice
        world_choice=${world_choice:-1}
        
        # Fixed validation - check if choice is within valid range
        if [[ "$world_choice" =~ ^[0-9]+$ ]] && [ "$world_choice" -ge 1 ] && [ "$world_choice" -le "${#world_options[@]}" ]; then
            SELECTED_WORLD=$(get_selection "$world_choice" "${world_options[@]}")
            print_success "Selected: $SELECTED_WORLD"
        else
            echo "Invalid selection. Using default..."
            SELECTED_WORLD=$(get_selection "1" "${world_options[@]}")
            print_success "Selected: $SELECTED_WORLD"
        fi
    fi
    
    # # Custom spawn position support (arguments 3-6)
    # SPAWN_X=${3:-0.0}; SPAWN_Y=${4:-0.0}; SPAWN_Z=${5:--2.0}; SPAWN_YAW=${6:-0.0}
    # export PX4_SIM_INIT_LOCATION_X=$SPAWN_X PX4_SIM_INIT_LOCATION_Y=$SPAWN_Y PX4_SIM_INIT_LOCATION_Z=$SPAWN_Z PX4_SIM_INIT_YAW=$SPAWN_YAW

    # Custom spawn position support (arguments 3-6)
    SPAWN_X=${3:-0.0}; SPAWN_Y=${4:-0.0}; SPAWN_Z=${5:-0.0}; SPAWN_YAW=${6:-0.0}

    # Convert yaw from degrees to radians
    SPAWN_YAW_RAD=$(echo "scale=6; $SPAWN_YAW * 3.14159265359 / 180" | bc -l 2>/dev/null || echo "0.0")

    # Set PX4_GZ_MODEL_POSE for spawn position (x,y,z,roll,pitch,yaw)
    export PX4_GZ_MODEL_POSE="$SPAWN_X,$SPAWN_Y,$SPAWN_Z,0,0,$SPAWN_YAW_RAD"
    export PX4_SIM_MODEL=$SELECTED_DRONE
    
    # Start core processes
    print_header "Starting PX4 simulation stack"
    # start_process "px4_sitl" "cd $CORE_SCRIPT_DIR/PX4-Autopilot && UXRCE_DDS_CFG=udp://:8888 PX4_GZ_WORLD=$SELECTED_WORLD make px4_sitl $SELECTED_DRONE" "px4_sitl.log" "Starting PX4 SITL" 8
    # start_process "px4_sitl" "cd $CORE_SCRIPT_DIR/PX4-Autopilot && UXRCE_DDS_CFG=udp://:8888 PX4_GZ_WORLD=$SELECTED_WORLD PX4_SIM_INIT_LOCATION_X=$SPAWN_X PX4_SIM_INIT_LOCATION_Y=$SPAWN_Y PX4_SIM_INIT_LOCATION_Z=$SPAWN_Z PX4_SIM_INIT_YAW=$SPAWN_YAW make px4_sitl $SELECTED_DRONE" "px4_sitl.log" "Starting PX4 SITL" 8
    start_process "px4_sitl" "cd $CORE_SCRIPT_DIR/PX4-Autopilot && UXRCE_DDS_CFG=udp://:8888 PX4_GZ_WORLD=$SELECTED_WORLD PX4_GZ_MODEL_POSE='$SPAWN_X,$SPAWN_Y,$SPAWN_Z,0,0,$SPAWN_YAW_RAD' PX4_SIM_MODEL=$SELECTED_DRONE make px4_sitl $SELECTED_DRONE" "px4_sitl.log" "Starting PX4 SITL" 8
    start_process "MicroXRCEAgent" "$CORE_SCRIPT_DIR/internal/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888" "xrce_dds_agent.log" "Starting XRCE-DDS Agent" 3
    
    print_success "Spawn position set to X:$SPAWN_X Y:$SPAWN_Y Z:$SPAWN_Z Yaw:$SPAWN_YAW"

    # Setup sensor bridge if needed
    setup_sensor_bridge "$SELECTED_DRONE" "$SELECTED_WORLD"
    
    # Start remaining processes
    start_process "bidirectional_bridge.py" "python3 $CORE_SCRIPT_DIR/internal/ros2_bridge/bidirectional_bridge.py" "bidirectional_bridge.log" "Starting bridge" 3
    
    # # QGroundControl, windows
    # if [ "$SHOW_QGC" = "1" ]; then
    # start_process "QGroundControl.AppImage" 'su - qgcuser -c "export DISPLAY=:0 && cd /home/qgcuser/Downloads && ./QGroundControl.AppImage"' "qgroundcontrol.log" "Starting QGroundControl" 5 "visible"
    # else
    #     start_process "QGroundControl.AppImage" 'su - qgcuser -c "export DISPLAY=:0 && cd /home/qgcuser/Downloads && ./QGroundControl.AppImage"' "qgroundcontrol.log" "Starting QGroundControl" 5
    # fi

    # # QGroundControl, linux
    # if [ "$SHOW_QGC" = "1" ]; then
    # start_process "QGroundControl.AppImage" 'su - qgcuser -c "cd /home/qgcuser/Downloads && ./QGroundControl.AppImage"' "qgroundcontrol.log" "Starting QGroundControl" 5 "visible"
    # else
    #     start_process "QGroundControl.AppImage" 'su - qgcuser -c "cd /home/qgcuser/Downloads && ./QGroundControl.AppImage"' "qgroundcontrol.log" "Starting QGroundControl" 5
    # fi
    
    # QGroundControl, auto detect OS
    if [ "$SHOW_QGC" = "1" ]; then
        start_process "QGroundControl.AppImage" 'su - qgcuser -c "export DISPLAY='$DISPLAY' && cd /home/qgcuser/Downloads && ./QGroundControl.AppImage"' "qgroundcontrol.log" "Starting QGroundControl" 5 "visible"
    else
        start_process "QGroundControl.AppImage" 'su - qgcuser -c "export DISPLAY='$DISPLAY' && cd /home/qgcuser/Downloads && ./QGroundControl.AppImage"' "qgroundcontrol.log" "Starting QGroundControl" 5
    fi
    export ROS_DOMAIN_ID=1
    display_status
    export ROS_DOMAIN_ID=1
}

# Run main function with all arguments
main "$@"