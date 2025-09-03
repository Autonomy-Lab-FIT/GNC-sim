#!/bin/bash
#
# PX4-ROS2-MATLAB Bridge Process Killer
# Stops all processes related to the PX4-ROS2-MATLAB bridge

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() { echo -e "\n${BLUE}$1${NC}"; }
print_success() { echo -e "${GREEN}$1${NC}"; }
print_warning() { echo -e "${YELLOW}$1${NC}"; }

kill_if_running() {
    local pattern="$1"
    local name="$2"
    
    if pgrep -f "$pattern" >/dev/null 2>&1; then
        pkill -f "$pattern" 2>/dev/null
        sleep 0.5
        # Force kill if still running
        pgrep -f "$pattern" >/dev/null 2>&1 && pkill -9 -f "$pattern" 2>/dev/null
        print_success "Stopped $name"
    else
        print_warning "No $name processes found"
    fi
}

cleanup_processes() {
    print_header "Stopping PX4-ROS2-MATLAB Bridge Processes"
    
    # Define processes to kill (pattern, display_name)
    local processes=(
        "bidirectional_bridge.py:Bidirectional Bridge"
        "MicroXRCEAgent:XRCE-DDS Agent"
        "px4_sitl:PX4 SITL"
        "QGroundControl:QGroundControl"
        "ros_gz_bridge:ROS-Gazebo Bridge"
        "gz sim:Gazebo Simulator"
        "gz server:Gazebo Server"
    )
    
    # Kill each process
    for process_info in "${processes[@]}"; do
        IFS=':' read -r pattern name <<< "$process_info"
        kill_if_running "$pattern" "$name"
    done
    
    # Brief pause to let processes terminate
    sleep 1
    
    print_success "Process cleanup completed"
}

main() {
    cleanup_processes
}

main "$@"