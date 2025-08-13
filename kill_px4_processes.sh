#!/bin/bash
#
# PX4-ROS2-MATLAB Bridge Process Killer
# This script kills all processes related to the PX4-ROS2-MATLAB bridge


# Color codes for terminal output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to print section headers
print_header() {
    echo -e "\n${BLUE}====================================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}====================================================================${NC}"
}

# Function to print success messages
print_success() {
    echo -e "${GREEN}[✓] $1${NC}"
}

# Function to print warning messages
print_warning() {
    echo -e "${YELLOW}[!] $1${NC}"
}

# Function to print error messages
print_error() {
    echo -e "${RED}[✗] $1${NC}"
}

# Function to kill a process by pattern
kill_process() {
    local pattern="$1"
    local display_name="$2"

    print_header "Killing $display_name Processes (Pattern: $pattern)"

    # Count the number of matching processes, excluding the script itself
    local count=$(pgrep -f "$pattern" | grep -v "$$" | wc -l)

    if [ "$count" -gt 0 ]; then
        echo "Found $count $display_name process(es)..."

        # Try gentle terminate first (SIGTERM)
        echo "Attempting gentle termination..."
        pkill -f "$pattern"
        sleep 2

        # Check if processes are still running, excluding the script itself
        local remaining=$(pgrep -f "$pattern" | grep -v "$$" | wc -l)
        if [ "$remaining" -gt 0 ]; then
            echo "$remaining processes still running, using force kill..."
            pkill -9 -f "$pattern"
            sleep 1

            # Final check, excluding the script itself
            remaining=$(pgrep -f "$pattern" | grep -v "$$" | wc -l)
            if [ "$remaining" -gt 0 ]; then
                print_error "Failed to kill all $display_name processes"
            else
                print_success "All $display_name processes killed"
            fi
        else
            print_success "All $display_name processes killed"
        fi
    else
        print_warning "No $display_name processes found"
    fi
}

# Enhanced QGroundControl killer function
kill_qgroundcontrol() {
    print_header "Killing QGroundControl Processes"
    
    # Multiple patterns for different QGC installations
    local qgc_patterns=(
        "QGroundControl"
        "qgroundcontrol" 
        "QGroundControl.AppImage"
        "qgc"
        "QGC"
        "org.mavlink.qgroundcontrol"
    )
    
    local found_any=false
    
    for pattern in "${qgc_patterns[@]}"; do
        local count=$(pgrep -f "$pattern" | grep -v "$$" | wc -l)
        if [ "$count" -gt 0 ]; then
            found_any=true
            echo "Found $count process(es) matching '$pattern'"
            pkill -f "$pattern"
        fi
    done
    
    if [ "$found_any" = true ]; then
        sleep 2
        
        # Force kill any remaining
        local still_running=false
        for pattern in "${qgc_patterns[@]}"; do
            local remaining=$(pgrep -f "$pattern" | grep -v "$$" | wc -l)
            if [ "$remaining" -gt 0 ]; then
                still_running=true
                echo "Force killing remaining processes matching '$pattern'..."
                pkill -9 -f "$pattern"
            fi
        done
        
        if [ "$still_running" = true ]; then
            sleep 1
        fi
        
        print_success "QGroundControl termination completed"
    else
        print_warning "No QGroundControl processes found"
    fi
}

# Main script execution
print_header "PX4-ROS2-MATLAB Bridge Process Killer"

# Kill all bridge related processes
kill_process "bidirectional_bridge.py" "Bidirectional Bridge"
kill_process "MicroXRCEAgent" "XRCE-DDS Agent"
kill_process "px4_sitl" "PX4 SITL"

# Enhanced QGroundControl killing
kill_qgroundcontrol

# Optional: Uncomment these if you want to kill Gazebo and ROS2 processes
# Be cautious with killing generic 'gz' processes, might affect other simulations
kill_process "gz" "Gazebo"
kill_process "gzserver" "Gazebo Server"
kill_process "gzclient" "Gazebo Client"
kill_process "ros_gz_bridge" "Camera Bridge" "ros2"


# Be cautious with killing generic 'ros2' processes
# kill_process "ros2" "ROS2"

# Check if any related processes are still running
print_header "Checking for remaining processes"

# List of patterns to check (excluding the killer script itself)
# Updated to include all QGroundControl patterns
patterns=("bidirectional_bridge.py" "MicroXRCEAgent" "px4_sitl" "QGroundControl" "qgroundcontrol" "QGroundControl.AppImage" "gz " "gzserver" "gzclient" "ros2")

# Flag to track if any processes are still running
remaining=false

# Check each pattern
for pattern in "${patterns[@]}"; do
    # Use pgrep -f and grep -v for more accurate check excluding this script
    count=$(pgrep -f "$pattern" | grep -v "$$" | wc -l)
    if [ "$count" -gt 0 ]; then
        echo -e "${YELLOW}Found $count processes matching '$pattern':${NC}"
        # Use ps and grep -v to show process details, excluding the grep command and the killer script
        ps aux | grep "$pattern" | grep -v grep | grep -v "$$"
        remaining=true
    fi
done

# Final message
if [ "$remaining" = true ]; then
    print_warning "Some processes are still running"
    echo "You may need to kill them manually using:"
    echo "  ps aux | grep <process-name-pattern>"
    echo "  kill -9 <PID>"
else
    print_success "All expected PX4-ROS2-MATLAB bridge processes have been killed"
fi

print_header "Process termination completed"