#!/bin/bash

# 🚀 Enhanced Swarm System Launcher
# Convenient script for launching different swarm scenarios
# 
# Usage:
#   ./scripts/launch_swarm.sh                    # Default scenario
#   ./scripts/launch_swarm.sh high_speed         # High-speed scenario
#   ./scripts/launch_swarm.sh conservative       # Conservative scenario
#   ./scripts/launch_swarm.sh research           # Research scenario
#   ./scripts/launch_swarm.sh debug              # Debug mode
#   ./scripts/launch_swarm.sh headless           # Headless mode

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Emoji support
ROCKET="🚀"
CHECKMARK="✅"
WARNING="⚠️"
GEAR="⚙️"
MICROSCOPE="🔬"
SHIELD="🛡️"

echo -e "${BLUE}${ROCKET} Enhanced ROS2 Swarm System Launcher${NC}"
echo "=============================================="

# Check if ROS2 environment is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}${WARNING} ROS2 environment not detected. Sourcing environment...${NC}"
    source activate_swarm_ros2.sh
fi

# Parse scenario argument
SCENARIO=${1:-"default"}

# Configuration based on scenario
case $SCENARIO in
    "high_speed"|"fast"|"aggressive")
        echo -e "${RED}${ROCKET} Launching HIGH-SPEED scenario${NC}"
        CONFIG_FILE="config/scenarios/high_speed.yaml"
        FORMATION="circle"
        CONTROLLER="proportional"
        ;;
    "conservative"|"safe"|"stable")
        echo -e "${GREEN}${SHIELD} Launching CONSERVATIVE scenario${NC}"
        CONFIG_FILE="config/scenarios/conservative.yaml"
        FORMATION="triangle"
        CONTROLLER="proportional"
        ;;
    "research"|"analysis"|"metrics")
        echo -e "${PURPLE}${MICROSCOPE} Launching RESEARCH scenario${NC}"
        CONFIG_FILE="config/scenarios/research.yaml"
        FORMATION="v_shape"
        CONTROLLER="mpc"
        ;;
    "debug"|"dev"|"development")
        echo -e "${YELLOW}${GEAR} Launching DEBUG mode${NC}"
        CONFIG_FILE="config/swarm_parameters.yaml"
        FORMATION="line"
        CONTROLLER="proportional"
        DEBUG_MODE="true"
        ;;
    "headless"|"no_gui"|"background")
        echo -e "${CYAN}${GEAR} Launching HEADLESS mode${NC}"
        CONFIG_FILE="config/swarm_parameters.yaml"
        FORMATION="triangle"
        CONTROLLER="proportional"
        HEADLESS_MODE="true"
        ;;
    "default"|"")
        echo -e "${BLUE}${CHECKMARK} Launching DEFAULT scenario${NC}"
        CONFIG_FILE="config/swarm_parameters.yaml"
        FORMATION="triangle"
        CONTROLLER="proportional"
        ;;
    *)
        echo -e "${RED}❌ Unknown scenario: $SCENARIO${NC}"
        echo "Available scenarios: high_speed, conservative, research, debug, headless, default"
        exit 1
        ;;
esac

# Build launch command
LAUNCH_CMD="ros2 launch launch/enhanced_swarm_system.launch.py"
LAUNCH_CMD="$LAUNCH_CMD config_file:=$CONFIG_FILE"
LAUNCH_CMD="$LAUNCH_CMD formation:=$FORMATION"
LAUNCH_CMD="$LAUNCH_CMD controller:=$CONTROLLER"

# Add optional flags
if [ "${DEBUG_MODE:-false}" = "true" ]; then
    LAUNCH_CMD="$LAUNCH_CMD debug:=true"
fi

if [ "${HEADLESS_MODE:-false}" = "true" ]; then
    LAUNCH_CMD="$LAUNCH_CMD headless:=true enable_viz:=false"
fi

# Display configuration
echo ""
echo -e "${CYAN}📋 Configuration:${NC}"
echo -e "   ${GEAR} Config File: $CONFIG_FILE"
echo -e "   ${GEAR} Formation: $FORMATION"
echo -e "   ${GEAR} Controller: $CONTROLLER"
if [ "${DEBUG_MODE:-false}" = "true" ]; then
    echo -e "   ${GEAR} Debug Mode: Enabled"
fi
if [ "${HEADLESS_MODE:-false}" = "true" ]; then
    echo -e "   ${GEAR} Headless Mode: Enabled"
fi
echo ""

# Show helpful commands
echo -e "${BLUE}🔧 Available Commands (run in separate terminal):${NC}"
echo "   ros2 service call /swarm/set_formation std_srvs/srv/Trigger"
echo "   ros2 service call /swarm/set_controller std_srvs/srv/Trigger"
echo "   ros2 param set /swarm_controller formation.spacing 3.0"
echo "   python scripts/test_parameters.py"
echo ""

# Countdown
echo -e "${GREEN}${ROCKET} Starting in 3 seconds...${NC}"
sleep 1
echo -e "${GREEN}${ROCKET} 2...${NC}"
sleep 1
echo -e "${GREEN}${ROCKET} 1...${NC}"
sleep 1

# Launch the system
echo -e "${GREEN}${CHECKMARK} Launching Enhanced Swarm System!${NC}"
echo "Command: $LAUNCH_CMD"
echo ""

eval $LAUNCH_CMD 