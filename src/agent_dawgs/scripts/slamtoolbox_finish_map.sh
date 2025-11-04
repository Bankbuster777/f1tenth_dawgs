#!/bin/bash

# ============================================================================
# SLAM Toolbox Map Saver with Retry Logic
# ============================================================================
# Usage: ./slamtoolbox_finish_map.sh <map_name> [timeout] [max_retries]
#
# Arguments:
#   map_name     - Name of the map to save (required)
#   timeout      - Timeout in seconds for each save attempt (default: 10)
#   max_retries  - Maximum number of retry attempts (default: 3)
#
# Examples:
#   ./slamtoolbox_finish_map.sh mohyun_v3           # Use defaults (10s timeout, 3 retries)
#   ./slamtoolbox_finish_map.sh mohyun_v3 20        # 20s timeout, 3 retries
#   ./slamtoolbox_finish_map.sh mohyun_v3 20 5      # 20s timeout, 5 retries
# ============================================================================

# Configuration
DIRECTORY="/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps"
TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
NAME=${1:-"maps"}
TIMEOUT=${2:-10}          # Default: 10 seconds timeout
MAX_RETRIES=${3:-3}       # Default: 3 retry attempts
FULLNAME="${DIRECTORY}/${NAME}_${TIMESTAMP}"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo ""
echo -e "${BLUE}============================================================================${NC}"
echo -e "${BLUE}SLAM Toolbox Map Saver${NC}"
echo -e "${BLUE}============================================================================${NC}"
echo -e "Map name:       ${GREEN}${NAME}${NC}"
echo -e "Output path:    ${FULLNAME}"
echo -e "Timeout:        ${YELLOW}${TIMEOUT}${NC} seconds"
echo -e "Max retries:    ${YELLOW}${MAX_RETRIES}${NC}"
echo -e "${BLUE}============================================================================${NC}"
echo ""

# Retry logic
ATTEMPT=1
SUCCESS=false

while [ $ATTEMPT -le $MAX_RETRIES ] && [ "$SUCCESS" = false ]; do
    echo -e "${BLUE}[Attempt ${ATTEMPT}/${MAX_RETRIES}]${NC} Saving map from 'map' topic..."
    echo ""

    # Run map_saver_cli with timeout
    # Note: ROS2 map_saver_cli uses a 2-second default timeout internally
    # We use the timeout command to enforce our custom timeout
    timeout ${TIMEOUT}s ros2 run nav2_map_server map_saver_cli -f "${FULLNAME}"
    EXIT_CODE=$?

    echo ""

    if [ $EXIT_CODE -eq 0 ]; then
        # Success
        echo -e "${GREEN}✓ SUCCESS:${NC} Map saved successfully!"
        echo -e "  Files created:"
        echo -e "    - ${FULLNAME}.pgm"
        echo -e "    - ${FULLNAME}.yaml"
        SUCCESS=true

    elif [ $EXIT_CODE -eq 124 ]; then
        # Timeout (exit code 124 from timeout command)
        echo -e "${RED}✗ TIMEOUT:${NC} Map save operation timed out after ${TIMEOUT} seconds"

        if [ $ATTEMPT -lt $MAX_RETRIES ]; then
            echo -e "${YELLOW}→ Retrying in 2 seconds...${NC}"
            sleep 2
        fi

    else
        # Other error
        echo -e "${RED}✗ ERROR:${NC} Map save failed with exit code ${EXIT_CODE}"

        if [ $ATTEMPT -lt $MAX_RETRIES ]; then
            echo -e "${YELLOW}→ Retrying in 2 seconds...${NC}"
            sleep 2
        fi
    fi

    ATTEMPT=$((ATTEMPT + 1))
done

echo ""
echo -e "${BLUE}============================================================================${NC}"

if [ "$SUCCESS" = true ]; then
    echo -e "${GREEN}Map saving completed successfully!${NC}"
    echo -e "Location: ${FULLNAME}"
    exit 0
else
    echo -e "${RED}Map saving failed after ${MAX_RETRIES} attempts.${NC}"
    echo ""
    echo -e "${YELLOW}Troubleshooting tips:${NC}"
    echo "  1. Check if SLAM Toolbox is running: ros2 node list | grep slam"
    echo "  2. Check if /map topic exists: ros2 topic list | grep /map"
    echo "  3. Check map topic: ros2 topic echo /map --once"
    echo "  4. Try increasing timeout: $0 ${NAME} 20"
    echo "  5. Try more retries: $0 ${NAME} 20 5"
    exit 1
fi

# ============================================================================
# Cartographer (commented out - uncomment if using Cartographer instead)
# ============================================================================
# echo "Finish trajectory..."
# ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

# echo "Save pbstream under the name ${1}"
# ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${FULLNAME}.pbstream'}"

# Check interface:
# ros2 interface show cartographer_ros_msgs/srv/WriteState