#!/bin/bash
#
# Launch the full precision landing simulation stack.
# Usage: ./run_sim.sh
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/config.sh"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

WORLD_FILE="${PROJECT_ROOT}/sim/worlds/${PX4_GZ_WORLD}.sdf"

echo -e "${GREEN}[sim] PX4_DIR:   ${PX4_DIR}${NC}"
echo -e "${GREEN}[sim] Model:     ${PX4_SIM_MODEL}${NC}"
echo -e "${GREEN}[sim] World:     ${WORLD_FILE}${NC}"
echo -e "${GREEN}[sim] Port:      ${XRCE_PORT}${NC}"

if [ ! -d "${PX4_DIR}" ]; then
    echo -e "${RED}[error] PX4 directory not found: ${PX4_DIR}${NC}"
    exit 1
fi

# Purge orphaned processes
echo -e "${YELLOW}[sim] Purging orphaned sim processes...${NC}"
pkill -f "gz sim"       2>/dev/null || true
pkill -f "gz server"    2>/dev/null || true
pkill -f MicroXRCEAgent 2>/dev/null || true
pkill -f "bin/px4"      2>/dev/null || true
sleep 2

# Start Agent in the background, hide its output to keep the terminal clean
echo -e "${GREEN}[sim] Starting micro-XRCE-DDS Agent on port ${XRCE_PORT}${NC}"
MicroXRCEAgent udp4 -p ${XRCE_PORT} > /dev/null 2>&1 &
XRCE_PID=$!
sleep 2

# Cleanup function for when the user presses Ctrl+C
cleanup() {
    echo ""
    echo -e "${YELLOW}[sim] Shutting down...${NC}"
    kill ${XRCE_PID} 2>/dev/null || true
    pkill -f "gz sim"       2>/dev/null || true
    pkill -f "gz server"    2>/dev/null || true
    pkill -f MicroXRCEAgent 2>/dev/null || true
    pkill -f "bin/px4"      2>/dev/null || true
    echo -e "${GREEN}[sim] Clean shutdown complete.${NC}"
    exit 0
}

# Trap Ctrl+C (SIGINT) and script exit
trap cleanup SIGINT SIGTERM EXIT

# Run PX4 in the FOREGROUND so you can type commands in the pxh> prompt
echo -e "${GREEN}[sim] Starting PX4 SITL in foreground...${NC}"
echo -e "${YELLOW}[sim] Type 'commander takeoff' in the pxh> prompt. Press Ctrl+C to exit.${NC}"
cd "${PX4_DIR}"
export PX4_GZ_WORLD
make px4_sitl ${PX4_SIM_MODEL}
