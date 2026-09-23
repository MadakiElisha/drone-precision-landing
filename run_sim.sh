#!/bin/bash
#
# Launch the full precision landing simulation stack.
# Usage: ./run_sim.sh
#

set -e

# Load configuration
source "$(dirname "$0")/config.sh"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}[sim] PX4_DIR: ${PX4_DIR}${NC}"
echo -e "${GREEN}[sim] Model:   ${PX4_SIM_MODEL}${NC}"
echo -e "${GREEN}[sim] Port:    ${XRCE_PORT}${NC}"
echo ""

# Verify PX4 directory exists
if [ ! -d "${PX4_DIR}" ]; then
    echo -e "${RED}[error] PX4 directory not found: ${PX4_DIR}${NC}"
    exit 1
fi

echo -e "${GREEN}[sim] Starting micro-XRCE-DDS Agent on port ${XRCE_PORT}${NC}"
MicroXRCEAgent udp4 -p ${XRCE_PORT} &
XRCE_PID=$!

sleep 2

echo -e "${GREEN}[sim] Starting PX4 SITL${NC}"
cd ${PX4_DIR}
make px4_sitl ${PX4_SIM_MODEL} &
PX4_PID=$!

# Trap Ctrl+C for clean shutdown
cleanup() {
    echo ""
    echo -e "${YELLOW}[sim] Shutting down...${NC}"
    kill $XRCE_PID 2>/dev/null || true
    kill $PX4_PID 2>/dev/null || true
    wait 2>/dev/null || true
    echo -e "${GREEN}[sim] Clean shutdown complete.${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

wait -n $XRCE_PID $PX4_PID 2>/dev/null || true
cleanup
