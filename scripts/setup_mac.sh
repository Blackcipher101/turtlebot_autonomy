#!/bin/bash
# Phase 0 — macOS Apple Silicon setup for TurtleBot3 autonomy stack

set -e

BOLD='\033[1m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BOLD}TurtleBot3 Autonomy Stack — macOS Apple Silicon Setup${NC}"
echo "======================================================="

# 1. Homebrew
if ! command -v brew &>/dev/null; then
    echo -e "${YELLOW}Installing Homebrew...${NC}"
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    eval "$(/opt/homebrew/bin/brew shellenv)"
else
    echo -e "${GREEN}✓ Homebrew already installed${NC}"
fi

# 2. XQuartz
if ! [ -d "/Applications/Utilities/XQuartz.app" ]; then
    echo -e "${YELLOW}Installing XQuartz (X11 display server for GUI apps)...${NC}"
    brew install --cask xquartz
    echo -e "${RED}ACTION REQUIRED: Log out and log back in after XQuartz installs.${NC}"
    echo -e "Then re-run this script."
else
    echo -e "${GREEN}✓ XQuartz already installed${NC}"
fi

# 3. Configure XQuartz to allow network connections (needed for Docker X11 forwarding)
echo -e "${YELLOW}Configuring XQuartz network access...${NC}"
defaults write org.xquartz.X11 nolisten_tcp -bool false
defaults write org.xquartz.X11 app_to_run /usr/bin/true
defaults write org.xquartz.X11 no_auth -bool false
echo -e "${GREEN}✓ XQuartz configured${NC}"

# 4. Docker Desktop
if ! command -v docker &>/dev/null; then
    echo -e "${YELLOW}Installing Docker Desktop...${NC}"
    brew install --cask docker
    echo -e "${RED}ACTION REQUIRED: Open Docker Desktop and complete setup.${NC}"
    echo "Ensure these Docker Desktop settings:"
    echo "  - Resources > Memory: 8 GB minimum (12 GB recommended)"
    echo "  - Resources > CPUs: 4 minimum"
    echo "  - Features in Development: Enable Rosetta (for x86 compatibility if needed)"
else
    echo -e "${GREEN}✓ Docker already installed: $(docker --version)${NC}"
fi

# 5. Allow XQuartz connections from localhost
echo -e "${YELLOW}Opening XQuartz and allowing localhost connections...${NC}"
open -a XQuartz 2>/dev/null || true
sleep 2
xhost +localhost 2>/dev/null || echo "  (XQuartz not running yet — run 'xhost +localhost' after opening XQuartz)"

# 6. Set DISPLAY for Docker
DISPLAY_VAL=":0"
echo ""
echo -e "${BOLD}Add these lines to your ~/.zshrc or ~/.bash_profile:${NC}"
echo "export DISPLAY=:0"
echo "xhost +localhost 2>/dev/null || true"

# 7. Docker X11 display test instructions
echo ""
echo -e "${BOLD}===== Next Steps =====${NC}"
echo "1. Make sure XQuartz is running: open -a XQuartz"
echo "2. In XQuartz menu → Preferences → Security → check 'Allow connections from network clients'"
echo "3. Run: xhost +localhost"
echo "4. Build the Docker image:"
echo "   cd $(dirname $0)/../docker && docker compose build"
echo "5. Start a container:"
echo "   docker compose run --rm ros2"
echo ""
echo -e "${BOLD}===== Apple Silicon Notes =====${NC}"
echo "• The Docker image uses linux/arm64 (Ubuntu 22.04 native ARM)"
echo "• No GPU passthrough — all ML inference runs on CPU"
echo "• Gazebo may be slow; reduce physics rate or world complexity if needed"
echo "• If Gazebo shows black screen: set LIBGL_ALWAYS_SOFTWARE=1 in docker-compose.yml"
echo "• Expected Gazebo FPS: 10-30 on M2, 5-15 on M1"
echo ""
echo -e "${GREEN}Setup script complete.${NC}"
