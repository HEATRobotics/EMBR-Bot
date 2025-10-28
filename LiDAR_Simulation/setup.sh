#!/bin/bash
set -e

echo "===================================="
echo "HEAT Robotics Docker Setup"
echo "===================================="
echo ""

# Detect platform - CHECK WSL FIRST before Linux
if [[ -n "${WSL_DISTRO_NAME}" ]] || grep -qi microsoft /proc/version 2>/dev/null; then
    PLATFORM="wsl"
    echo "✓ Detected: WSL (Windows Subsystem for Linux)"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    PLATFORM="linux"
    echo "✓ Detected: Linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    PLATFORM="mac"
    echo "✓ Detected: macOS"
elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
    PLATFORM="windows"
    echo "✓ Detected: Windows"
else
    PLATFORM="unknown"
    echo "⚠ Unknown platform, defaulting to Linux settings"
fi
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Docker Compose is installed
if ! command -v docker compose &> /dev/null; then
    echo "❌ Docker Compose is not installed. Please install Docker Compose first."
    exit 1
fi

echo "✓ Docker and Docker Compose found"
echo ""

# Platform-specific X11 setup
if [ "$PLATFORM" = "wsl" ]; then
    echo "Setting up GUI for WSL..."
    # WSL2 has built-in WSLg support - force correct display
    export DISPLAY=:0
    export WAYLAND_DISPLAY=wayland-0
    export XDG_RUNTIME_DIR=/run/user/$(id -u)
    export PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native
    echo "✓ Using WSLg for GUI support (no X server needed)"
    echo "  DISPLAY=$DISPLAY"
    echo "  XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
elif [ "$PLATFORM" = "linux" ]; then
    echo "Setting up X11 display for Linux..."
    export DISPLAY=${DISPLAY:-:0}
    xhost +local:docker 2>/dev/null || echo "⚠ Warning: Could not configure xhost (GUI may not work)"
elif [ "$PLATFORM" = "mac" ]; then
    echo "Setting up X11 display for macOS..."
    echo "⚠ Make sure XQuartz is installed and running!"
    echo "  Download from: https://www.xquartz.org/"
    export DISPLAY=host.docker.internal:0
elif [ "$PLATFORM" = "windows" ]; then
    echo "Setting up X11 display for Windows..."
    echo "⚠ Make sure VcXsrv is installed and running!"
    echo "  Download from: https://sourceforge.net/projects/vcxsrv/"
    echo "  Run XLaunch with 'Disable access control' checked"
    export DISPLAY=host.docker.internal:0.0
fi
echo ""

# Build the Docker image
echo ""
echo "Building Docker image (this may take 5-10 minutes)..."
docker build -t heat_rb_jazzy .

if [ $? -ne 0 ]; then
    echo "❌ Docker build failed!"
    exit 1
fi

echo "✓ Docker image built successfully"
echo ""

# Clean up any existing container
echo "Checking for existing container..."
if docker ps -a | grep -q heat_jazzy; then
    echo "Removing existing container..."
    docker rm -f heat_jazzy
fi

# Start the container with appropriate compose file
echo "Starting container..."
if [ "$PLATFORM" = "wsl" ] && [ -f "docker-compose.linux.yml" ]; then
    # WSL uses Linux kernel with WSLg, so use Linux config
    docker compose -f docker-compose.yml -f docker-compose.linux.yml up -d
elif [ "$PLATFORM" = "linux" ] && [ -f "docker-compose.linux.yml" ]; then
    docker compose -f docker-compose.yml -f docker-compose.linux.yml up -d
elif [ "$PLATFORM" = "mac" ] && [ -f "docker-compose.mac.yml" ]; then
    docker compose -f docker-compose.yml -f docker-compose.mac.yml up -d
elif [ "$PLATFORM" = "windows" ] && [ -f "docker-compose.windows.yml" ]; then
    docker compose -f docker-compose.yml -f docker-compose.windows.yml up -d
else
    docker compose up -d
fi

if [ $? -ne 0 ]; then
    echo "❌ Failed to start container!"
    exit 1
fi

# Wait a moment for container to fully start
sleep 2

# Check if container is running
if docker ps | grep -q heat_jazzy; then
    echo ""
    echo "===================================="
    echo "✓ Setup Complete!"
    echo "===================================="
    echo ""
    echo "Platform: $PLATFORM"
    echo ""
    echo "To access the container:"
    echo "  docker exec -it heat_jazzy bash"
    echo ""
    echo "To test Gazebo:"
    echo "  docker exec -it heat_jazzy gz sim"
    echo ""
    echo "To stop the container:"
    echo "  docker compose down"
    echo ""
    
    # Automatically enter the container
    echo "Entering container..."
    docker exec -it heat_jazzy bash
else
    echo "❌ Container failed to start. Check logs with:"
    echo "  docker logs heat_jazzy"
    exit 1
fi