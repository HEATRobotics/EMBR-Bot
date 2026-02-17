#!/bin/bash
set -e

echo "===================================="
echo "HEAT Robotics Docker Setup"
echo "===================================="
echo ""

# Detect platform - CHECK WSL FIRST before Linux
echo "🔍 Platform Detection Details:"
echo "  OSTYPE: $OSTYPE"
echo "  WSL_DISTRO_NAME: ${WSL_DISTRO_NAME:-not set}"
if [ -f /proc/version ]; then
    echo "  /proc/version: $(head -n1 /proc/version)"
fi
echo ""

if [[ -n "${WSL_DISTRO_NAME}" ]] || grep -qi microsoft /proc/version 2>/dev/null; then
    PLATFORM="wsl"
    echo "✓ Detected: WSL (Windows Subsystem for Linux)"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    PLATFORM="linux"
    echo "✓ Detected: Native Linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    PLATFORM="mac"
    echo "✓ Detected: macOS"
elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
    PLATFORM="windows"
    echo "✓ Detected: Windows (via Git Bash/Cygwin)"
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
    echo "Setting up X11 display for Native Linux..."
    
    # Display diagnostics
    echo ""
    echo "🔍 Linux Display Diagnostics:"
    echo "  Current DISPLAY: ${DISPLAY}"
    echo "  Current USER: $(whoami)"
    echo "  Current UID: $(id -u)"
    
    # Check if X11 is running
    if [ -z "$DISPLAY" ]; then
        echo "  ⚠ WARNING: DISPLAY variable is not set!"
        echo "  Setting to :0 (default)"
        export DISPLAY=:0
    fi
    
    # Check if we can connect to X server
    if command -v xset &> /dev/null; then
        if xset q &>/dev/null; then
            echo "  ✓ X server is accessible"
        else
            echo "  ⚠ WARNING: Cannot connect to X server at $DISPLAY"
            echo "  Make sure you're running this in a graphical session"
        fi
    else
        echo "  ⚠ xset not installed (optional, used for testing)"
    fi
    
    # Check for xhost
    if command -v xhost &> /dev/null; then
        echo "  ✓ xhost is available"
        echo "  Allowing Docker containers to access X server..."
        xhost +local:docker 2>/dev/null || echo "  ⚠ Could not run xhost (may need to run from GUI terminal)"
    else
        echo "  ⚠ WARNING: xhost not installed"
        echo "  Install with: sudo apt-get install x11-xserver-utils"
    fi
    
    # Check /tmp/.X11-unix
    if [ -d "/tmp/.X11-unix" ]; then
        echo "  ✓ /tmp/.X11-unix exists"
        ls -la /tmp/.X11-unix/ 2>/dev/null | head -5
    else
        echo "  ❌ /tmp/.X11-unix does NOT exist - X11 may not be running"
    fi
    
    echo ""
    
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
echo "Force Clean Build with Dockerfile from ./LiDAR_Simulation"
docker build --no-cache -t gz_sim ./LiDAR_Simulation

if [ $? -ne 0 ]; then
    echo "❌ Docker build failed!"
    exit 1
fi

echo "✓ Docker image built successfully"
echo ""

# Clean up any existing container
echo "Checking for existing container..."
if docker ps -a | grep -q ember_gz_sim; then
    echo "Removing existing container..."
    docker stop ember_gz_sim 2>/dev/null || true
    sleep 1
    docker rm -f ember_gz_sim 2>/dev/null || true
    sleep 1
fi

# Start the container with appropriate compose file
echo "Starting container with compose file for: $PLATFORM"
if [ "$PLATFORM" = "wsl" ] && [ -f "docker-compose.wsl.yml" ]; then
    # WSL uses its own config with WSLg mounts
    echo "  Using: docker-compose.yml + docker-compose.wsl.yml"
    docker compose -f docker-compose.yml -f docker-compose.wsl.yml up -d
elif [ "$PLATFORM" = "linux" ] && [ -f "docker-compose.linux.yml" ]; then
    # Native Linux - no WSLg mounts
    echo "  Using: docker-compose.yml + docker-compose.linux.yml"
    docker compose -f docker-compose.yml -f docker-compose.linux.yml up -d
elif [ "$PLATFORM" = "mac" ] && [ -f "docker-compose.mac.yml" ]; then
    echo "  Using: docker-compose.yml + docker-compose.mac.yml"
    docker compose -f docker-compose.yml -f docker-compose.mac.yml up -d
elif [ "$PLATFORM" = "windows" ] && [ -f "docker-compose.windows.yml" ]; then
    echo "  Using: docker-compose.yml + docker-compose.windows.yml"
    docker compose -f docker-compose.yml -f docker-compose.windows.yml up -d
else
    echo "  Using: docker-compose.yml only (no platform override found)"
    docker compose up -d
fi

if [ $? -ne 0 ]; then
    echo "❌ Failed to start container!"
    exit 1
fi

# Wait a moment for container to fully start
sleep 2

# Check if container is running
if docker ps | grep -q ember_gz_sim; then
    echo ""
    echo "===================================="
    echo "✓ Setup Complete!"
    echo "===================================="
    echo ""
    echo "Platform: $PLATFORM"
    
    # Verify display settings in container
    echo "🔍 Verifying container display settings..."
    CONTAINER_DISPLAY=$(docker exec ember_gz_sim printenv DISPLAY 2>/dev/null || echo "NOT SET")
    echo "  Host DISPLAY: ${DISPLAY:-NOT SET}"
    echo "  Container DISPLAY: $CONTAINER_DISPLAY"
    
    if [ "$PLATFORM" = "wsl" ]; then
        if [ "$CONTAINER_DISPLAY" != ":0" ]; then
            echo "  ⚠ Warning: Expected :0 for WSL, got $CONTAINER_DISPLAY"
        else
            echo "  ✓ Display settings correct for WSL"
        fi
    elif [ "$PLATFORM" = "windows" ]; then
        if [ "$CONTAINER_DISPLAY" != "host.docker.internal:0.0" ]; then
            echo "  ⚠ Warning: Expected host.docker.internal:0.0 for Windows, got $CONTAINER_DISPLAY"
            echo "  ⚠ Make sure VcXsrv is running!"
        else
            echo "  ✓ Display settings correct for Windows"
        fi
    elif [ "$PLATFORM" = "linux" ]; then
        if [ "$CONTAINER_DISPLAY" != "$DISPLAY" ]; then
            echo "  ⚠ Warning: Container DISPLAY ($CONTAINER_DISPLAY) doesn't match host ($DISPLAY)"
        else
            echo "  ✓ Display settings match"
        fi
    fi
    echo ""
    
    echo "To access the container:"
    echo "  docker exec -it ember_gz_sim bash"
    echo ""
    echo "To test Gazebo:"
    echo "  docker exec -it ember_gz_sim gz sim"
    echo ""
    echo "To stop the container:"
    echo "  docker compose down"
    echo ""
    
    # Offer to test GUI (Linux only)
    if [ "$PLATFORM" = "linux" ]; then
        echo "Would you like to test GUI access now? (y/n)"
        read -r response
        if [[ "$response" =~ ^[Yy]$ ]]; then
            echo ""
            echo "Testing GUI with a simple X application..."
            if docker exec ember_gz_sim bash -c "DISPLAY=$DISPLAY xeyes" 2>/dev/null &
            then
                echo "✓ If you see a small window with eyes, GUI is working!"
                echo "  Close it and press Enter to continue..."
                read -r
            else
                echo "❌ GUI test failed. Please check the error messages above."
            fi
        fi
    fi
    
    # Automatically enter the container
    echo ""
    echo "Entering container..."
    docker exec -it ember_gz_sim bash
else
    echo "❌ Container failed to start. Check logs with:"
    echo "  docker logs ember_gz_sim"
    exit 1
fi