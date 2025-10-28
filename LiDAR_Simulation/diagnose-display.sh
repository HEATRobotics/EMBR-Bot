#!/bin/bash
# Display Diagnostics for HEAT Robotics Docker Setup

echo "========================================"
echo "🔍 HEAT Robotics Display Diagnostics"
echo "========================================"
echo ""

# 1. Platform Detection
echo "1️⃣ Platform Detection:"
if [[ -n "${WSL_DISTRO_NAME}" ]] || grep -qi microsoft /proc/version 2>/dev/null; then
    echo "  Platform: WSL"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "  Platform: Native Linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    echo "  Platform: macOS"
else
    echo "  Platform: $OSTYPE"
fi
echo "  Kernel: $(uname -r)"
echo ""

# 2. User & Session Info
echo "2️⃣ User & Session Info:"
echo "  Username: $(whoami)"
echo "  UID: $(id -u)"
echo "  GID: $(id -g)"
echo "  Session Type: ${XDG_SESSION_TYPE:-Not Set}"
echo "  Desktop: ${XDG_CURRENT_DESKTOP:-Not Set}"
echo ""

# 3. Display Environment
echo "3️⃣ Display Environment Variables:"
echo "  DISPLAY: ${DISPLAY:-Not Set}"
echo "  WAYLAND_DISPLAY: ${WAYLAND_DISPLAY:-Not Set}"
echo "  XDG_RUNTIME_DIR: ${XDG_RUNTIME_DIR:-Not Set}"
echo ""

# 4. X11 Server Status
echo "4️⃣ X11 Server Status:"
if [ -z "$DISPLAY" ]; then
    echo "  ❌ DISPLAY variable not set"
else
    if command -v xset &> /dev/null; then
        if xset q &>/dev/null 2>&1; then
            echo "  ✓ X server is accessible at $DISPLAY"
        else
            echo "  ❌ Cannot connect to X server at $DISPLAY"
        fi
    else
        echo "  ⚠ xset not installed (cannot test X connection)"
        echo "    Install with: sudo apt-get install x11-xserver-utils"
    fi
fi
echo ""

# 5. X11 Socket
echo "5️⃣ X11 Socket Check:"
if [ -d "/tmp/.X11-unix" ]; then
    echo "  ✓ /tmp/.X11-unix directory exists"
    echo "  Contents:"
    ls -la /tmp/.X11-unix/ 2>/dev/null | sed 's/^/    /'
else
    echo "  ❌ /tmp/.X11-unix does NOT exist"
fi
echo ""

# 6. Required Tools
echo "6️⃣ Required Tools:"
for tool in xhost xset docker; do
    if command -v $tool &> /dev/null; then
        echo "  ✓ $tool: $(which $tool)"
    else
        echo "  ❌ $tool: NOT INSTALLED"
    fi
done
echo ""

# 7. Docker Status
echo "7️⃣ Docker Status:"
if command -v docker &> /dev/null; then
    if docker ps &>/dev/null 2>&1; then
        echo "  ✓ Docker is running"
        if docker ps | grep -q heat_jazzy; then
            echo "  ✓ heat_jazzy container is running"
            echo ""
            echo "  Container Environment:"
            docker exec heat_jazzy printenv | grep -E 'DISPLAY|WAYLAND|XDG_RUNTIME' | sed 's/^/    /'
        else
            echo "  ⚠ heat_jazzy container is NOT running"
        fi
    else
        echo "  ❌ Docker daemon is not running or you don't have permission"
        echo "    Try: sudo usermod -aG docker $USER"
        echo "    Then log out and back in"
    fi
else
    echo "  ❌ Docker is not installed"
fi
echo ""

# 8. WSLg Check (if WSL)
if [[ -n "${WSL_DISTRO_NAME}" ]] || grep -qi microsoft /proc/version 2>/dev/null; then
    echo "8️⃣ WSLg Status:"
    if [ -d "/mnt/wslg" ]; then
        echo "  ✓ /mnt/wslg exists"
        ls -la /mnt/wslg/ 2>/dev/null | head -5 | sed 's/^/    /'
    else
        echo "  ❌ /mnt/wslg does NOT exist"
        echo "    WSLg may not be properly configured"
    fi
    echo ""
fi

# 9. Recommendations
echo "========================================"
echo "📋 Recommendations:"
echo "========================================"

if [ -z "$DISPLAY" ]; then
    echo "❌ DISPLAY not set"
    echo "   → Make sure you're running this from a graphical session"
    echo "   → Try: export DISPLAY=:0"
fi

if ! command -v xhost &> /dev/null; then
    echo "❌ xhost not installed"
    echo "   → Install: sudo apt-get install x11-xserver-utils"
fi

if [ ! -d "/tmp/.X11-unix" ]; then
    echo "❌ X11 socket missing"
    echo "   → Make sure you're running a graphical desktop environment"
    echo "   → Try logging out and back in"
fi

if command -v docker &> /dev/null; then
    if ! docker ps &>/dev/null 2>&1; then
        echo "❌ Docker permission issue"
        echo "   → Run: sudo usermod -aG docker $USER"
        echo "   → Log out and back in"
    fi
fi

echo ""
echo "========================================"
echo "Save this output and share it with the team if you need help!"
echo "========================================"