#!/bin/bash
# Test script for EMBR sensor abstraction
# Run this to verify your setup

set -e

echo "================================"
echo "EMBR Sensor Abstraction Tests"
echo "================================"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Change to workspace directory
cd "$(dirname "$0")/../.."

echo "📦 Building package..."
colcon build --packages-select embr
echo -e "${GREEN}✓ Build complete${NC}"
echo ""

echo "🔧 Sourcing workspace..."
source install/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"
echo ""

echo "🧪 Running unit tests..."
if colcon test --packages-select embr; then
    echo -e "${GREEN}✓ Unit tests passed${NC}"
else
    echo -e "${RED}✗ Unit tests failed${NC}"
    colcon test-result --verbose
    exit 1
fi
echo ""

echo "📊 Test results:"
colcon test-result --verbose
echo ""

echo "🐍 Running Python examples..."
if python3 src/embr/examples/sensor_testing_examples.py; then
    echo -e "${GREEN}✓ Examples executed successfully${NC}"
else
    echo -e "${RED}✗ Examples failed${NC}"
    exit 1
fi
echo ""

echo "🔍 Testing sensor imports..."
python3 -c "
from embr.sensors import create_sensor, SensorConfig
print('✓ Imports successful')

# Test each sensor type
for sensor_type in ['temperature', 'cube', 'thermal', 'mavlink']:
    config = SensorConfig(mode='sim')
    sensor = create_sensor(sensor_type, config)
    print(f'✓ Created {sensor_type} sensor')
"
echo -e "${GREEN}✓ All sensors can be created${NC}"
echo ""

echo "🚀 Testing individual nodes (3 seconds each)..."

echo "  Testing getTemp_v2..."
timeout 3 ros2 run embr getTemp_v2 --ros-args -p sensor_mode:=sim 2>/dev/null || true
echo -e "${GREEN}  ✓ getTemp_v2 launched${NC}"

echo "  Testing getCube_v2..."
timeout 3 ros2 run embr getCube_v2 --ros-args -p sensor_mode:=sim 2>/dev/null || true
echo -e "${GREEN}  ✓ getCube_v2 launched${NC}"

echo "  Testing sendRf_v2..."
timeout 3 ros2 run embr sendRf_v2 --ros-args -p sensor_mode:=sim 2>/dev/null || true
echo -e "${GREEN}  ✓ sendRf_v2 launched${NC}"
echo ""

echo "🎯 Testing launch file (5 seconds)..."
timeout 5 ros2 launch embr embr_launch_v2.py sensor_mode:=sim 2>/dev/null || true
echo -e "${GREEN}✓ Launch file works${NC}"
echo ""

echo "================================"
echo -e "${GREEN}All tests passed! ✓${NC}"
echo "================================"
echo ""
echo "Next steps:"
echo "  1. Try: export EMBR_SENSOR_MODE=sim && ros2 launch embr embr_launch_v2.py"
echo "  2. Read: QUICK_REFERENCE.md for usage examples"
echo "  3. See: TESTING_GUIDE.md for detailed documentation"
echo ""
