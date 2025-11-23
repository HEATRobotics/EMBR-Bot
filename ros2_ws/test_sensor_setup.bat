@echo off
REM Test script for EMBR sensor abstraction (Windows)
REM Run this to verify your setup

echo ================================
echo EMBR Sensor Abstraction Tests
echo ================================
echo.

REM Change to workspace directory
cd /d "%~dp0"
cd ..\..

echo [92mBuilding package...[0m
call colcon build --packages-select embr
if errorlevel 1 (
    echo [91mBuild failed[0m
    exit /b 1
)
echo [92m✓ Build complete[0m
echo.

echo [92mSourcing workspace...[0m
call install\setup.bat
echo [92m✓ Workspace sourced[0m
echo.

echo [92mRunning unit tests...[0m
call colcon test --packages-select embr
if errorlevel 1 (
    echo [91mUnit tests failed[0m
    call colcon test-result --verbose
    exit /b 1
)
echo [92m✓ Unit tests passed[0m
echo.

echo [92mTest results:[0m
call colcon test-result --verbose
echo.

echo [92mRunning Python examples...[0m
python src\embr\examples\sensor_testing_examples.py
if errorlevel 1 (
    echo [91mExamples failed[0m
    exit /b 1
)
echo [92m✓ Examples executed successfully[0m
echo.

echo [92mTesting sensor imports...[0m
python -c "from embr.sensors import create_sensor, SensorConfig; print('✓ Imports successful'); [print(f'✓ Created {t} sensor') or create_sensor(t, SensorConfig(mode='sim')) for t in ['temperature', 'cube', 'thermal', 'mavlink']]"
if errorlevel 1 (
    echo [91mImport test failed[0m
    exit /b 1
)
echo [92m✓ All sensors can be created[0m
echo.

echo ================================
echo [92mAll tests passed! ✓[0m
echo ================================
echo.
echo Next steps:
echo   1. Try: set EMBR_SENSOR_MODE=sim ^&^& ros2 launch embr embr_launch_v2.py
echo   2. Read: QUICK_REFERENCE.md for usage examples
echo   3. See: TESTING_GUIDE.md for detailed documentation
echo.
