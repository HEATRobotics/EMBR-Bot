# EMBR-Bot Docker Simulation - Cheat Sheet

## Quick Commands

### Start & Stop
```bash
# Start container
docker compose up -d

# Stop container
docker compose down

# Restart container
docker compose restart
```

### Access Container
```bash
# Primary terminal
docker compose exec embr-ras-sim /bin/bash

# Additional terminal
docker exec -it embr-ras-sim /bin/bash

# Run single command
docker exec embr-ras-sim ros2 topic list
```

### Build & Update
```bash
# Build image
docker compose build

# Rebuild from scratch
docker compose build --no-cache

# Pull latest code and rebuild
git pull
docker compose build
docker compose up -d
```

## Inside Container

### Launch Nodes
```bash
# All nodes
ros2 launch embr embr_launch.py

# Individual nodes
ros2 run embr getTemp
ros2 run embr getCube
ros2 run embr radio
```

### Monitor Topics
```bash
# List topics
ros2 topic list

# Echo topics
ros2 topic echo /temperature
ros2 topic echo /gps

# Topic info
ros2 topic info /temperature

# Topic frequency
ros2 topic hz /temperature
```

### Run Examples & Tests
```bash
# Examples
python3 /workspace/ros2_ws/src/embr/examples/sensor_testing_examples.py

# Tests
cd /workspace/ros2_ws
colcon test --packages-select embr
colcon test-result --verbose
```

### Development
```bash
# Rebuild after changes
cd /workspace/ros2_ws
colcon build --packages-select embr
source install/setup.bash

# Check node status
ros2 node list
ros2 node info /temperature_publisher
```

## Debugging

### View Logs
```bash
# Container logs
docker logs embr-ras-sim
docker logs -f embr-ras-sim  # Follow

# ROS logs (inside container)
ros2 run embr getTemp --ros-args --log-level debug
```

### Check Status
```bash
# Container status
docker ps
docker stats embr-ras-sim

# ROS status (inside container)
ros2 doctor
ros2 wtf  # "Where's the failure"
```

### Troubleshooting
```bash
# Restart container
docker compose restart

# Clean restart
docker compose down
docker compose up -d

# Check network
docker network ls
docker network inspect bridge

# Clean everything
docker compose down
docker system prune -a
```

## Configuration

### Change Sensor Mode
```bash
# Set in docker compose.yml
environment:
  - EMBR_SENSOR_MODE=sim  # or real, auto

# Or set in container
export EMBR_SENSOR_MODE=sim
```

### Custom Config File
```bash
# Mount custom config in docker compose.yml
volumes:
  - ./my_config.json:/workspace/ros2_ws/src/embr/config/sensors.json

# Or copy into container
docker cp my_config.json embr-ras-sim:/workspace/ros2_ws/src/embr/config/
```

## Multi-Terminal Workflow

### Terminal 1: Run Nodes
```bash
docker compose exec embr-ras-sim /bin/bash
ros2 launch embr embr_launch.py
```

### Terminal 2: Monitor Topics
```bash
docker exec -it embr-ras-sim /bin/bash
ros2 topic echo /temperature
```

### Terminal 3: Development
```bash
docker exec -it embr-ras-sim /bin/bash
cd /workspace/ros2_ws
colcon build --packages-select embr
```

## Resource Management

### Check Resources
```bash
# Docker resource usage
docker stats embr-ras-sim

# Container processes
docker top embr-ras-sim

# Disk usage
docker system df
```

### Adjust Resources
Edit `docker compose.yml`:
```yaml
deploy:
  resources:
    limits:
      cpus: '4.0'
      memory: 4G
```

### Clean Up
```bash
# Remove container
docker compose down

# Remove image
docker rmi embr-bot:simulation

# Clean all unused
docker system prune -a

# Clean volumes
docker volume prune
```

## Networking

### Check Network
```bash
# Inside container
hostname -I
ping google.com

# ROS discovery
ros2 multicast receive
```

### Multiple Containers
```bash
# Start both containers
docker compose --profile tools up -d

# They can communicate via ROS topics automatically
```

## Backup & Export

### Save Container State
```bash
# Commit container to image
docker commit embr-ras-sim embr-bot:backup

# Save image to file
docker save embr-bot:backup -o embr-backup.tar

# Load image from file
docker load -i embr-backup.tar
```

### Export Logs
```bash
# Export container logs
docker logs embr-ras-sim > container.log 2>&1

# Export ROS logs (inside container)
tar -czf logs.tar.gz /workspace/ros2_ws/log/
docker cp embr-ras-sim:/workspace/ros2_ws/log/logs.tar.gz .
```

## CI/CD

### Test in Docker
```bash
# Run tests non-interactively
docker run --rm embr-bot:simulation /bin/bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  cd /workspace/ros2_ws &&
  colcon test --packages-select embr &&
  colcon test-result --verbose
"
```

### Build and Push
```bash
# Tag for registry
docker tag embr-bot:simulation myregistry/embr-bot:latest

# Push to registry
docker push myregistry/embr-bot:latest

# Pull from registry
docker pull myregistry/embr-bot:latest
```

## Tips & Tricks

### Aliases
Add to your `~/.bashrc`:
```bash
alias embr-start='docker compose up -d'
alias embr-stop='docker compose down'
alias embr-shell='docker compose exec embr-ras-sim /bin/bash'
alias embr-logs='docker logs -f embr-ras-sim'
```

### One-Liner Launch
```bash
docker compose up -d && docker compose exec embr-ras-sim bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  ros2 launch embr embr_launch.py
"
```

### Background Launch
```bash
# Launch in background
docker exec -d embr-ras-sim bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  ros2 launch embr embr_launch.py
"

# Then monitor from outside
docker exec embr-ras-sim bash -c "
  source /opt/ros/humble/setup.bash &&
  source /workspace/ros2_ws/install/setup.bash &&
  ros2 topic list
"
```

## Common Issues

### Issue: Container won't start
```bash
# Check logs
docker compose logs embr-ras-sim

# Try recreating
docker compose down
docker compose up -d
```

### Issue: Can't connect to topics
```bash
# Check ROS_DOMAIN_ID matches
docker exec embr-ras-sim printenv | grep ROS_DOMAIN_ID

# Check network mode
docker inspect embr-ras-sim | grep NetworkMode
```

### Issue: Build fails
```bash
# Clean and rebuild
docker compose down
docker system prune -a
docker compose build --no-cache
```

### Issue: Out of disk space
```bash
# Clean Docker
docker system prune -a
docker volume prune

# Check usage
docker system df
```

---

**Quick Start**: `docker compose up -d && docker compose exec embr-ras-sim /bin/bash`
