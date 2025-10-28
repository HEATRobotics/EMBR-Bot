# 🤖 HEAT Robotics - LiDAR Simulation Environment

ROS2 Jazzy + Gazebo Harmonic development environment for robotics simulation, fully containerized with Docker for cross-platform compatibility.

## 🚀 Quick Start

### Prerequisites

**All Platforms:**
- [Docker Desktop](https://www.docker.com/products/docker-desktop/)
- [Git](https://git-scm.com/downloads)

**Windows:**
- [VcXsrv X Server](https://sourceforge.net/projects/vcxsrv/) for GUI support
  - Run XLaunch with **"Disable access control"** checked

**macOS:**
- [XQuartz](https://www.xquartz.org/) for GUI support

**Linux:**
- X11 (usually pre-installed)

### Installation

1. **Clone the repository:**
```bash
git clone <your-repo-url>
cd LiDAR_Simulation
```

2. **Run setup (first time only):**
```bash
bash setup.sh
```

This will:
- Detect your operating system
- Build the Docker image (~5-10 minutes)
- Create the container
- Start it automatically
- Drop you into a bash shell

3. **Test Gazebo GUI:**
```bash
gz sim
```

### Daily Usage

**Start the container and enter shell:**
```bash
bash start.sh
```

**Exit the container:**
```bash
exit
```

**Stop the container:**
```bash
docker compose down
```

---

## 📁 Project Structure

```
LiDAR_Simulation/
├── Dockerfile                    # Container image definition
├── docker-compose.yml            # Base container configuration
├── docker-compose.linux.yml      # Linux-specific overrides
├── docker-compose.windows.yml    # Windows-specific overrides
├── docker-compose.mac.yml        # macOS-specific overrides
├── entrypoint.sh                 # Container startup script
├── setup.sh                      # Initial setup script (first time)
├── start.sh                      # Quick start script (daily use)
├── my_scripts/                   # Your custom scripts
├── ros2_ws/                      # ROS2 workspace
│   └── src/                      # Place your ROS2 packages here
└── scripts/                      # Additional utilities
```

---

## 🛠️ Development Workflow

### Working with ROS2

All your work happens in the **mounted directories** (`my_scripts/` and `ros2_ws/`), which sync between your host machine and the container.

**Inside the container:**

```bash
# Navigate to workspace
cd ~/ros2_ws/src

# Create a new ROS2 package
ros2 pkg create my_robot --build-type ament_python

# Build your workspace
cd ~/ros2_ws
colcon build

# Source your workspace
source install/setup.bash

# Run your nodes
ros2 run my_robot my_node
```

**Edit files on your host machine** using VS Code or any editor - changes sync automatically!

### Running Gazebo Simulations

```bash
# Launch Gazebo
gz sim

# Launch with a specific world
gz sim my_world.sdf

# Run with ROS2 bridge
ros2 launch ros_gz_sim gz_sim.launch.py
```

---

## 🔧 Common Commands

### Container Management

```bash
# Start container (if stopped)
docker start heat_jazzy

# Enter running container
docker exec -it heat_jazzy bash

# Stop container
docker compose down

# View container logs
docker logs heat_jazzy

# Rebuild image (after Dockerfile changes)
docker compose down
docker build -t heat_rb_jazzy .
docker compose up -d
```

### ROS2 Commands

```bash
# List topics
ros2 topic list

# Echo a topic
ros2 topic echo /scan

# List nodes
ros2 node list

# Run a launch file
ros2 launch <package> <launch_file>

# Check package dependencies
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🖥️ Platform-Specific Notes

### Windows

1. **Install VcXsrv** and run XLaunch before starting the container
2. **XLaunch Settings:**
   - Display: Multiple windows
   - Start: Start no client
   - **Critical:** Check "Disable access control"
3. Use **Git Bash** or **WSL** to run scripts (not Command Prompt/PowerShell)

### macOS

1. **Install XQuartz** before starting the container
2. **Enable network connections:** `xhost +localhost`
3. XQuartz must be running whenever you use GUI applications

### Linux

1. Allow Docker to access X11: `xhost +local:docker`
2. This is handled automatically by `setup.sh`

---

## 🐛 Troubleshooting

### GUI Doesn't Open

**Windows:**
- Verify VcXsrv is running (check system tray)
- Ensure "Disable access control" is checked in XLaunch
- Restart VcXsrv if needed

**macOS:**
- Verify XQuartz is running
- Run: `xhost +localhost`

**Linux:**
- Run: `xhost +local:docker`

### Permission Denied Errors

```bash
# Inside container
sudo chown -R ros:ros ~/ros2_ws
```

### Container Won't Start

```bash
# Clean restart
docker compose down
docker rm -f heat_jazzy
bash setup.sh
```

### "GID already exists" Build Error

This is handled automatically by the Dockerfile. If you see this:
```bash
docker system prune -a
bash setup.sh
```

### Line Ending Issues (Windows)

If you see `bad interpreter: /bin/bash^M`:
```bash
# Files should have LF endings, not CRLF
# This is handled by .gitattributes
git rm --cached -r .
git reset --hard
```

---

## 📦 What's Included

- **ROS2 Jazzy** - Latest ROS2 LTS release
- **Gazebo Harmonic** - Modern robotics simulator
- **ROS-Gazebo Bridge** - Seamless ROS2-Gazebo integration
- **Nav2** - Navigation stack
- **TurtleBot4 Simulator** - Example robot platform
- **Development Tools** - colcon, rosdep, vcstool, git, nano, etc.

---

## 🤝 Contributing

1. Create a feature branch: `git checkout -b feature/my-feature`
2. Make your changes in `ros2_ws/src/` or `my_scripts/`
3. Test inside the container
4. Commit: `git commit -am 'Add my feature'`
5. Push: `git push origin feature/my-feature`
6. Create a Pull Request

---

## 📚 Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [Docker Documentation](https://docs.docker.com/)
- [ROS-Gazebo Integration](https://github.com/gazebosim/ros_gz)

---

## 📄 License

[Your License Here]

---

## 👥 Team

HEAT Robotics - [Your Team Info]

---

## 🆘 Support

For issues or questions:
- Open an issue on GitHub
- Contact: [Your Contact Info]

---

**Happy Robot Building! 🤖**