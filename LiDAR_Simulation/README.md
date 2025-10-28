# 🤖 HEAT Robotics - LiDAR Simulation Environment
ROS2 Jazzy + Gazebo Harmonic development environment for robotics simulation, fully containerized with Docker for cross-platform compatibility.
Simulation Goal: Working LiDAR Robot🤖🫨🚨

## 🚀 Quick Start
**If Container exists**
```bash
bash start.sh
```
## 🚨 If Having Display Issues
```bash
bash diagnose-display.sh
```
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

**Note if you are having issues**
```bash
docker compose down
bash setup.sh
```
**Exit the container:**
```bash
exit
```

**Stop the container:**
```bash
docker compose down
```

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
├── diagnose-display.sh           # Check Display If having issues
```

## 🛠️ Development Workflow

### Running Gazebo Simulations
```bash
# Launch Gazebo
gz sim
```

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

### ROS2 Jazzy Commands
```
...
```
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
2. Make your changes 
3. Test inside the container
4. Commit: `git commit -m 'Add my feature'`
5. Push: `git push origin feature/my-feature`
6. Create a Pull Request

---

## 📚 Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [Docker Documentation](https://docs.docker.com/)
- [ROS-Gazebo Integration](https://github.com/gazebosim/ros_gz)

---

## 👥 Team

HEAT Robotics - [Your Team Info]

---

## 🆘 Support

For issues or questions:
- Open an issue on GitHub
- Contact: [Maison Gulyas (250)258 1736]

---

**Happy Robot Building! 🤖**