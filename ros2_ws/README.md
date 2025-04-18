# What is this?

ROS2_WS is the root directory for the project

## Prerequisites

If this is the first time building the system install the python dependencies required for the project. This command installs all pythons dependencies:
```
pip install -r ./requirements.txt
```
> Note: serial and pyserial aren't the same package, even though pyserial is imported as serial.

## Usage

### Build
`colcon build` is the build system used by ros2. It is used to compile all packages within the system. 

To actually be able to get your executables, you must:
1. Make sure your new nodes are represented in `./src/embr/setup.py` and `./src/embr/launch/embr_launch.py`
2. Run `colcon build`. This compiles all packages with in the project.
3. Run `source install/setup.bash`. This executes commands within the file in order to setup the environment.

### Run

Used to run a particular ROS2 node:
```
ros2 run <package_name> <executable_name>
```

Example:
```
ros2 run embr sendRf
```


You may (probably will) run into errors here if you try to run the `ros2` command with sudo or in superuser mode in general. This is because the source call (`echo "source /opt/ros/humble/setup.bash"`) in your `~/.bashrc` file isn't read, as you are no longer operating as the same user.

If you are executing certain bits of code (mostly the ones that deal with serial communication) then you will have to run it in superuser mode and manually source the setup file.

# ALSO SEE
