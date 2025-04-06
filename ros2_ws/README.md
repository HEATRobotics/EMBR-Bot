# What is this?

ROS2_WS is the root directory for the project

We run `colcon build` here.

To actually be able to get your executables, you must:
1. Make sure your new nodes are represented in `./src/embr/setup.py` and `./src/embr/launch/embr_launch.py`
2. Run `source install/setup.bash`

You may (probably will) run into errors here if you try to run the `ros2` command with sudo or in superuser mode in general. This is because the source call (`echo "source /opt/ros/humble/setup.bash"`) in your `~/.bashrc` file isn't read, as you are no longer operating as the same user.

If you are executing certain bits of code (mostly the ones that deal with serial communication) then you will have to run it in superuser mode and manually source the setup file.

# ALSO SEE
This command installs all pythons dependencies.
`pip install -r ./requirements.txt`

> Note: serial and pyserial aren't the same package, even though pyserial is imported as serial.