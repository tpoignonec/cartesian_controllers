# Cartesian Controller Simulation
This package provides a simulated robot for controller development and testing.

## Physics engine and rationales
We build the simulator on Todorov's [MuJoCo](https://mujoco.org/) physics engine, which
has been acquired and open-sourced by Google [here](https://github.com/deepmind/mujoco).
This gives us a strong environment to realistically test control and contact phenomena with minimal dependencies.


## Build and install
We use MuJoCo in [headless mode](https://mujoco.readthedocs.io/en/latest/programming.html?highlight=headless#using-opengl)
and don't need OpenGL-related dependencies.

1. Download MuJoCo's most recent [release](https://github.com/deepmind/mujoco/releases/) and extract that somewhere.
It's a ready-to-use, pre-built library package, and we will just point to it during the build.
   ```bash
   cd $HOME
   wget https://github.com/deepmind/mujoco/releases/download/2.1.1/mujoco-2.1.1-linux-x86_64.tar.gz
   tar -xf mujoco-2.1.1-linux-x86_64.tar.gz
   ```

3. Switch to the *root* of your ROS2 workspace and build the package (*standalone*) with
   ```bash
   colcon build --symlink-install --cmake-args "-DMUJOCO_DIR=$HOME/mujoco-2.1.1" --packages-select cartesian_controller_simulation
   ```
   **Note**: The `--symlink-install` is required at the moment.

## Getting started
In a sourced terminal, run
```bash
ros2 launch cartesian_controller_simulation simulation.launch.py
```

## I don't see the robot in RViz
This might be a *locals* problem. Try setting this in your shell prior to launch:
```bash
export LC_NUMERIC="en_US.UTF-8"
```

## Hardware interfaces for controllers
Exposed interfaces per joint:

- `command_interfaces`: position, velocity, stiffness, damping
- `state_interfaces`: position, velocity
