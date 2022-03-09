# `cwru_abby` Package

This package supports the mobile manipulator platform at CWRU composed of an Invacare mobile base and an ABB IRB-120 industrial manipulator.

The robot was named ABBY.

The package is composed of several ROS Packages developed for a masters thesis with ROS Fuerte.  It has been updated to support use in simulation with ROS Melodic and Noetic.  

Most packages have not been updated.  Specifically, they still use the `manifest.xml` package specification file.  (`rosdep` does not seem to play well with that.)

The updates focus on `abby_description`

## `abby_description`

The `abby_description` package contains the description of the robot and the ability to simulate the robot.  

The ABB IRB-120 manipulator model is visible in RVIZ, but it does not appear to function in simulation.  The description for the ABB manipulator is in the `abb_common` package contained `swri-ros-pkg` directory.  

### Launch Simulation

The simulation can be started with the following command:

> `roslaunch abby_description abby_simulation.launch`

### Simulation Parameters

The `abby_simulation.launch` file has the following parameters:
- `model`:  The model to use for the simulation.  Only `abby` is currently available.  `[abby]`
- `filename`:   The absolute path of a model to use.
- `teleop`: Whether/which teleoperation interface to use.  There are teleoperation interfaces used existing support for joystick, keyboard, and the `rqt_robot_steering` GUI.  `[none|joystick|keyboard|rqt]`
- `world`:  The full path to an alternate world file.
- `physics`:    The physics engine to use for the simulation.  This is passed to Gazebo.  Currently, only ODE and Bullet are supported. '[ode|bullet]'
- `paused`: Whether to begin the simulation paused.  '[false|true]'
- `gui`:    Whether to start an RVIZ display.  '[false|true]'
- `joy_dev`:    The joystick device file to use.  '[/dev/input/js0]'

To start a simulation that can be teleoperated using at joystic, use the following command:

> `roslaunch abby_description abby_simulation.launch teleop:=joystick`

