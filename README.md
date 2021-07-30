# LeArm ROS Node

> LeArm is an intelligent robotic arm aiming at education.
> - https://www.hiwonder.com/store/learn/2.html

This repository houses EXPERIMENTAL components for connecting HiWonder's LeArm system to ROS the Robot Operating System.
I am completely self-taught when it comes to both of these tools, so apologies if I am not adhearing to the standard practices.
Any contributions made in good faith are welcome, especially if they are documented enough for me to understand and learn from the changes proposed.

## Components

- Experiments - Attempting to communication with LeArm via Python
- URDF - Unified Robot Description Format
- Meshes - 3d models of the links of the physical robot (referenced by URDF)

## Related Projects / Links

- [xarm](https://pypi.org/project/xarm/) - Python library to communicate with the robot over USB/Serial
  - Source Repository: https://github.com/ccourson/xArmServoController
  - Modifications requried to get working with my LeArm: 
- Photon Library](https://github.com/bharrisonb/LSC-6_Cmd_Photon) - Using a Particle Photon to communicate with the Robot.
  - Has some useful documentation committed to the project.
- [ned_ros](https://github.com/NiryoRobotics/ned_ros) - A full fledged project for a similar type of robot.
  - Some inspiratoin for this project?
- [servo controller](https://www.hiwonder.hk/products/lsc-6-hiwonder-6-ch-bluetooth-4-0-servo-controller-module-over-current-protection-remote-control-rc-parts-robot-toy-for-children?_pos=1&_sid=b04c241c9&_ss=r) - The servo controller the robot I own might be using
  - [LeswanSoul Bus Servo Communication Protocol](https://images-na.ssl-images-amazon.com/images/I/71WyZDfQwkL.pdf)
  - [LSC-24 Servo Controller User Manual](https://usermanual.wiki/Document/LSC2420Servo20Controller20User20Manual.901505409/view)
  - [Dropbox : Servo Controller](https://www.dropbox.com/sh/b3v81sb9nwir16q/AABHb3nPSC7uUFfrnu30RyrCa?dl=0&lst=)
- Learning ROS
  - [How to command join posiiton of a robot in ROS using Python](https://www.theconstructsim.com/ros-qa-149-how-to-command-joint-position-of-a-robot-in-ros-using-python/)
  - [ROS Control](https://www.rosroboticslearning.com/ros-control)
  - [How to implement ros_control on a custom robot](https://slaterobotics.medium.com/how-to-implement-ros-control-on-a-custom-robot-748b52751f2e)
  - [Dropbox : LeArm Expansion](https://drive.google.com/drive/folders/1XROWMoxATQgtth6PfmkJGh0ac2aASvfC) - Model sources provided by manufacturer.

## Running

I ran into some problems getting rviz woring with openGL, but was able to force it with:

```sh
export LIBGL_ALWAYS_SOFTWARE=1
```
