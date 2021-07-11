# LeArm ROS Node

> LeArm is an intelligent robotic arm aiming at education.
> - https://www.hiwonder.com/store/learn/2.html

This repository houses EXPERIMENTAL components for connecting HiWonder's LeArm system to ROS the Robot Operating System.
I am completely self-taught when it comes to both of these tools, so apologies if I am not adhearing to the standard practices.
Any contributions made in good faith are welcome, especially if they are documented enough for me to understand and learn from the changes proposed.

## Components

- Experiments - Attempting to communication with LeArm via Python
- URDF - Unified Robot Description Format

## Related Projects / Links

- [xarm](https://pypi.org/project/xarm/) - Python library to communicate with the robot over USB/Serial
  - Source Repository: https://github.com/ccourson/xArmServoController
  - Modifications requried to get working with my LeArm: 
- Photon Library](https://github.com/bharrisonb/LSC-6_Cmd_Photon) - Using a Particle Photon to communicate with the Robot.
  - Has some useful documentation committed to the project.
- [ned_ros](https://github.com/NiryoRobotics/ned_ros) - A full fledged project for a similar type of robot.
  - Some inspiratoin for this project?