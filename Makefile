simulate:
	roslaunch urdf_tutorial display.launch model:=urdf/learm.urdf

follower:
	rosrun learm-ros follower.py

hacking:
	# roslaunch learm-ros fake-cpp.launch &

	# Position controller docs: http://wiki.ros.org/robot_mechanism_controllers/JointPositionController
	rostopic pub --once /learm/elbow_position_controller/command std_msgs/Float64 123456
	rostopic echo -n 1 /learm/joint_states
	rosnode kill /learm/my_robot
