simulate:
	roslaunch urdf_tutorial display.launch model:=urdf/learm.urdf

follower:
	rosrun learm-ros follower.py

hacking:
	rosparam load config/learm.yaml
	ROS_NAMESPACE=/learm rosrun learm-ros my_robot &

	# Found configuration over at: https://answers.ros.org/question/264444/controller-manager-wont-load-the-controllers-effort_controllers-and-joint_state_controller/
	rosservice call /learm/controller_manager/load_controller "name: 'joint1_position_controller'"
	rosservice call /learm/controller_manager/load_controller "name: 'joint_state_controller'"
	rosservice call /learm/controller_manager/switch_controller "{start_controllers: ['joint1_position_controller'], stop_controllers: [], strictness: 2}"
	rosservice call /learm/controller_manager/switch_controller "{start_controllers: ['joint_state_controller'], stop_controllers: [], strictness: 2}"
	rosservice call /learm/controller_manager/list_controllers

	rostopic echo -n 1 /learm/joint_states
	rostopic pub --once /learm/joint1_position_controller/command std_msgs/Float64 123456

	# Position controller docs: http://wiki.ros.org/robot_mechanism_controllers/JointPositionController

	rosservice call /learm/controller_manager/switch_controller "{stop_controllers: ['joint_state_controller', 'joint1_position_controller'], strictness: 2}"
	rosservice call /learm/controller_manager/unload_controller "name: 'joint1_position_controller'"
	rosservice call /learm/controller_manager/unload_controller "name: 'joint_state_controller'"
	rosservice call /learm/controller_manager/list_controllers
	rosnode kill /learm/my_robot
