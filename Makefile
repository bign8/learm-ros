simulate:
	roslaunch urdf_tutorial display.launch model:=urdf/learm.urdf

follower:
	rosrun learm-ros follower.py
