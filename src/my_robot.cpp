#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

namespace hwi = hardware_interface;

class LeArm : public hwi::RobotHW {
public:
  LeArm() {
    // Initialization of the robot's resources (joints, sensors, actuators) and interfaces can be done here or
    // inside init(). E.g. parse the URDF for joint names & interfaces, then initialize them

    // Zero out command and measured values (TODO: remove once read/write actually do things)
    for (int i = 0; i < 6; i++) {
      cmd[i] = 0;
      pos[i] = 0;
      vel[i] = 0;
      eff[i] = 0;
    }

    // Create a JointStateHandle for each joint and register them with the JointStateInterface.
    jnt_state.registerHandle(hwi::JointStateHandle("shoulder_pan",  &pos[0], &vel[0], &eff[0]));
    jnt_state.registerHandle(hwi::JointStateHandle("shoulder_lift", &pos[1], &vel[1], &eff[1]));
    jnt_state.registerHandle(hwi::JointStateHandle("elbow",         &pos[2], &vel[2], &eff[2]));
    jnt_state.registerHandle(hwi::JointStateHandle("wrist_flex",    &pos[3], &vel[3], &eff[3]));
    jnt_state.registerHandle(hwi::JointStateHandle("wrist_roll",    &pos[4], &vel[4], &eff[4]));
    jnt_state.registerHandle(hwi::JointStateHandle("grip_right",    &pos[5], &vel[5], &eff[5]));

    // Register the JointStateInterface containing the read only joints with this robot's hwi::RobotHW.
    registerInterface(&jnt_state);

    // Create a JointHandle (read and write) for each controllable joint using the read-only joint handles within the
    // JointStateInterface and register them with the JointPositionInterface.
    jnt_pos.registerHandle(hwi::JointHandle(jnt_state.getHandle("shoulder_pan"),  &cmd[0]));
    jnt_pos.registerHandle(hwi::JointHandle(jnt_state.getHandle("shoulder_lift"), &cmd[1]));
    jnt_pos.registerHandle(hwi::JointHandle(jnt_state.getHandle("elbow"),         &cmd[2]));
    jnt_pos.registerHandle(hwi::JointHandle(jnt_state.getHandle("wrist_flex"),    &cmd[3]));
    jnt_pos.registerHandle(hwi::JointHandle(jnt_state.getHandle("wrist_roll"),    &cmd[4]));
    jnt_pos.registerHandle(hwi::JointHandle(jnt_state.getHandle("grip_right"),    &cmd[5]));

    // Register the JointPositionInterface containing the read/write joints with this robot's hwi::RobotHW.
    registerInterface(&jnt_pos);
  }

  // Read data from the robot hardware.
  void read(const ros::Time &time, const ros::Duration &period) {
    pos[0] = cmd[0]; // shoulder_pan
    pos[1] = cmd[1]; // shoulder_lift
    pos[2] = cmd[2]; // elbow
    pos[3] = cmd[3]; // wrist_flex
    pos[4] = cmd[4]; // wrist_roll
    pos[5] = cmd[5]; // grip_right
  }

  // Write commands to the robot hardware.
  void write(const ros::Time &time, const ros::Duration &period) {
    // TODO
  }

private:
  // hwi::JointStateInterface gives read access to all joint values without conflicting with other controllers.
  hwi::JointStateInterface jnt_state;

  // hwi::PositionJointInterface inherits from hwi::JointCommandInterface and is used for reading and writing
  // joint positions. Because this interface reserves the joints for write access, conflicts with other controllers
  // writing to the same joints might occure. To only read joint positions, avoid conflicts using hwi::JointStateInterface.
  hwi::PositionJointInterface jnt_pos;

  // Data member array to store the controller commands which are sent to the robot's resources (joints, actuators)
  double cmd[6];

  // Data member arrays to store the state of the robot's resources (joints, sensors)
  double pos[6];
  double vel[6];
  double eff[6];
};


int main(int argc, char **argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "my_robot");
    
    // Create an instance of your robot so that this instance knows about all the resources that are available.
    LeArm robot;

    // Create an instance of the controller manager and pass it the robot, so that it can handle its resources.
    controller_manager::ControllerManager cm(&robot);
    
    // Setup a separate thread that will be used to service ROS callbacks.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Setup for the control loop.
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0); // 10 Hz rate

    ROS_INFO("I'm Alive!");
    
    while (ros::ok()) {

        // Basic bookkeeping to get the system time in order to compute the control period.
        const ros::Time     time   = ros::Time::now();
        const ros::Duration period = time - prev_time;
        
        // Execution of the actual control loop.
        robot.read(time, period);

        // If needed, its possible to define transmissions in software by calling the
        // transmission_interface::ActuatorToJointPositionInterface::propagate() after reading the joint states.
        cm.update(time, period);

        // In case of software transmissions, use transmission_interface::JointToActuatorEffortHandle::propagate()
        // to convert from the joint space to the actuator space.
        robot.write(time, period);
        
        // All these steps keep getting repeated with the specified rate.
        rate.sleep();
    }
    return 0;
}
