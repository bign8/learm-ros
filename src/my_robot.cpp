#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

class LeArm : public hardware_interface::RobotHW {
public:
  LeArm() {
    // Initialization of the robot's resources (joints, sensors, actuators) and
    // interfaces can be done here or inside init().
    // E.g. parse the URDF for joint names & interfaces, then initialize them
  // }

  // // The init function is called to initialize the RobotHW from a non-realtime thread.
  // bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
    // Create a JointStateHandle for each joint and register them with the 
    // JointStateInterface.
    hardware_interface::JointStateHandle state_handle_shoulder_pan(
      "shoulder_pan", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_shoulder_pan);

    hardware_interface::JointStateHandle state_handle_shoulder_lift(
      "shoulder_lift", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_shoulder_lift);

    hardware_interface::JointStateHandle state_handle_elbow(
      "elbow", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_elbow);

    hardware_interface::JointStateHandle state_handle_wrist_flex(
      "wrist_flex", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_wrist_flex);

    hardware_interface::JointStateHandle state_handle_wrist_roll(
      "wrist_roll", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_wrist_roll);

    hardware_interface::JointStateHandle state_handle_grip_right(
      "grip_right", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_grip_right);

    // Register the JointStateInterface containing the read only joints
    // with this robot's hardware_interface::RobotHW.
    registerInterface(&jnt_state_interface);

    // Create a JointHandle (read and write) for each controllable joint
    // using the read-only joint handles within the JointStateInterface and 
    // register them with the JointPositionInterface.
    hardware_interface::JointHandle pos_handle_shoulder_pan(
      jnt_state_interface.getHandle("shoulder_pan"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_shoulder_pan);

    hardware_interface::JointHandle pos_handle_shoulder_lift(
      jnt_state_interface.getHandle("shoulder_lift"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_shoulder_lift);

    hardware_interface::JointHandle pos_handle_elbow(
      jnt_state_interface.getHandle("elbow"), &cmd[2]);
    jnt_pos_interface.registerHandle(pos_handle_elbow);

    hardware_interface::JointHandle pos_handle_wrist_flex(
      jnt_state_interface.getHandle("wrist_flex"), &cmd[3]);
    jnt_pos_interface.registerHandle(pos_handle_wrist_flex);

    hardware_interface::JointHandle pos_handle_wrist_roll(
      jnt_state_interface.getHandle("wrist_roll"), &cmd[4]);
    jnt_pos_interface.registerHandle(pos_handle_wrist_roll);

    hardware_interface::JointHandle pos_handle_grip_right(
      jnt_state_interface.getHandle("grip_right"), &cmd[5]);
    jnt_pos_interface.registerHandle(pos_handle_grip_right);

    // Register the JointPositionInterface containing the read/write joints
    // with this robot's hardware_interface::RobotHW.
    registerInterface(&jnt_pos_interface);

    // return true;
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
  // hardware_interface::JointStateInterface gives read access to all joint values 
  // without conflicting with other controllers.
  hardware_interface::JointStateInterface jnt_state_interface;

  // hardware_interface::PositionJointInterface inherits from 
  // hardware_interface::JointCommandInterface and is used for reading and writing
  // joint positions. Because this interface reserves the joints for write access,
  // conflicts with other controllers writing to the same joints might occure.
  // To only read joint positions, avoid conflicts using 
  // hardware_interface::JointStateInterface.
  hardware_interface::PositionJointInterface jnt_pos_interface;

  // Data member array to store the controller commands which are sent to the 
  // robot's resources (joints, actuators)
  double cmd[6];

  // Data member arrays to store the state of the robot's resources (joints, sensors)
  double pos[6];
  double vel[6];
  double eff[6];
};


int main(int argc, char **argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "my_robot");
    
    // Create an instance of your robot so that this instance knows about all 
    // the resources that are available.
    LeArm robot;

    // robot.init();

    // Create an instance of the controller manager and pass it the robot, 
    // so that it can handle its resources.
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
        // transmission_interface::ActuatorToJointPositionInterface::propagate()
        // after reading the joint states.
        cm.update(time, period);

        // In case of software transmissions, use 
        // transmission_interface::JointToActuatorEffortHandle::propagate()
        // to convert from the joint space to the actuator space.
        robot.write(time, period);
        
        // All these steps keep getting repeated with the specified rate.
        rate.sleep();
    }
    return 0;
}
