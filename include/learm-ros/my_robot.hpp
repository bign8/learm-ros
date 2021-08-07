// #include <hardware_interface/joint_command_interface.h>
// #include <hardware_interface/joint_state_interface.h>
// #include <hardware_interface/robot_hw.h>

// class MyRobot : public hardware_interface::RobotHW
// {
// public:
//   MyRobot() 
//   { 
//     // Initialization of the robot's resources (joints, sensors, actuators) and
//     // interfaces can be done here or inside init().
//     // E.g. parse the URDF for joint names & interfaces, then initialize them
//   }
//   bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
//   {
//     // Create a JointStateHandle for each joint and register them with the 
//     // JointStateInterface.
//     hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
//     jnt_state_interface.registerHandle(state_handle_a);
//     hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
//     jnt_state_interface.registerHandle(state_handle_b);
//     // Register the JointStateInterface containing the read only joints
//     // with this robot's hardware_interface::RobotHW.
//     registerInterface(&jnt_state_interface);
//     // Create a JointHandle (read and write) for each controllable joint
//     // using the read-only joint handles within the JointStateInterface and 
//     // register them with the JointPositionInterface.
//     hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
//     jnt_pos_interface.registerHandle(pos_handle_a);
//     hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
//     jnt_pos_interface.registerHandle(pos_handle_b);
//     // Register the JointPositionInterface containing the read/write joints
//     // with this robot's hardware_interface::RobotHW.
//     registerInterface(&jnt_pos_interface);
//     return true;
//   }

// private:
//   // hardware_interface::JointStateInterface gives read access to all joint values 
//   // without conflicting with other controllers.
//   hardware_interface::JointStateInterface jnt_state_interface;
//   // hardware_interface::PositionJointInterface inherits from 
//   // hardware_interface::JointCommandInterface and is used for reading and writing
//   // joint positions. Because this interface reserves the joints for write access,
//   // conflicts with other controllers writing to the same joints might occure.
//   // To only read joint positions, avoid conflicts using 
//   // hardware_interface::JointStateInterface.
//   hardware_interface::PositionJointInterface jnt_pos_interface;
//   // Data member array to store the controller commands which are sent to the 
//   // robot's resources (joints, actuators)
//   double cmd[2];
//   // Data member arrays to store the state of the robot's resources (joints, sensors)
//   double pos[2];
//   double vel[2];
//   double eff[2];
// };

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() 
  { 
    // Initialization of the robot's resources (joints, sensors, actuators) and
    // interfaces can be done here or inside init().
    // E.g. parse the URDF for joint names & interfaces, then initialize them
  }

  bool init() //ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
  {
    // Create a JointStateHandle for each joint and register them with the 
    // JointStateInterface.
    hardware_interface::JointStateHandle state_handle_a("joint1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_b("joint2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    // Register the JointStateInterface containing the read only joints
    // with this robot's hardware_interface::RobotHW.
    registerInterface(&jnt_state_interface);

    // Create a JointHandle (read and write) for each controllable joint
    // using the read-only joint handles within the JointStateInterface and 
    // register them with the JointPositionInterface.
    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("joint1"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("joint2"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_b);

    // Register the JointPositionInterface containing the read/write joints
    // with this robot's hardware_interface::RobotHW.
    registerInterface(&jnt_pos_interface);

    return true;
  }

  bool loop()
  {
    pos[0] = cmd[0];
    pos[1] = cmd[1];
    return true;
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
  double cmd[2];

  // Data member arrays to store the state of the robot's resources (joints, sensors)
  double pos[2];
  double vel[2];
  double eff[2];
};