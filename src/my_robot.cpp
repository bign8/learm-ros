// #include <ros/ros.h>
// #include <learm-ros/my_robot.hpp>
// #include <controller_manager/controller_manager.h>

// int main(int argc, char **argv)
// {
//     // Initialize the ROS node
//     ros::init(argc, argv, "my_robot");
    
//     // Create an instance of your robot so that this instance knows about all 
//     // the resources that are available.
//     MyRobot robot;
//     // Create an instance of the controller manager and pass it the robot, 
//     // so that it can handle its resources.
//     controller_manager::ControllerManager cm(&robot);
    
//     // Setup a separate thread that will be used to service ROS callbacks.
//     ros::AsyncSpinner spinner(1);
//     spinner.start();
    
//     // Setup for the control loop.
//     ros::Time prev_time = ros::Time::now();
//     ros::Rate rate(10.0); // 10 Hz rate
    
//     while (ros::ok())
//     {
//         // Basic bookkeeping to get the system time in order to compute the control period.
//         const ros::Time     time   = ros::Time::now();
//         const ros::Duration period = time - prev_time;
        
//         // Execution of the actual control loop.
//         robot.read();
//         // If needed, its possible to define transmissions in software by calling the 
//         // transmission_interface::ActuatorToJointPositionInterface::propagate()
//         // after reading the joint states.
//         cm.update(time, period);
//         // In case of software transmissions, use 
//         // transmission_interface::JointToActuatorEffortHandle::propagate()
//         // to convert from the joint space to the actuator space.
//         robot.write();
        
//         // All these steps keep getting repeated with the specified rate.
//         rate.sleep();
//     }
//     return 0;
// }

// #include <ros/ros.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <hardware_interface/joint_state_interface.h>
// #include <hardware_interface/robot_hw.h>
// #include <controller_manager/controller_manager.h>

// class MyRobot : public hardware_interface::RobotHW
// {
// public:
//   MyRobot() 
//  { 
//    // connect and register the joint state interface
//    hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
//    jnt_state_interface.registerHandle(state_handle_a);

//    hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
//    jnt_state_interface.registerHandle(state_handle_b);

//    registerInterface(&jnt_state_interface);

//    // connect and register the joint position interface
//    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
//    jnt_pos_interface.registerHandle(pos_handle_a);

//    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
//    jnt_pos_interface.registerHandle(pos_handle_b);

//    registerInterface(&jnt_pos_interface);
//   }

// private:
//   hardware_interface::JointStateInterface jnt_state_interface;
//   hardware_interface::PositionJointInterface jnt_pos_interface;
//   double cmd[2];
//   double pos[2];
//   double vel[2];
//   double eff[2];
// };

// int main(int argc, char **argv) {
//   MyRobot robot;
//   controller_manager::ControllerManager cm(&robot);

//   while (true)
//   {
//      robot.read(robot.get_time(), robot.get_period());
//      cm.update(robot.get_time(), robot.get_period());
//      robot.write(robot.get_time(), robot.get_period());
//     //  sleep();
//   }
// }

#include <ros/ros.h>
#include <learm-ros/my_robot.hpp>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "my_robot");
    
    // Create an instance of your robot so that this instance knows about all 
    // the resources that are available.
    MyRobot robot;

    robot.init();

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
    
    while (ros::ok())
    {
        // Basic bookkeeping to get the system time in order to compute the control period.
        const ros::Time     time   = ros::Time::now();
        const ros::Duration period = time - prev_time;
        
        // Execution of the actual control loop.
        robot.read(time, period);

        // If needed, its possible to define transmissions in software by calling the 
        // transmission_interface::ActuatorToJointPositionInterface::propagate()
        // after reading the joint states.
        cm.update(time, period);

        // HACK
        robot.loop();

        // In case of software transmissions, use 
        // transmission_interface::JointToActuatorEffortHandle::propagate()
        // to convert from the joint space to the actuator space.
        robot.write(time, period);
        
        // All these steps keep getting repeated with the specified rate.
        rate.sleep();
    }
    return 0;
}
