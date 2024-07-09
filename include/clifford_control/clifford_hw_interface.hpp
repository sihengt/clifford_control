#ifndef CLIFFORD_HW_INTERFACE_HPP
#define CLIFFORD_HW_INTERFACE_HPP

#include "ros/ros.h"
#include "hardware_interface/robot_hw.h"
#include "hardware_interface/joint_command_interface.h"

class CliffordHWInterface : public hardware_interface::RobotHW
{
public:
    CliffordHWInterface();
    void read();
    void write();
    
    enum JointIndices
    {
        F_STEER,
        R_STEER,
        F_WHEEL,
        R_WHEEL,
        N_JOINTS
    };

private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    
    void stateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    double cmd_pos_[N_JOINTS];
    double cmd_vel_[N_JOINTS];
    double pos_[N_JOINTS];
    double vel_[N_JOINTS];
    double eff_[N_JOINTS];
    
    ros::NodeHandle nh_;

    ros::Subscriber steering_sub_;
    ros::Subscriber wheel_speed_sub_;

    ros::Publisher steering_pub_;
    ros::Publisher wheel_speed_pub_;

    sensor_msgs::JointState latest_state_msg_;
};

#endif // CLIFFORD_HW_INTERFACE_HPP