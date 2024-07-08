#ifndef CLIFFORD_HW_INTERFACE_HPP
#define CLIFFORD_HW_INTERFACE_HPP

#include "ros/ros.h"
#include "hardware_interface/robot_hw.h"
#include "hardware_interface/joint_command_interface.h"

class CliffordHWInterface : public hardware_interface::RobotHW
{
public:
    CliffordHWInterface();

    bool init();
    void read();
    void write();
private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    
    void steeringCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void wheelSpeedCallback(const sensor_msgs::JointState::ConstPtr& msg);

    double cmd_pos_[2]; // Front and rear steering
    double cmd_vel_[2]; // Front and rear motors
    
    double pos_[4]; // Position: front steering, rear steering, front wheel, rear wheel
    double vel_[4]; // Velocities: front steering, rear steering, front wheel, rear wheel

    ros::NodeHandle nh_;
    ros::Duration control_period_;
    ros::Time last_time_;

    ros::Subscriber steering_sub_;
    ros::Subscriber wheel_speed_sub_;

    sensor_msgs::JointState latest_steering_msg_;
    sensor_msgs::JointState latest_wheel_speed_msg_;
};

#endif // CLIFFORD_HW_INTERFACE_HPP