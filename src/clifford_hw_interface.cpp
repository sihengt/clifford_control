#include "clifford_control/clifford_hw_interface.hpp"
#include "clifford_hw_interface.hpp"

CliffordHWInterface::CliffordHWInterface() : nh_("~")
{
    // For setting / getting states of both steerings and wheels.
    hardware_interface::JointStateHandle state_handle_front_steer(
        "front_steer", &pos_[F_STEER], &vel_[F_STEER], &eff_[F_STEER]);
    joint_state_interface_.registerHandle(state_handle_front_steer);
    
    hardware_interface::JointStateHandle state_handle_rear_steer(
        "rear_steer", &pos_[R_STEER], &vel_[R_STEER], &eff_[R_STEER]);
    joint_state_interface_.registerHandle(state_handle_rear_steer);
    
    hardware_interface::JointStateHandle state_handle_front_wheel(
        "front_wheel", &pos_[F_WHEEL], &vel_[F_WHEEL], &eff_[F_WHEEL]);
    joint_state_interface_.registerHandle(state_handle_front_wheel);
    
    hardware_interface::JointStateHandle state_handle_rear_wheel(
        "rear_wheel", &pos_[R_WHEEL], &vel_[R_WHEEL], &eff_[R_WHEEL]);
    joint_state_interface_.registerHandle(state_handle_rear_wheel);
    
    registerInterface(&joint_state_interface_);

    // For controlling steering position
    hardware_interface::JointHandle pos_handle_front_steer(
        joint_state_interface_.getHandle("front_steer"), &cmd_pos_[F_STEER]);
    position_joint_interface_.registerHandle(pos_handle_front_steer);
    
    hardware_interface::JointHandle pos_handle_rear_steer(
        joint_state_interface_.getHandle("rear_steer"), &cmd_pos_[R_STEER]);
    position_joint_interface_.registerHandle(pos_handle_rear_steer);
    registerInterface(&position_joint_interface_);

    // For controlling the velocity of the front / rear wheels.
    hardware_interface::JointHandle vel_handle_front_wheel(
        joint_state_interface_.getHandle("front_wheel"), &cmd_vel_[F_WHEEL]);
    velocity_joint_interface_.registerHandle(vel_handle_front_wheel);
    
    hardware_interface::JointHandle vel_handle_rear_wheel(
        joint_state_interface_.getHandle("rear_wheel"), &cmd_vel_[R_WHEEL]);
    velocity_joint_interface_.registerHandle(vel_handle_rear_wheel);
    registerInterface(&velocity_joint_interface_);

    std::string state_topic;
    if (nh_.getParam("state_topic", state_topic);)
        steering_sub_ = nh_.subscribe(state_topic, 10, &CliffordHwInterface::stateCallback, this);
    else
        ROS_ERROR("state_topic not found in rosparam server!");

    command_pub_ = nh_.advertise<sensor_msgs::JointState>("/motor_controller/desired_joint_state")
}

void CliffordHwInterface::read()
{
    if (!latest_state_msg_.name.empty())
    {
        pos_[F_STEER] = latest_state_msg_.position[F_STEER];
        pos_[R_STEER] = latest_state_msg_.position[R_STEER];
        pos_[F_WHEEL] = latest_state_msg_.position[F_WHEEL];
        pos_[R_WHEEL] = latest_state_msg_.position[R_WHEEL];

        vel_[F_STEER] = latest_state_msg_.velocity[F_STEER];
        vel_[R_STEER] = latest_state_msg_.velocity[R_STEER];
        vel_[F_WHEEL] = latest_state_msg_.velocity[F_WHEEL];
        vel_[R_WHEEL] = latest_state_msg_.velocity[R_WHEEL];
    }
}

void CliffordHwInterface::write()
{
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name = {"front_steer, rear_steer, front_wheel, rear_wheel"};
    msg.position    = {pos_[F_STEER], pos_[R_STEER], pos_[F_WHEEL], pos_[R_WHEEL]};
    msg.velocity    = {vel_[F_STEER], vel_[R_STEER], vel_[F_WHEEL], vel_[R_WHEEL]};
    msg.effort      = {eff_[F_STEER], eff_[R_STEER], eff_[F_WHEEL], eff_[R_WHEEL]};
    
    command_pub_.publish(msg);
}

void CliffordHwInterface::stateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    latest_state_msg_ = *msg;
}
