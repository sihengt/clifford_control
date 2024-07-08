#include "clifford_control/clifford_hw_interface.hpp"

// TODO: what is control_period_ for?
CliffordHWInterface::CliffordHWInterface() : nh_("~"), control_period_(ros::Duration(0.01))
{
    // JointStateHandles for car joints
    hardware_interface::JointStateHandle state_handle_front_steering("front_steering_joint", &pos_[0], &vel_[0], &eff_[0]);
    joint_state_interface_.registerHandle(state_handle_front_steering);

    hardware_interface::JointStateHandle state_handle_rear_steering("rear_steering_joint", &pos_[1], &vel_[1], &eff_[1]);
    joint_state_interface_.registerHandle(state_handle_rear_steering);

    hardware_interface::JointStateHandle state_handle_front_wheel("front_wheel_joint", &pos_[2], &vel_[2], &eff_[2]);
    joint_state_interface_.registerHandle(state_handle_rear_steering);

    hardware_interface::JointStateHandle state_handle_rear_wheel("rear_wheel_joint", &pos_[3], &vel_[3], &eff_[3]);
    joint_state_interface_.registerHandle(state_handle_rear_steering);

    registerInterface(&joint_state_interface_);

    hardware_interface::JointHandle pos_handle_front_steering(joint_state_interface_.getHandle("front_steering_joint"), &cmd_pos_[0]);
    position_joint_interface_.registerHandle(pos_handle_front_steering);
    
    hardware_interface::JointHandle pos_handle_rear_steering(joint_state_interface_.getHandle("rear_steering_joint"), &cmd_pos_[1]);
    position_joint_interface_.registerHandle(pos_handle_rear_steering);

    registerInterface(&position_joint_interface_);

    hardware_interface::JointHandle vel_handle_front_wheel(joint_state_interface_.getHandle("front_wheel_joint"), &cmd_vel_[0]);
    velocity_joint_interface_.registerHandle(vel_handle_front_wheel);
    
    hardware_interface::JointHandle vel_handle_rear_wheel(joint_state_interface_.getHandle("rear_wheel_joint"), &cmd_vel_[1]);
    velocity_joint_interface_.registerHandle(vel_handle_rear_wheel);
    
    registerInterface(&velocity_joint_interface_);

    std::string steering_topic;
    std::string wheel_speed_topic;
    bool success = true;

    success &= nh_.getParam("steering_topic", steering_topic);
    success &= nh_.getParam("wheel_speed_topic", wheel_speed_topic);
    
    if (success)
    {
        steering_sub_ = nh_.subscribe(steering_topic, 10, &CliffordHwInterface::steeringCallback, this);
        wheel_speed_sub_ = nh_.subscribe(wheel_speed_topic, 10, &CliffordHwInterface::wheelSpeedCallback, this);
    }
    else
    {
        ROS_ERROR("steering_topic or wheel_speed_topic not found in rosparam server!");
    }

    last_time_ = ros::Time::now();
}

CliffordHwInterface::read()
{
    if (!latest_steering_msg_.name.empty())
    {
        pos_[0] = latest_steering_msg.position[0];
        vel_[0] = latest_steering_msg.velocity[0];
        pos_[1] = latest_steering_msg.position[1];
        pos_[1] = latest_steering_msg.velocity[1];
    }

    if (!latest_steering_msg_.name.empty())
    {
        pos_[2] = latest_wheel_speed_msg_.position[0];
        vel_[2] = latest_wheel_speed_msg_.velocity[0];
        pos_[3] = latest_wheel_speed_msg_.position[1];
        vel_[3] = latest_wheel_speed_msg_.velocity[1];        
    }
}

CliffordHwInterface::write()
{
    // Todo: think about the kind of publishers you need to make this happen.
}

CliffordHwInterface::steeringCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    latest_steering_msg_ = *msg;
}
CliffordHwInterface::wheelSpeedCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    latest_wheel_speed_msg_ = *msg;
}
