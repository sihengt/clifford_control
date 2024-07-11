#include "clifford_control/clifford_control_node.hpp"

CliffordControlNode::CliffordControlNode(std::shared_ptr<hardware_interface::RobotHW> hardware_interface)
    : nh_("~"), hardware_interface_(hardware_interface), last_time_(ros::Time::now())
{
    controller_manager_.reset(new controller_manager::ControllerManager(hardware_interface_.get(), nh_));
    nh.param<double>("control_loop_hz", control_loop_hz_, 50.0);
}

void CliffordControlNode::run()
{
    ros::Rate rate(control_loop_hz_);

    while (ros::ok())
    {
        update();
        rate.sleep();
    }
}

void CliffordControlNode::update()
{
    ros::Time current_time = ros::Time::now();
    ros::Duration elapsed_time = current_time - last_time;
    last_time = current_time;

    hardware_interface_->read();
    controller_manager_->update(current_time, elapsed_time); // current_time, change in time since last call to update
    hardware_interface_->write();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "clifford_control_node");
    std::shared_ptr<CliffordHWInterface> clifford_hw = std::make_shared<CliffordHwInterface>();
    CliffordControlNode clifford_control(clifford_hw);
    clifford_control.run();
    return 0;
}