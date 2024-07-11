#include "ros/ros.h"
#include "clifford_control/clifford_hw_interface.hpp"

class CliffordControlNode
{
public:
    CliffordControlNode(std::shared_ptr<hardware_interface::RobotHW> hardware_interface);

    // Blocking code that runs the control loop.
    void run();

private:
    void update();
    ros::NodeHandle nh_;

    std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    std::shared_ptr<hardware_interface::RobotHW> hardware_interface_;

    ros::Time last_time_;
    double control_loop_hz_;
};