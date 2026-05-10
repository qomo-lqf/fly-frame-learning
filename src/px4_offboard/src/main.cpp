#include "px4_offboard/px4_offboard.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PX4Offboard> node_ = std::make_shared<PX4Offboard>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}