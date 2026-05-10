#include "px4_command/px4_command.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PX4Command> node_ = std::make_shared<PX4Command>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}