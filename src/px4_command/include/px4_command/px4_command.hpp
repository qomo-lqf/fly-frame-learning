#ifndef PX4_COMMAND_HPP
#define PX4_COMMAND_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "mission_msgs/action/prearm_check.hpp"
#include "mission_msgs/action/arm.hpp"
#include "mission_msgs/action/offboard.hpp"
#include "px4_msgs/srv/vehicle_command.hpp"

class PX4Command : public rclcpp::Node
{
public:
    PX4Command();

private:
    // service client
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

    //action server
    rclcpp_action::Server<mission_msgs::action::PrearmCheck>::SharedPtr prearm_check_action_server_;
    rclcpp_action::Server<mission_msgs::action::Arm>::SharedPtr arm_action_server_;
    rclcpp_action::Server<mission_msgs::action::Offboard>::SharedPtr offboard_action_server_;

    //action callback
    rclcpp_action::GoalResponse handle_prearm_check_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const mission_msgs::action::PrearmCheck::Goal> goal);
    rclcpp_action::CancelResponse handle_prearm_check_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::PrearmCheck>> goal_handle);
    void handle_prearm_check_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::PrearmCheck>> goal_handle);

    rclcpp_action::GoalResponse handle_arm_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const mission_msgs::action::Arm::Goal> goal);
    rclcpp_action::CancelResponse handle_arm_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Arm>> goal_handle);
    void handle_arm_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Arm>> goal_handle);

    rclcpp_action::GoalResponse handle_offboard_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const mission_msgs::action::Offboard::Goal> goal);
    rclcpp_action::CancelResponse handle_offboard_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Offboard>> goal_handle);
    void handle_offboard_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Offboard>> goal_handle);

    //action executors
    void prearm_check_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::PrearmCheck>> goal_handle);
    void arm_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Arm>> goal_handle);
    void offboard_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Offboard>> goal_handle);

    // //command combination
    // void arm_command();
    // void offboard_command();
};

#endif // PX4_COMMAND_HPP