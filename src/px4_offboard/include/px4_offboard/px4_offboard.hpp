#ifndef PX4_OFFBOARD_HPP
#define PX4_OFFBOARD_HPP 

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "mission_msgs/action/set_point.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

class PX4Offboard : public rclcpp::Node
{
public :
    PX4Offboard();

private:
    // subscriber
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;

    // publisher
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

    // action server
    rclcpp_action::Server<mission_msgs::action::SetPoint>::SharedPtr set_point_action_server_;

    // subscriber callback & values
    float x_;
    float y_;
    float z_;
    float yaw_;
    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

    // timer & timer callback
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    void heartbeat_timer_callback();

    // action server callback
    rclcpp_action::GoalResponse set_point_goal_callback(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const mission_msgs::action::SetPoint::Goal> goal);
    rclcpp_action::CancelResponse set_point_cancel_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::SetPoint>> goal_handle);
    void set_point_accepted_callback(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::SetPoint>> goal_handle);

    // action executor
    void set_point_execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::SetPoint>> goal_handle);

    // frame transform 
    std::array<float,3> frame_transform(float x, float y, float z, float yaw);
};


#endif // PX4_OFFBOARD_HPP