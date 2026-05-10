#include "px4_offboard/px4_offboard.hpp"

PX4Offboard::PX4Offboard() : Node("px4_offboard")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "fmu/out/vehicle_local_position", qos, std::bind(&PX4Offboard::local_position_callback, this, std::placeholders::_1));

    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "fmu/in/offboard_control_mode", 10);

    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "fmu/in/trajectory_setpoint", 10);

    heartbeat_timer_ = create_wall_timer(
        std::chrono::milliseconds(250), std::bind(&PX4Offboard::heartbeat_timer_callback, this));

    set_point_action_server_ = rclcpp_action::create_server<px4_msgs::action::Setpoint>(
        this,
        "setpoint",
        std::bind(&PX4Offboard::set_point_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PX4Offboard::set_point_cancel_callback, this, std::placeholders::_1),
        std::bind(&PX4Offboard::set_point_accepted_callback, this, std::placeholders::_1)
    )
}