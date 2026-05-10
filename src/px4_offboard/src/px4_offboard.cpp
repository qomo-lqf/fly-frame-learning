#include "px4_offboard/px4_offboard.hpp"
#include <thread>

PX4Offboard::PX4Offboard() : Node("px4_offboard"),x_(0),y_(0),z_(0),yaw_(0),control_type_(CONTROL_MODE::CTRL_MODE_POSITION)
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

    set_point_action_server_ = rclcpp_action::create_server<mission_msgs::action::SetPoint>(
        this,
        "setpoint",
        std::bind(&PX4Offboard::set_point_goal_callback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PX4Offboard::set_point_cancel_callback, this, std::placeholders::_1),
        std::bind(&PX4Offboard::set_point_accepted_callback, this, std::placeholders::_1)
    );
}

void PX4Offboard::local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    x_ = msg->x; 
    y_ = msg->y;
    z_ = msg->z;
    vx_ = msg->vx;
    vy_ = msg->vy;
    vz_ = msg->vz;
    yaw_ = msg->heading;
}

void PX4Offboard::heartbeat_timer_callback()
{
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    switch(control_type_)
    {
        case CONTROL_MODE::CTRL_MODE_POSITION:
            msg.position = true;
            msg.velocity = false;
            msg.acceleration = false;
            break;
        case CONTROL_MODE::CTRL_MODE_VELOCITY:
            msg.position = false;
            msg.velocity = true;
            msg.acceleration = false;
            break;
        case CONTROL_MODE::CTRL_MODE_ACCELERATION:
            msg.position = false;
            msg.velocity = false;
            msg.acceleration = true;
            break;
    }
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_control_mode_pub_->publish(msg);
}

rclcpp_action::GoalResponse PX4Offboard::set_point_goal_callback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const mission_msgs::action::SetPoint::Goal> goal)
{
    RCLCPP_INFO(get_logger(), "Received goal request");
    (void)uuid;
    switch(goal->control_type)
    {
        case CONTROL_MODE::CTRL_MODE_POSITION :
            control_type_ = CONTROL_MODE::CTRL_MODE_POSITION;
            RCLCPP_INFO(get_logger(), "Control mode: Position");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            break;
        case CONTROL_MODE::CTRL_MODE_VELOCITY : 
            control_type_ = CONTROL_MODE::CTRL_MODE_VELOCITY;
            RCLCPP_INFO(get_logger(), "Control mode: Velocity");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            break;
        case CONTROL_MODE::CTRL_MODE_ACCELERATION :
            control_type_ = CONTROL_MODE::CTRL_MODE_ACCELERATION;
            RCLCPP_INFO(get_logger(), "Control mode: Acceleration");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Invalid control mode");
            return rclcpp_action::GoalResponse::REJECT;
    }
}

rclcpp_action::CancelResponse PX4Offboard::set_point_cancel_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::SetPoint>> goal_handle)
{
    RCLCPP_INFO(get_logger(), "Received request to cancel goal ");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PX4Offboard::set_point_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::SetPoint>> goal_handle)
{
    std::thread{std::bind(&PX4Offboard::set_point_execute, this, std::placeholders::_1),goal_handle}.detach();
}

void PX4Offboard::set_point_execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::SetPoint>> goal_handle)
{
    auto goal = goal_handle->get_goal();
    RCLCPP_INFO(get_logger(), "Executing goal : \n mode : %d \n x : %f \n y : %f \n z : %f \n current yaw : %f",goal->control_type, goal->value.x ,goal->value.y, goal->value.z, yaw_ );

    switch(control_type_)
    {
        case CTRL_MODE_POSITION:
        {
            px4_msgs::msg::TrajectorySetpoint set_point_msg_;
            set_point_msg_.timestamp = get_clock()->now().nanoseconds() / 1000;
            set_point_msg_.position = frame_transform(goal->value.x, goal->value.y, goal->value.z, yaw_);
            set_point_msg_.velocity = {0,0,0};
            set_point_msg_.acceleration = {0,0,0};
            set_point_msg_.jerk = {0,0,0};
            set_point_msg_.yaw = yaw_;
            set_point_msg_.yawspeed = 0;
            trajectory_setpoint_pub_->publish(set_point_msg_);
            break;
        }
        case CTRL_MODE_VELOCITY:
        {
            px4_msgs::msg::TrajectorySetpoint set_point_msg_;
            set_point_msg_.timestamp = get_clock()->now().nanoseconds() / 1000;
            set_point_msg_.position = {0,0,0};
            set_point_msg_.velocity = frame_transform(goal->value.x, goal->value.y, goal->value.z, yaw_);
            set_point_msg_.acceleration = {0,0,0};
            set_point_msg_.jerk = {0,0,0};
            set_point_msg_.yaw = yaw_;
            set_point_msg_.yawspeed = 0;
            trajectory_setpoint_pub_->publish(set_point_msg_);
            break;
        }
        case CTRL_MODE_ACCELERATION:
        {
            px4_msgs::msg::TrajectorySetpoint set_point_msg_;
            set_point_msg_.timestamp = get_clock()->now().nanoseconds() / 1000;
            set_point_msg_.position = {0,0,0};
            set_point_msg_.velocity = {0,0,0};
            set_point_msg_.acceleration = frame_transform(goal->value.x, goal->value.y, goal->value.z, yaw_);
            set_point_msg_.jerk = {0,0,0};
            set_point_msg_.yaw = yaw_;
            set_point_msg_.yawspeed = 0;
            trajectory_setpoint_pub_->publish(set_point_msg_);
            break;
        }
        default:
        {
            px4_msgs::msg::TrajectorySetpoint set_point_msg_;
            set_point_msg_.timestamp = get_clock()->now().nanoseconds() / 1000;
            set_point_msg_.position = {0,0,0};
            set_point_msg_.velocity = {0,0,0};
            set_point_msg_.acceleration = {0,0,0};
            set_point_msg_.jerk = {0,0,0};
            set_point_msg_.yaw = yaw_;
            set_point_msg_.yawspeed = 0;
            trajectory_setpoint_pub_->publish(set_point_msg_);
            RCLCPP_INFO(get_logger(), "Invalid control mode , holding");
            std::shared_ptr<mission_msgs::action::SetPoint::Result> result = std::make_shared<mission_msgs::action::SetPoint::Result>();
            result->ack.success = false;
            result->ack.message = "invalid control mode";
            result->ack.result_param = 0;
            goal_handle->abort(result);
            break;
        }
    }

    rclcpp::Rate loop_rate(10);
    float origin_x = x_;
    float origin_y = y_;
    float origin_z = z_;
    int timeout_counter = 0;
    while (rclcpp::ok())
    {
        if(goal_handle->is_canceling())
        {
            px4_msgs::msg::TrajectorySetpoint set_point_msg_;
            set_point_msg_.timestamp = get_clock()->now().nanoseconds() / 1000;
            set_point_msg_.position = {0,0,0};
            set_point_msg_.velocity = {0,0,0};
            set_point_msg_.acceleration = {0,0,0};
            set_point_msg_.jerk = {0,0,0};
            set_point_msg_.yaw = yaw_;
            set_point_msg_.yawspeed = 0;
            trajectory_setpoint_pub_->publish(set_point_msg_);
            std::shared_ptr<mission_msgs::action::SetPoint::Result> result = std::make_shared<mission_msgs::action::SetPoint::Result>();
            result->ack.success = true;
            result->ack.message = "set point canceled";
            result->ack.result_param = 0;
            goal_handle->canceled(result);
        }

        if(timeout_counter > SET_POINT_TIMEOUT_TIMES)
        {
            px4_msgs::msg::TrajectorySetpoint set_point_msg_;
            set_point_msg_.timestamp = get_clock()->now().nanoseconds() / 1000;
            set_point_msg_.position = {0,0,0};
            set_point_msg_.velocity = {0,0,0};
            set_point_msg_.acceleration = {0,0,0};
            set_point_msg_.jerk = {0,0,0};
            set_point_msg_.yaw = yaw_;
            set_point_msg_.yawspeed = 0;
            trajectory_setpoint_pub_->publish(set_point_msg_);
            std::shared_ptr<mission_msgs::action::SetPoint::Result> result = std::make_shared<mission_msgs::action::SetPoint::Result>();
            result->ack.success = false;
            result->ack.message = "set point timeout";
            result->ack.result_param = 0;
            goal_handle->abort(result);
        }

        if(abs(x_ - origin_x) < SET_POINT_EPSILON_M && abs(y_ - origin_y) < SET_POINT_EPSILON_M && abs(z_ - origin_z) < SET_POINT_EPSILON_M)
        {
            std::shared_ptr<mission_msgs::action::SetPoint::Result> result = std::make_shared<mission_msgs::action::SetPoint::Result>();
            result->ack.success = true;
            result->ack.message = "set point reached";
            result->ack.result_param = 0;
            goal_handle->succeed(result);
        }

        std::shared_ptr<mission_msgs::action::SetPoint::Feedback> feedback = std::make_shared<mission_msgs::action::SetPoint::Feedback>();
        feedback->position.x = x_;
        feedback->position.y = y_;
        feedback->position.z = z_;
        feedback->yaw = yaw_;
        feedback->velocity.x = vx_;
        feedback->velocity.y = vy_;
        feedback->velocity.z = vz_;
        goal_handle->publish_feedback(feedback);

    }
}

inline std::array<float, 3> PX4Offboard::frame_transform(float x, float y, float z, float yaw)
{
    return {x * cos(yaw) - y * sin(yaw), x * sin(yaw) + y * cos(yaw), z}; 
}