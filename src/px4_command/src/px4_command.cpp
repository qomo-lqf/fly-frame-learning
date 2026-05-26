#include "px4_command/px4_command.hpp"

#define DEBUG

#include <chrono>
#include <thread>
#include <functional>
#include <cmath>

using namespace std::chrono_literals;

// PX4 Command IDs
constexpr uint32_t VEHICLE_CMD_ARM = 400;
constexpr uint32_t VEHICLE_CMD_PREARM_CHECK = 401;
constexpr uint32_t VEHICLE_CMD_SET_MODE = 176;


PX4Command::PX4Command(): Node("px4_command")
{
    // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
    
    while (!vehicle_command_client_->wait_for_service(0.5s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    arm_action_server_ = rclcpp_action::create_server<mission_msgs::action::Arm>(
        this,
        "/mission/arm",
        std::bind(&PX4Command::handle_arm_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PX4Command::handle_arm_cancel, this, std::placeholders::_1),
        std::bind(&PX4Command::handle_arm_accepted, this, std::placeholders::_1)
    );

    prearm_check_action_server_ = rclcpp_action::create_server<mission_msgs::action::PrearmCheck>(
        this,
        "/mission/prearm",
        std::bind(&PX4Command::handle_prearm_check_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PX4Command::handle_prearm_check_cancel, this, std::placeholders::_1),
        std::bind(&PX4Command::handle_prearm_check_accepted, this, std::placeholders::_1)
    );
    
    offboard_action_server_ = rclcpp_action::create_server<mission_msgs::action::Offboard>(
        this,
        "/mission/offboard",
        std::bind(&PX4Command::handle_offboard_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PX4Command::handle_offboard_cancel, this, std::placeholders::_1),
        std::bind(&PX4Command::handle_offboard_accepted, this, std::placeholders::_1)
    );

}

rclcpp_action::GoalResponse PX4Command::handle_prearm_check_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const mission_msgs::action::PrearmCheck::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received prearm goal request ");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PX4Command::handle_prearm_check_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::PrearmCheck>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel prearm goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PX4Command::handle_prearm_check_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::PrearmCheck>> goal_handle)
{
    std::thread{std::bind(&PX4Command::prearm_check_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void PX4Command::prearm_check_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::PrearmCheck>> goal_handle)
{
#ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "prearm check execute");
#endif

    auto feedback = std::make_shared<mission_msgs::action::PrearmCheck::Feedback>();
    
    for(int i = 0; i < 3; i++)
    {
        if (!rclcpp::ok()) {
            goal_handle->abort(std::make_shared<mission_msgs::action::PrearmCheck::Result>());
            return;
        }
        
        std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
        request->request.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        request->request.command = VEHICLE_CMD_PREARM_CHECK;
        request->request.param1 = 0;
        request->request.param2 = 0;
        request->request.param3 = 0;
        request->request.param4 = 0;
        request->request.param5 = 0;
        request->request.param6 = 0;
        request->request.param7 = 0;
        request->request.target_system = 1;
        request->request.target_component = 1;
        request->request.source_system = 1;
        request->request.source_component = 1;
        request->request.from_external = true;

#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "prearm check checkpoint1 ： request发送前");
#endif

        vehicle_command_client_->async_send_request(request, [this, goal_handle](const std::shared_future<std::shared_ptr<px4_msgs::srv::VehicleCommand::Response>> future)
        { 
            const auto & reply = future.get();
            if(reply->reply.result != 0)
            {
                std::shared_ptr<mission_msgs::action::PrearmCheck::Result> result = std::make_shared<mission_msgs::action::PrearmCheck::Result>();
                result->ack.success = false;
                result->ack.result_param = reply->reply.result;
                result->ack.message = "prearm check failed";
                goal_handle->abort(result);
            }

        });

#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "prearm check checkpoint2 ： feedback发送前");
#endif
        feedback->step = i + 1;
        goal_handle->publish_feedback(feedback);
    }
    
#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "prearm check checkpoint3 : result发送前");
#endif

    auto result = std::make_shared<mission_msgs::action::PrearmCheck::Result>();
    if (rclcpp::ok()) {
        result->ack.success = true;
        result->ack.result_param = 0;
        result->ack.message = "prearm check success";
        goal_handle->succeed(result);
    } else {
        result->ack.success = false;
        result->ack.result_param = 1;
        result->ack.message = "prearm check failed";
        goal_handle->abort(result);
    }
#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "prearm check checkpoint4： result发送完毕");
#endif

}

rclcpp_action::GoalResponse PX4Command::handle_arm_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const mission_msgs::action::Arm::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received arm goal request : %s", goal->arm ? "arm" : "disarm" );
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
}

rclcpp_action::CancelResponse PX4Command::handle_arm_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Arm>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel arm goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PX4Command::handle_arm_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Arm>> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&PX4Command::arm_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void PX4Command::arm_execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Arm>> goal_handle)
{
#ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "arm execute");
#endif

     const auto goal = goal_handle->get_goal();
     auto feedback = std::make_shared<mission_msgs::action::Arm::Feedback>();
     
     if(goal->arm)
     {
        for(int i = 0; i < 5; i++)
        {
            if (!rclcpp::ok()) {
                goal_handle->abort(std::make_shared<mission_msgs::action::Arm::Result>());
                return;
            }
            
            std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
            request->request.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            request->request.command = VEHICLE_CMD_ARM;
            request->request.param1 = goal->arm ? 1.0f : 0.0f;
            request->request.param2 = 0;
            request->request.param3 = 0;
            request->request.param4 = 0;
            request->request.param5 = 0;
            request->request.param6 = 0;
            request->request.param7 = 0;
            request->request.target_system = 1;
            request->request.target_component = 1;
            request->request.source_system = 1;
            request->request.source_component = 1;
            request->request.from_external = true;
            
            vehicle_command_client_->async_send_request(request, [this, goal_handle](const std::shared_future<std::shared_ptr<px4_msgs::srv::VehicleCommand::Response>> future)
            {
                const auto & reply = future.get();
                if(reply->reply.result != 0)
                {
                    std::shared_ptr<mission_msgs::action::Arm::Result> result = std::make_shared<mission_msgs::action::Arm::Result>();
                    result->ack.success = false;
                    result->ack.result_param = reply->reply.result;
                    result->ack.message = "arm failed";
                    goal_handle->abort(result);
                }
            });
            feedback->step = i + 1;
            goal_handle->publish_feedback(feedback);
        }
    }
    
    auto result = std::make_shared<mission_msgs::action::Arm::Result>();
    if (rclcpp::ok()) {
        result->ack.success = true;
        result->ack.result_param = 0;
        result->ack.message = goal->arm ? "arm success" : "disarm success";
        goal_handle->succeed(result);
    } else {
        result->ack.success = false;
        result->ack.result_param = 1;
        result->ack.message = goal->arm ? "arm failed" : "disarm failed";
        goal_handle->abort(result);
    }
}

rclcpp_action::GoalResponse PX4Command::handle_offboard_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const mission_msgs::action::Offboard::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal offboard request : %d , %d", goal->mode , goal->custom_mode);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
}

rclcpp_action::CancelResponse PX4Command::handle_offboard_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Offboard>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel offboard goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PX4Command::handle_offboard_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Offboard>> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&PX4Command::offboard_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void PX4Command::offboard_execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mission_msgs::action::Offboard>> goal_handle)
{
#ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "offboard execute");
#endif

    const auto goal = goal_handle->get_goal();
    std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
    request->request.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    request->request.command = VEHICLE_CMD_SET_MODE;
    request->request.param1 = goal->mode;
    request->request.param2 = goal->custom_mode;
    request->request.param3 = 0;
    request->request.param4 = 0;
    request->request.param5 = 0;
    request->request.param6 = 0;
    request->request.param7 = 0;
    request->request.target_system = 1;
    request->request.target_component = 1;
    request->request.source_system = 1;
    request->request.source_component = 1;
    request->request.from_external = true;
    
    vehicle_command_client_->async_send_request(request, [this, goal_handle](const std::shared_future<std::shared_ptr<px4_msgs::srv::VehicleCommand::Response>> future)
    { 
        const auto & reply = future.get();
        if(reply->reply.result != 0)
        {
            std::shared_ptr<mission_msgs::action::Offboard::Result> result = std::make_shared<mission_msgs::action::Offboard::Result>();
            result->ack.success = false;
            result->ack.result_param = reply->reply.result;
            result->ack.message = "offboard mode change failed";
            goal_handle->abort(result);
        }
    });
    
    auto result = std::make_shared<mission_msgs::action::Offboard::Result>();
    if (rclcpp::ok()) {
        result->ack.success = true;
        result->ack.result_param = 0;
        result->ack.message = "offboard mode change success";
        goal_handle->succeed(result);
    } else {
        result->ack.success = false;
        result->ack.result_param = 1;
        result->ack.message = "offboard mode change failed";
        goal_handle->abort(result);
    }
}