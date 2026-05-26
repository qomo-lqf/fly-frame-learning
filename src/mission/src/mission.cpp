#include "mission/mission.hpp"

#include <chrono>
#include <future>
#include <thread>

using namespace std::chrono_literals;

Mission::Mission() : Node("mission") 
{
    prearm_check_action_client_ = rclcpp_action::create_client<mission_msgs::action::PrearmCheck>(this, "/mission/prearm");
    arm_action_client_ = rclcpp_action::create_client<mission_msgs::action::Arm>(this, "/mission/arm");
    offboard_action_client_ = rclcpp_action::create_client<mission_msgs::action::Offboard>(this, "/mission/offboard");
    set_point_action_client_ = rclcpp_action::create_client<mission_msgs::action::SetPoint>(this, "/mission/set_point");



    std::future<bool> prearm_future = std::async(std::launch::async, [&]() -> bool { 
            while(!prearm_check_action_client_->wait_for_action_server(0.5s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the prearm action server.");
                    rclcpp::shutdown();
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "prearm action server not available, waiting again... ");
            }
            return true;
    });

    std::future<bool> arm_future = std::async(std::launch::async, [&]() -> bool { 
            while(!arm_action_client_->wait_for_action_server(0.5s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the arm action server.");
                    rclcpp::shutdown();
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "arm action server not available, waiting again... ");
            }
            return true;
    });

    std::future<bool> offboard_future = std::async(std::launch::async, [&]() -> bool { 
            while(!offboard_action_client_->wait_for_action_server(0.5s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the offboard action server.");
                    rclcpp::shutdown();
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "offboard action server not available, waiting again... ");
            }
            return true;
    });

    std::future<bool> set_point_future = std::async(std::launch::async, [&]() -> bool { 
            while(!set_point_action_client_->wait_for_action_server(0.5s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_point action server.");
                    rclcpp::shutdown();
                    return false;
                }
                RCLCPP_INFO(this->get_logger(), "set_point action server not available, waiting again... ");
            }
            return true;
    });

    prearm_future.wait();
    arm_future.wait();
    offboard_future.wait();
    set_point_future.wait();

    if(!prearm_future.get() || !arm_future.get() || !offboard_future.get() || !set_point_future.get())
    {
        rclcpp::shutdown();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Mission ready.");
    }

}

void Mission::prearm_check()
{
    prearm_check_handle_.is_finish = false;
    prearm_check_handle_.is_success = false;
    mission_msgs::action::PrearmCheck::Goal goal{};
    auto send_goal_options = rclcpp_action::Client<mission_msgs::action::PrearmCheck>::SendGoalOptions();
    send_goal_options.goal_response_callback = [&](rclcpp_action::ClientGoalHandle<mission_msgs::action::PrearmCheck>::SharedPtr goal_handle) -> void {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Prearm check failed.");
            prearm_check_handle_.is_success = false;
            prearm_check_handle_.is_finish = true;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Prearm check successful.");
        }
    };
    
    send_goal_options.feedback_callback = 
        [&](rclcpp_action::ClientGoalHandle<mission_msgs::action::PrearmCheck>::SharedPtr,
            const std::shared_ptr<const mission_msgs::action::PrearmCheck::Feedback> feedback) -> void {
                RCLCPP_INFO(this->get_logger(), "Prearm check feedback: %d", feedback->step);
            };

    send_goal_options.result_callback = [&](const rclcpp_action::ClientGoalHandle<mission_msgs::action::PrearmCheck>::WrappedResult &result) -> void {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "prearm check succeeded.");
                prearm_check_handle_.is_success = true;
                break;
            case rclcpp_action::ResultCode::CANCELED:
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Prearm check aborted or canceled, reason: %s" , result.result->ack.message.c_str());
                prearm_check_handle_.is_success = false;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                prearm_check_handle_.is_success = false;
        }
        prearm_check_handle_.is_finish = true;
    };

    auto goal_handle_future = prearm_check_action_client_->async_send_goal(goal, send_goal_options);
    // prearm_check_handle_.goal_handle = goal_handle_future.get();
}

void Mission::arm(bool is_arm)
{
    arm_handle_.is_finish = false;
    arm_handle_.is_success = false;
    mission_msgs::action::Arm::Goal goal;
    goal.arm = is_arm;
    auto send_goal_options = rclcpp_action::Client<mission_msgs::action::Arm>::SendGoalOptions();
    send_goal_options.goal_response_callback = [&](rclcpp_action::ClientGoalHandle<mission_msgs::action::Arm>::SharedPtr goal_handle) -> void {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Arm failed.");
            arm_handle_.is_success = false;
            arm_handle_.is_finish = true;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Arm successful.");
        }
    };

    send_goal_options.feedback_callback = 
        [&](rclcpp_action::ClientGoalHandle<mission_msgs::action::Arm>::SharedPtr,
            const std::shared_ptr<const mission_msgs::action::Arm::Feedback> feedback) -> void {
                RCLCPP_INFO(this->get_logger(), "Arm feedback: %d", feedback->step);
            };

    send_goal_options.result_callback = [&](const rclcpp_action::ClientGoalHandle<mission_msgs::action::Arm>::WrappedResult &result) -> void { 
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Arm succeeded.");
                arm_handle_.is_success = true;
                break;
            case rclcpp_action::ResultCode::CANCELED:
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Arm aborted or canceled, reason: %s" , result.result->ack.message.c_str());
                arm_handle_.is_success = false;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                arm_handle_.is_success = false;
        }
        arm_handle_.is_finish = true;
    };

    auto goal_handle_future = arm_action_client_->async_send_goal(goal, send_goal_options);
    //arm_handle_.goal_handle = goal_handle_future.get();
}

void Mission::offboard()
{
    offboard_handle_.is_finish = false;
    offboard_handle_.is_success = false;
    mission_msgs::action::Offboard::Goal goal;
    goal.mode = mission_msgs::action::Offboard::Goal::MODE_CUSTOM_MODE_ENABLED;
    goal.custom_mode = mission_msgs::action::Offboard::Goal::CUSTOM_MODE_OFFBOARD;
    auto send_goal_options = rclcpp_action::Client<mission_msgs::action::Offboard>::SendGoalOptions();
    send_goal_options.goal_response_callback = [&](rclcpp_action::ClientGoalHandle<mission_msgs::action::Offboard>::SharedPtr goal_handle) -> void {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Offboard failed.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Offboard successful.");
        }
    };

    send_goal_options.feedback_callback = 
        [&](rclcpp_action::ClientGoalHandle<mission_msgs::action::Offboard>::SharedPtr,
            const std::shared_ptr<const mission_msgs::action::Offboard::Feedback> feedback) -> void {};

    send_goal_options.result_callback = [&](const rclcpp_action::ClientGoalHandle<mission_msgs::action::Offboard>::WrappedResult &result) -> void { 
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Offboard succeeded.");
                offboard_handle_.is_success = true;
                break;
            case rclcpp_action::ResultCode::CANCELED:
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Offboard aborted or canceled, reason: %s" , result.result->ack.message.c_str());
                offboard_handle_.is_success = false;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                offboard_handle_.is_success = false;
        }
        offboard_handle_.is_finish = true;
    };

    auto goal_handle_future = offboard_action_client_->async_send_goal(goal, send_goal_options);
    //offboard_handle_.goal_handle = goal_handle_future.get();
}

void Mission::set_point(double x, double y, double z, uint8_t control_type)
{
    set_point_handle_.is_finish = false;
    set_point_handle_.is_success = false;
    mission_msgs::action::SetPoint::Goal goal;
    goal.control_type = control_type;
    goal.value.x = x;
    goal.value.y = y;
    goal.value.z = z;

    auto send_goal_options = rclcpp_action::Client<mission_msgs::action::SetPoint>::SendGoalOptions();
    send_goal_options.goal_response_callback = [&](rclcpp_action::ClientGoalHandle<mission_msgs::action::SetPoint>::SharedPtr goal_handle) -> void {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Set point failed.");
            set_point_handle_.is_success = false;
            set_point_handle_.is_finish = true;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Set point successful.");
        }
    };

    send_goal_options.feedback_callback = 
        [&](rclcpp_action::ClientGoalHandle<mission_msgs::action::SetPoint>::SharedPtr,
            const std::shared_ptr<const mission_msgs::action::SetPoint::Feedback> feedback) -> void {
                RCLCPP_INFO(this->get_logger(), "Set point feedback: \n Position (ref : start postion) : (%f, %f, %f) , Velocity : (%f, %f, %f)",
                    feedback->position.x, feedback->position.y, feedback->position.z, feedback->velocity.x, feedback->velocity.y, feedback->velocity.z);
            };

    send_goal_options.result_callback = [&](const rclcpp_action::ClientGoalHandle<mission_msgs::action::SetPoint>::WrappedResult &result) -> void {
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Set point succeeded.");
                set_point_handle_.is_success = true;
                break;
            case rclcpp_action::ResultCode::CANCELED:
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Set point aborted or canceled, reason: %s" , result.result->ack.message.c_str());
                set_point_handle_.is_success = false;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                set_point_handle_.is_success = false;
        }
        set_point_handle_.is_finish = true;
    };

    auto goal_handle_future = set_point_action_client_->async_send_goal(goal, send_goal_options);
    //set_point_handle_.goal_handle = goal_handle_future.get();
}