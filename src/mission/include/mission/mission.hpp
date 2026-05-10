#ifndef MISSION_HPP
#define MISSION_HPP 

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "mission_msgs/action/arm.hpp"
#include "mission_msgs/action/offboard.hpp"
#include "mission_msgs/action/prearm_check.hpp"
#include "mission_msgs/action/set_point.hpp"

class Mission : public rclcpp::Node
{
public :
    Mission();

    // mission handle struct
    template <typename ActionT>
    struct mission_handle_t
    {
        bool is_finish;
        bool is_success;
        rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle;
    };

    // flight missions
    void prearm_check();
    void arm(bool is_arm);
    void offboard();
    void set_point(double x, double y, double z, uint8_t control_type = 0);

    // get mission handles
    const mission_handle_t<mission_msgs::action::PrearmCheck> get_prearm_check_handle() const { return prearm_check_handle_; };
    const mission_handle_t<mission_msgs::action::Arm> get_arm_handle() const { return arm_handle_;};
    const mission_handle_t<mission_msgs::action::Offboard> get_offboard_handle() const {return offboard_handle_;};
    const mission_handle_t<mission_msgs::action::SetPoint> get_set_point_handle() const {return set_point_handle_;};

private :
    //action clients
    rclcpp_action::Client<mission_msgs::action::PrearmCheck>::SharedPtr prearm_check_action_client_;
    rclcpp_action::Client<mission_msgs::action::Arm>::SharedPtr arm_action_client_;
    rclcpp_action::Client<mission_msgs::action::Offboard>::SharedPtr offboard_action_client_;
    rclcpp_action::Client<mission_msgs::action::SetPoint>::SharedPtr set_point_action_client_;

    //mission handles
    mission_handle_t<mission_msgs::action::PrearmCheck> prearm_check_handle_;
    mission_handle_t<mission_msgs::action::Arm> arm_handle_;
    mission_handle_t<mission_msgs::action::Offboard> offboard_handle_;
    mission_handle_t<mission_msgs::action::SetPoint> set_point_handle_;

};

#endif // MISSION_HPP