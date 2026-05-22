#include "mission/mission.hpp"

#include <vector>
#include <chrono>

using namespace std::chrono_literals;

enum State
{
    INIT,
    PREARM,
    ARM,
    OFFBOARD,
    WORK,
    DISARM,
    EXIT
};

struct MissionPoint
{
    float x;
    float y;
    float z;
};

std::vector<MissionPoint> mission_points = {
    {0.0, 0.0, 3.0},
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, -3.0}
};

int main(int argc , char** argv)
{
    State state = INIT;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mission>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok())
    {
        switch(state)
        {
            case INIT:
                node -> prearm_check();
                state = PREARM;
                break;
            case PREARM:
                if(node->get_prearm_check_handle().is_finish && node->get_prearm_check_handle().is_success)
                {
                    node -> arm(true);
                    state = ARM;
                }
                break;
            case ARM:
                if(node->get_arm_handle().is_finish && node->get_arm_handle().is_success)
                {
                    node -> offboard();
                    state = OFFBOARD;
                }
                break;
            case OFFBOARD:
                if(node->get_offboard_handle().is_finish && node->get_offboard_handle().is_success)
                {
                    state = WORK;
                }
                break;
            case WORK:
// -----------------设置航点区------------------ 
                static size_t current_point_idx = 0;
                static bool first_wp_sent = false;

                // 进入 WORK 后发送第一个航点
                if(!first_wp_sent && mission_points.size() > 0) {
                    node->set_point(mission_points[0].x, mission_points[0].y, mission_points[0].z);
                    first_wp_sent = true;
                }

                // 当前航点完成后发送下一个
                if(node->get_set_point_handle().is_finish)
                {
                    if(node->get_set_point_handle().is_success)
                    {
                        current_point_idx++;
                        if(current_point_idx < mission_points.size())
                        {
                            node->set_point(mission_points[current_point_idx].x,
                                            mission_points[current_point_idx].y,
                                            mission_points[current_point_idx].z);
                        }
                        else
                        {
                            // All waypoints completed
                            node -> arm(false);
                            state = DISARM;
                            current_point_idx = 0;
                            first_wp_sent = false;
                        }
                    }
                    else
                    {
                        RCLCPP_ERROR(node->get_logger(), "Set point failed at index %zu", current_point_idx);
                        node -> arm(false);
                        state = DISARM;
                        current_point_idx = 0;
                        first_wp_sent = false;
                    }
                }
// ---------------------------------------------
                break;
            case DISARM:
                if(node->get_arm_handle().is_finish && node->get_arm_handle().is_success)
                {
                    state = EXIT;
                }
        }
        executor.spin_once(1s);
        loop_rate.sleep();
    }
}