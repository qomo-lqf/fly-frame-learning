#include "mission/mission.hpp"

#include <vector>
#include <chrono>

using namespace std::chrono_literals;

enum State
{
    INIT,
    PREARM,
    ARM,
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
                    node -> set_point(0,0,0);
                    state = WORK;
                }
                break;
            case WORK:
// -----------------设置航点区------------------ 
                // 先发送第一个航点
                if(mission_points.size() > 0) {
                    auto first_point = mission_points[0];
                    node->set_point(first_point.x, first_point.y, first_point.z);
                }
                
                size_t current_point_idx = 0;
                while(current_point_idx < mission_points.size())
                {
                    if(node->get_set_point_handle().is_finish)
                    {
                        if(node->get_set_point_handle().is_success)
                        {
                            current_point_idx++;
                            if(current_point_idx < mission_points.size())
                            {
                                auto point = mission_points[current_point_idx];
                                node->set_point(point.x, point.y, point.z);
                            }
                        }
                        else
                        {
                            RCLCPP_ERROR(node->get_logger(), "Set point failed at index %zu", current_point_idx);
                            break;
                        }
                    }
                    loop_rate.sleep();
                }
// ---------------------------------------------
                if(node->get_set_point_handle().is_finish && node->get_set_point_handle().is_success)
                {
                    node -> arm(false);
                    state = DISARM;
                }
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