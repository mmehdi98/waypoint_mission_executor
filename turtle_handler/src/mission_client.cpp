#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "my_robot_interfaces/action/move_along_path.hpp"

using MoveAlongPath = my_robot_interfaces::action::MoveAlongPath;

class MissionClient : public rclcpp::Node
{
public:
    MissionClient() : Node("mission_client"){

    }

    
private:

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}