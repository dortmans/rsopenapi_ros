#include "Robot.hpp"
#include "Service.hpp"

#include "rclcpp/rclcpp.hpp"

class RsBridgeNode : public rclcpp::Node
{
public:
    RsBridgeNode() : Node("rs_bridge")
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&RsBridgeNode::timerCallback, this));
            
            service.start(); 
    }

private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello from RSOPENAPI bridge");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rsopen::Service service;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RsBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
