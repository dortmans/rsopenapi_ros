#include <unistd.h>
#include "Robot.hpp"
#include "Service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rs_bridge_msgs/msg/world_model.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class RsBridgeNode : public rclcpp::Node
{
public:
    RsBridgeNode() : Node("rs_bridge")
    {

        this->declare_parameter("robot", 0);
        this->declare_parameter("hash", "");
        this->declare_parameter("tf", false);

        auto robot_id = this->get_parameter("robot").as_int();
        auto hash_str = this->get_parameter("hash").as_string();
        auto tf_broadcast_ = get_parameter("tf").as_bool();

        RCLCPP_INFO(this->get_logger(), "Bridge for robot: %ld with hash: %s",
                    robot_id,
                    hash_str.c_str()
        );

        // Extract hash
        uint64_t hash = string2uint(hash_str);
        
         // Start communication service
        service_ = std::make_shared<rsopen::Service>();
        service_->start();

        // Create proxy to robot
        robot_ = std::make_shared<rsopen::Robot>(robot_id, hash);

        // Create ROS broadcasters, publishers, subscribers, actions
        odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        wm_publisher_ = this->create_publisher<rs_bridge_msgs::msg::WorldModel>("worldmodel", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                            std::bind(&RsBridgeNode::twist_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            1000ms, //std::chrono::milliseconds(1000),
            std::bind(&RsBridgeNode::timer_callback, this));
    }

private:

    uint64_t string2uint(const std::string& s)
    {
        if (s.find("0x") == 0)
        {
            uint64_t result;
            std::stringstream ss;
            ss << std::hex << s;
            ss >> result;
            return result;
        }
        return std::stoull(s);
    }

    void timer_callback()
    {
        if (robot_->read(data_))
        {
            // successful read

            std::cout << "Robot at (" << data_.self.pose.x << ", " << data_.self.pose.y << ", " << data_.self.pose.rz << ")" 
                      << "; Control ball: " << data_.player_status.control_ball << std::endl;

            RCLCPP_INFO(this->get_logger(), "Publish Odometry and WorldModel");

            //!!!!!!!!!!!
            // TODO: rotate 90 degrees from Robocup frame to ROS frame
            //!!!!!!!!!!!

            tf2::Quaternion q;
            q.setRPY(0,0,data_.self.pose.rz);

            // Publish odometry message

            nav_msgs::msg::Odometry odom;
            odom.header.stamp = this->get_clock()->now(); // TODO: use ts
            odom.header.frame_id = "odom";

            //set the position
            odom.pose.pose.position.x = data_.self.pose.x;
            odom.pose.pose.position.y = data_.self.pose.y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation.x = q.getX();
            odom.pose.pose.orientation.y = q.getY();
            odom.pose.pose.orientation.z = q.getZ();
            odom.pose.pose.orientation.w = q.getW();

            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = data_.self.vel.x;
            odom.twist.twist.linear.y = data_.self.vel.y;
            odom.twist.twist.angular.z = 0.0;

            //publish the message
            odom_publisher_->publish(odom);  

            if (tf_broadcast_) {
                    
                // Broadcast odom transform

                geometry_msgs::msg::TransformStamped odom_trans;
                odom_trans.header.stamp = this->get_clock()->now(); // TODO: use ts
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link";

                odom_trans.transform.translation.x = data_.self.pose.x;
                odom_trans.transform.translation.y = data_.self.pose.y;
                odom_trans.transform.rotation.x = q.getX();
                odom_trans.transform.rotation.y = q.getY();
                odom_trans.transform.rotation.z = q.getZ();
                odom_trans.transform.rotation.w = q.getW();
                odom_broadcaster_->sendTransform(odom_trans);
            }

        }
        else
        {
            // unsuccessful read
            std::cout << "No data" << std::endl;
        }

        robot_->writeVelocity(0, 0, 0);        
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Twist");       
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    rclcpp::Publisher<rs_bridge_msgs::msg::WorldModel>::SharedPtr wm_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<rsopen::Service> service_;
    std::shared_ptr<rsopen::Robot> robot_;
    rsopen::interrobot_t data_;

    bool tf_broadcast_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
   
    auto node = std::make_shared<RsBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
