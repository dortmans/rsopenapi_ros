#include <unistd.h>
#include "Robot.hpp"
#include "Service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rs_bridge_msgs/msg/world_model.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
            1000ms, //same as std::chrono::milliseconds(1000),
            std::bind(&RsBridgeNode::timer_callback, this));


        // Get transforms from/to MSL coordinate system
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
/*
    geometry_msgs::msg::TransformStamped tf_lookup(const std::string from_frame, const std::string to_frame)
    {
        try {
          geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            to_frame, from_frame,
            tf2::TimePointZero, 
						tf2::durationFromSec(1.0));
        } catch (const tf2::TransformException & e) {
            RCLCPP_DEBUG(
            get_logger(),
            "Failed to get transform: (%s)", e.what());
        }
        return transform;    
    }
*/
    template<typename T>
		bool tf_transform(
				const T input_pose,
				T transformed_pose,
				const std::string target_frame)
    {
        try
        {
					transformed_pose = tf_buffer_->transform(
      			input_pose, target_frame,
      			tf2::durationFromSec(1.0));
    			return true;
        } catch (const tf2::TransformException & e) {
            RCLCPP_DEBUG(
            get_logger(),
            "Failed to transform: (%s)", e.what());
        }
				return false;
    }


    void timer_callback()
    {
        if (robot_->read(data_))
        {
            // successful read

            RCLCPP_INFO(this->get_logger(), "Publish Odometry and WorldModel");

            rclcpp::Time now = this->get_clock()->now();

            // Get current robot pose
            geometry_msgs::msg::PoseStamped msl_pose, ros_pose;
						msl_pose.header.stamp = now; // TODO: use ts
            msl_pose.header.frame_id = "map_msl";
            msl_pose.pose.position.x = data_.self.pose.x;
            msl_pose.pose.position.y = data_.self.pose.y;
            msl_pose.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, data_.self.pose.rz);
						msl_pose.pose.orientation = tf2::toMsg(q);

            // Transform pose from MSL to ROS coordinate frame
            tf_transform<geometry_msgs::msg::PoseStamped>(msl_pose, ros_pose, "map");

						// Get current robot velocity
						geometry_msgs::msg::Vector3Stamped msl_velocity, ros_velocity;
						msl_velocity.header.stamp = now; // TODO: use ts
            msl_velocity.header.frame_id = "base_link_msl";
						msl_velocity.vector.x = data_.self.vel.x;
            msl_velocity.vector.y = data_.self.vel.y;
            msl_velocity.vector.z = 0.0;
						
						// Transform velocity from MSL to ROS coordinate frame
						tf_transform<geometry_msgs::msg::Vector3Stamped>(msl_velocity, ros_velocity, "base_link");
					
            // Publish odometry message
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = ros_pose.header.stamp;
            odom.header.frame_id = "odom";
						odom.pose.pose = ros_pose.pose;
            odom.child_frame_id = "base_link";
						odom.twist.twist.linear = ros_velocity.vector;
            odom_publisher_->publish(odom);  

            if (tf_broadcast_) {
                // Broadcast odom transform
                geometry_msgs::msg::TransformStamped odom_tf;
                odom_tf.header.stamp = ros_pose.header.stamp;
                odom_tf.header.frame_id = "odom";
                odom_tf.child_frame_id = "base_link";
								odom_tf.transform.translation.x = ros_pose.pose.position.x;
								odom_tf.transform.translation.y = ros_pose.pose.position.y;
								odom_tf.transform.translation.y = ros_pose.pose.position.z;
                odom_tf.transform.rotation = ros_pose.pose.orientation;
                odom_broadcaster_->sendTransform(odom_tf);
            }
        }
        else
        {
            // unsuccessful read
						RCLCPP_INFO(this->get_logger(), "No data");
        }
       
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Twist"); 

				robot_->writeVelocity(0, 0, 0);       
    }

    rclcpp::Publisher<rs_bridge_msgs::msg::WorldModel>::SharedPtr wm_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
 
    bool tf_broadcast_;
    
    std::shared_ptr<rsopen::Service> service_;
    std::shared_ptr<rsopen::Robot> robot_;
    rsopen::interrobot_t data_;   
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
   
    auto node = std::make_shared<RsBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
