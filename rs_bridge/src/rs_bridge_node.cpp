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
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
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
        tf_broadcast_ = this->get_parameter("tf").as_bool();

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
            25ms, //same as std::chrono::milliseconds(25),
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
    geometry_msgs::msg::TransformStamped tf_lookup(const std::string& from_frame, const std::string& to_frame)
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
				const T& input_pose,
				T& transformed_pose,
				const std::string& target_frame)
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

    geometry_msgs::msg::PoseStamped to_pose(
        const builtin_interfaces::msg::Time& stamp,
        const std::string& frame_id,
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw)
    {
        geometry_msgs::msg::PoseStamped pose;
				pose.header.stamp = stamp;
        pose.header.frame_id = frame_id;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
				pose.pose.orientation = tf2::toMsg(q);

        return pose;
    }

   geometry_msgs::msg::Vector3Stamped to_velocity(
        const builtin_interfaces::msg::Time& stamp,
        const std::string& frame_id,
        double x_vel,
        double y_vel,
        double z_vel)
    {
				geometry_msgs::msg::Vector3Stamped velocity;
				velocity.header.stamp = stamp;
        velocity.header.frame_id = frame_id;
				velocity.vector.x = x_vel;
        velocity.vector.y = y_vel;
        velocity.vector.z = z_vel;

        return velocity;
    }

    rclcpp::Time stamp_from_ts(float ts)
    {
        // ts: seconds since midnight
        
        using days = std::chrono::duration
          <int, std::ratio_multiply<std::ratio<24>, std::chrono::hours::period>>;
        
        auto now = std::chrono::system_clock::now();
        auto now_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
        auto midnight = std::chrono::floor<days>(now);
        auto midnight_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(midnight.time_since_epoch());

        uint64_t ts_nanoseconds = static_cast<uint64_t>(ts * 1e9);
        uint64_t nanoseconds_since_epoch = midnight_since_epoch.count() + ts_nanoseconds;
        rclcpp::Time stamp(nanoseconds_since_epoch);

        //std::cout << "now_since_epoch: " << now_since_epoch.count() 
        //            << " nanoseconds_since_epoch: " << nanoseconds_since_epoch 
        //            << " diff: " << now_since_epoch.count()-nanoseconds_since_epoch << std::endl; 

        return stamp;
    }

    void timer_callback()
    {
        if (robot_->read(data_))
        {
            // successful read

            //std::cout << "Robot at (" << data_.self.pose.x << ", " << data_.self.pose.y << ", " << data_.self.pose.rz << ")" 
            //          << "; Control ball: " << data_.player_status.control_ball << std::endl;           

            rclcpp::Time now = this->get_clock()->now();
            rclcpp::Time stamp = now;

            // Get current robot pose
            geometry_msgs::msg::PoseStamped msl_pose;
            stamp = stamp_from_ts(data_.self.ts);
            msl_pose = to_pose(stamp, "map_msl", data_.self.pose.x, data_.self.pose.y, 0, 0, 0, data_.self.pose.rz);
/*
						msl_pose.header.stamp = stamp;
            msl_pose.header.frame_id = "map_msl";
            msl_pose.pose.position.x = data_.self.pose.x;
            msl_pose.pose.position.y = data_.self.pose.y;
            msl_pose.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, data_.self.pose.rz);
						msl_pose.pose.orientation = tf2::toMsg(q);
*/
            // Transform pose from MSL to ROS map coordinate frame
            geometry_msgs::msg::PoseStamped ros_pose;
            tf_transform<geometry_msgs::msg::PoseStamped>(msl_pose, ros_pose, "map");

						// Get current robot velocity
            geometry_msgs::msg::Vector3Stamped msl_velocity;
            stamp = stamp_from_ts(data_.self.ts);
            msl_velocity = to_velocity(stamp, "base_link_msl", data_.self.vel.x, data_.self.vel.y, 0);
/*
						msl_velocity.header.stamp = stamp
            msl_velocity.header.frame_id = "base_link_msl";
						msl_velocity.vector.x = data_.self.vel.x;
            msl_velocity.vector.y = data_.self.vel.y;
            msl_velocity.vector.z = 0.0;
*/

						// Transform velocity from MSL to ROS base_link coordinate frame
            geometry_msgs::msg::Vector3Stamped ros_velocity;
						tf_transform<geometry_msgs::msg::Vector3Stamped>(msl_velocity, ros_velocity, "base_link");

						//RCLCPP_INFO(this->get_logger(), "Publish Odometry");

            // Transform pose from ROS map coordinate frame to ROS odom frame
            geometry_msgs::msg::PoseStamped ros_odom_pose;
            tf_transform<geometry_msgs::msg::PoseStamped>(ros_pose, ros_odom_pose, "odom");
					
            // Publish odometry message

            nav_msgs::msg::Odometry odom;
            odom.header.stamp = ros_odom_pose.header.stamp;
            odom.header.frame_id = "odom";
						odom.pose.pose = ros_odom_pose.pose;
            odom.child_frame_id = "base_link";
						odom.twist.twist.linear = ros_velocity.vector;
            odom_publisher_->publish(odom);  

            if (tf_broadcast_) {
								//RCLCPP_INFO(this->get_logger(), "Broadcast odom-->base_link");

                // Broadcast odom-->baselink transform

                geometry_msgs::msg::TransformStamped odom_tf;
                odom_tf.header.stamp = ros_odom_pose.header.stamp;
                odom_tf.header.frame_id = "odom";
                odom_tf.child_frame_id = "base_link";
								odom_tf.transform.translation.x = ros_odom_pose.pose.position.x;
								odom_tf.transform.translation.y = ros_odom_pose.pose.position.y;
								odom_tf.transform.translation.z = ros_odom_pose.pose.position.z;
                odom_tf.transform.rotation = ros_odom_pose.pose.orientation;
                odom_broadcaster_->sendTransform(odom_tf);
            }

            // Build and publish worldmodel message

            rs_bridge_msgs::msg::WorldModel wm;
            // Metadata
            wm.header.stamp = stamp; // TODO: use ts
            wm.metadata.version = data_.metadata.version;
	          wm.metadata.hash = data_.metadata.hash;
	          wm.metadata.tick = data_.metadata.tick;

            // HwStatus
            //wm.hw_status.*

            // PlayerStatus 
            wm.player_status.control_ball = data_.player_status.control_ball;

            // local
            wm.self.header.stamp = stamp; // TODO: use ts
            wm.self.header.frame_id = "map";
						wm.self.pose = ros_pose.pose;
            wm.self.child_frame_id = "base_link";
						wm.self.twist.linear = ros_velocity.vector;
            wm.self.confidence = data_.self.confidence;



            // local ball
            wm.ball.header.stamp = stamp; // TODO: use ts
            wm.ball.header.frame_id = "map";
						//wm.ball.pose = ros ball pose
            wm.ball.child_frame_id = "base_link";
						//wm.ball.twist = ros ball twist
            wm.ball.confidence = data_.ball.confidence;

            // local obstacles 
            //wm.obstacles

            // fused

            // pass

            // planner

            wm_publisher_->publish(wm);  

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
