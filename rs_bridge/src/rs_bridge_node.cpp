#include <unistd.h>
#include <math.h> 
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
            25ms, std::bind(&RsBridgeNode::timer_callback, this));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        rclcpp::sleep_for(1s); // to wait for TF to come-up

        last_twist_received_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

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
        transformed_pose.header.frame_id = target_frame;
    		return true;
      } catch (const tf2::TransformException & e) {
          RCLCPP_DEBUG(get_logger(), "Failed to transform: (%s)", e.what());
      }
			return false;
    }


    geometry_msgs::msg::PoseStamped to_ros_pose(
        double x,
        double y,
        double z,
        double rz)
    /*
     * Position (x, y, z) is relative to world coordinate system
     * Heading (rz) is relative to robot coordinate system
    */
    {
        geometry_msgs::msg::PoseStamped msl_pose, ros_pose;

        msl_pose.header.frame_id = "map_msl";
        msl_pose.pose.position.x = x;
        msl_pose.pose.position.y = y;
        msl_pose.pose.position.z = z;
        double yaw = rz + M_PI_2; // correction for angle between x and y axis
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
				msl_pose.pose.orientation = tf2::toMsg(q);

        // Transform pose to ROS world coordinate system (map)
        tf_transform<geometry_msgs::msg::PoseStamped>(msl_pose, ros_pose, "map");

        return ros_pose;
    }


    geometry_msgs::msg::Vector3Stamped to_ros_velocity(
        double x_vel,
        double y_vel,
        double z_vel)
    /*
     * Velocity vector (x_vel, y_vel, z_vel) is relative to MSL world coordinate system (map_msl)
     * Transform velocity vector to ROS robot coordinate system (base_link)
    */
    {
        geometry_msgs::msg::Vector3Stamped msl_velocity, ros_velocity;

        msl_velocity.header.frame_id = "map_msl";
				msl_velocity.vector.x = x_vel;
        msl_velocity.vector.y = y_vel;
        msl_velocity.vector.z = z_vel;

				tf_transform<geometry_msgs::msg::Vector3Stamped>(msl_velocity, ros_velocity, "base_link");

        return ros_velocity;
    }


    geometry_msgs::msg::Vector3Stamped to_msl_velocity(
        double x_vel,
        double y_vel,
        double z_vel)
    /*
     * Velocity vector (x_vel, y_vel, z_vel) is relative to ROS robot coordinate system (base_link)
     * Transform velocity vector to MSL robot coordinate system (base_link_msl)
    */
    {
        geometry_msgs::msg::Vector3Stamped msl_velocity, ros_velocity;

        ros_velocity.header.frame_id = "base_link";
				ros_velocity.vector.x = x_vel;
        ros_velocity.vector.y = y_vel;
        ros_velocity.vector.z = z_vel;

				tf_transform<geometry_msgs::msg::Vector3Stamped>(ros_velocity, msl_velocity, "base_link_msl");

        return msl_velocity;
    }


    rclcpp::Time ros_stamp_from_ts(float ts)
    /*
     * ts: seconds since midnight
    */
    {
        using days = std::chrono::duration
          <int, std::ratio_multiply<std::ratio<24>, std::chrono::hours::period>>;
        
        auto now = std::chrono::system_clock::now();
        auto now_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
        auto midnight = std::chrono::floor<days>(now);
        auto midnight_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(midnight.time_since_epoch());

        uint64_t ts_nanoseconds = static_cast<uint64_t>(ts * 1e9);
        uint64_t ts_nanoseconds_since_epoch = midnight_since_epoch.count() + ts_nanoseconds;
        rclcpp::Time stamp(ts_nanoseconds_since_epoch);

        return stamp;
    }


    void timer_callback()
    {
        // Current time
        rclcpp::Time now = this->get_clock()->now();

        if (robot_->read(data_))
        {
          geometry_msgs::msg::PoseStamped ros_pose;
          geometry_msgs::msg::PoseStamped ros_odom_pose;
          geometry_msgs::msg::PoseStamped msl_pose;
          geometry_msgs::msg::Vector3Stamped ros_velocity;
          geometry_msgs::msg::Vector3Stamped msl_velocity;            
            
            // Current robot pose
            ros_pose = to_ros_pose(data_.self.pose.x, data_.self.pose.y, 0, data_.self.pose.rz);

						// Current robot velocity
            ros_velocity = to_ros_velocity(data_.self.vel.x, data_.self.vel.y, 0);

            // Transform pose from ROS map coordinate frame to ROS odom frame
            tf_transform<geometry_msgs::msg::PoseStamped>(ros_pose, ros_odom_pose, "odom");

            if (tf_broadcast_) {
                // Broadcast odom-->baselink transform
                geometry_msgs::msg::TransformStamped odom_tf;
                odom_tf.header.stamp = ros_stamp_from_ts(data_.self.ts);
                odom_tf.header.frame_id = "odom";
                odom_tf.child_frame_id = "base_link";
								odom_tf.transform.translation.x = ros_odom_pose.pose.position.x;
								odom_tf.transform.translation.y = ros_odom_pose.pose.position.y;
								odom_tf.transform.translation.z = ros_odom_pose.pose.position.z;
                odom_tf.transform.rotation = ros_odom_pose.pose.orientation;

                odom_broadcaster_->sendTransform(odom_tf);
            }

            // Build and publish odometry message
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = ros_stamp_from_ts(data_.self.ts);
            odom.header.frame_id = "odom";
						odom.pose.pose = ros_odom_pose.pose;
            odom.child_frame_id = "base_link";
						odom.twist.twist.linear = ros_velocity.vector;
            
            odom_publisher_->publish(odom);

            // Build and publish WorldModel message
            rs_bridge_msgs::msg::WorldModel wm;
            rs_bridge_msgs::msg::Object object;

            // Metadata
            wm.header.stamp = ros_stamp_from_ts(data_.metadata.ts);
            wm.metadata.version = data_.metadata.version;
	          wm.metadata.hash = data_.metadata.hash;
	          wm.metadata.tick = data_.metadata.tick;

            // HwStatus
            wm.hw_status.battery_voltage = data_.hw_status.battery_voltage;
            wm.hw_status.kicker_soc = data_.hw_status.kicker_soc;
            wm.hw_status.is_freemode = data_.hw_status.is_freemode;
            wm.hw_status.is_compass_healthy = data_.hw_status.is_compass_healthy;
            wm.hw_status.is_kicker_healthy = data_.hw_status.is_kicker_healthy;
            wm.hw_status.is_omni_healthy = data_.hw_status.is_omni_healthy;
            wm.hw_status.is_battery_low = data_.hw_status.is_battery_low;
            wm.hw_status.is_disk_low = data_.hw_status.is_disk_low;
            wm.hw_status.kinect_num_balls = data_.hw_status.kinect_num_balls;

            // PlayerStatus 
            wm.player_status.vendor_id = wm.player_status.vendor_id;
            wm.player_status.shirt_number = data_.player_status.shirt_number;
            wm.player_status.shirt_color = data_.player_status.shirt_color;
            wm.player_status.assigned_role = data_.player_status.assigned_role;
            wm.player_status.active_role = data_.player_status.active_role;
            wm.player_status.dynamic_role = data_.player_status.dynamic_role;
            wm.player_status.game_state = data_.player_status.game_state;
            wm.player_status.behavior_state = data_.player_status.behavior_state;
            wm.player_status.control_ball = data_.player_status.control_ball;
            wm.player_status.team_control_ball = data_.player_status.team_control_ball;

            // local self
            wm.self.header.stamp = ros_stamp_from_ts(data_.self.ts);
            wm.self.header.frame_id = "map";
						wm.self.pose.pose = ros_pose.pose;
            wm.self.child_frame_id = "base_link";
						wm.self.twist.twist.linear = ros_velocity.vector;
            wm.self.confidence = data_.self.confidence;

            // local ball
            ros_pose = to_ros_pose(data_.ball.pos.x, data_.ball.pos.y, data_.ball.pos.z, 0);
            ros_velocity = to_ros_velocity(data_.ball.vel.x, data_.ball.vel.y, data_.ball.vel.z);
            wm.ball.header.stamp = ros_stamp_from_ts(data_.ball.ts);
            wm.ball.header.frame_id = "map";
						wm.ball.pose.pose = ros_pose.pose;
            wm.ball.child_frame_id = "base_link";
						wm.ball.twist.twist.linear = ros_velocity.vector;
            wm.ball.confidence = data_.ball.confidence;

            // local obstacles 
            for (auto & obstacle : data_.obstacles)
            {
              ros_pose = to_ros_pose(obstacle.pose.x, obstacle.pose.y, 0, obstacle.pose.rz);
              ros_velocity = to_ros_velocity(obstacle.vel.x, obstacle.vel.y, 0);
              object.header.stamp = ros_stamp_from_ts(obstacle.ts);
              object.header.frame_id = "map";
              object.pose.pose = ros_pose.pose;
              object.child_frame_id = "base_link";
              object.twist.twist.linear = ros_velocity.vector;
              object.confidence = obstacle.confidence;
              wm.obstacles.push_back(object);
            }

            // fused

            // fused fball
            ros_pose = to_ros_pose(data_.fball.pos.x, data_.fball.pos.y, data_.fball.pos.z, 0);
            ros_velocity = to_ros_velocity(data_.fball.vel.x, data_.fball.vel.y, data_.fball.vel.z);
            wm.fball.header.stamp = ros_stamp_from_ts(data_.fball.ts);
            wm.fball.header.frame_id = "map";
						wm.fball.pose.pose = ros_pose.pose;
            wm.fball.child_frame_id = "base_link";
						wm.fball.twist.twist.linear = ros_velocity.vector;
            wm.fball.confidence = data_.fball.confidence;

            // fused us
            for (auto & one_of_us : data_.us)
            {
              ros_pose = to_ros_pose(one_of_us.pose.x, one_of_us.pose.y, 0, one_of_us.pose.rz);
              ros_velocity = to_ros_velocity(one_of_us.vel.x, one_of_us.vel.y, 0);
              object.header.stamp = ros_stamp_from_ts(one_of_us.ts);
              object.header.frame_id = "map";
              object.pose.pose = ros_pose.pose;
              object.child_frame_id = "base_link";
              object.twist.twist.linear = ros_velocity.vector;
              object.confidence = one_of_us.confidence;
              wm.us.push_back(object);
            }

            // fused them
            for (auto & one_of_them : data_.them)
            {
              ros_pose = to_ros_pose(one_of_them.pose.x, one_of_them.pose.y, 0, one_of_them.pose.rz);
              ros_velocity = to_ros_velocity(one_of_them.vel.x, one_of_them.vel.y, 0);
              object.header.stamp = ros_stamp_from_ts(one_of_them.ts);
              object.header.frame_id = "map";
              object.pose.pose = ros_pose.pose;
              object.child_frame_id = "base_link";
              object.twist.twist.linear = ros_velocity.vector;
              object.confidence = one_of_them.confidence;
              wm.them.push_back(object);
            }

            // fused selfloc
            ros_pose = to_ros_pose(data_.selfloc.pose.x, data_.selfloc.pose.y, 0, data_.selfloc.pose.rz);
            ros_velocity = to_ros_velocity(data_.selfloc.vel.x, data_.selfloc.vel.y, 0);
            wm.selfloc.header.stamp = ros_stamp_from_ts(data_.selfloc.ts);
            wm.selfloc.header.frame_id = "map";
						wm.selfloc.pose.pose = ros_pose.pose;
            wm.selfloc.child_frame_id = "base_link";
						wm.selfloc.twist.twist.linear = ros_velocity.vector;
            wm.selfloc.confidence = data_.selfloc.confidence;

            //wm.selfloc_omni.*
            ros_pose = to_ros_pose(data_.selfloc_omni.pose.x, data_.selfloc_omni.pose.y, 0, data_.selfloc_omni.pose.rz);
            ros_velocity = to_ros_velocity(data_.selfloc_omni.vel.x, data_.selfloc_omni.vel.y, 0);
            wm.selfloc_omni.header.stamp = ros_stamp_from_ts(data_.selfloc_omni.ts);
            wm.selfloc_omni.header.frame_id = "map";
						wm.selfloc_omni.pose.pose = ros_pose.pose;
            wm.selfloc_omni.child_frame_id = "base_link";
						wm.selfloc_omni.twist.twist.linear = ros_velocity.vector;
            wm.selfloc_omni.confidence = data_.selfloc_omni.confidence;

            // pass

            // pass_detail
            wm.pass_detail.header.stamp = ros_stamp_from_ts(data_.pass_detail.ts);
            wm.pass_detail.valid = data_.pass_detail.valid;
            wm.pass_detail.target_id = data_.pass_detail.target_id;
            wm.pass_detail.kicked = data_.pass_detail.kicked;
            wm.pass_detail.eta = data_.pass_detail.eta;
            wm.pass_detail.speed = data_.pass_detail.speed;
            wm.pass_detail.angle = data_.pass_detail.angle;
            ros_pose = to_ros_pose(data_.pass_detail.origin.x, data_.pass_detail.origin.y, 0, 0);
            wm.pass_detail.origin = ros_pose;
            ros_pose = to_ros_pose(data_.pass_detail.target.x, data_.pass_detail.target.y, 0, 0);
            wm.pass_detail.target = ros_pose;

            // pass_request
            wm.pass_request.header.stamp = ros_stamp_from_ts(data_.pass_request.ts);
            wm.pass_request.valid = data_.pass_request.valid;
            wm.pass_request.eta = data_.pass_request.eta;
            ros_pose = to_ros_pose(data_.pass_request.target.x, data_.pass_request.target.y, 0, 0);
            wm.pass_request.target = ros_pose;       

            // planner

            // planner ball_pickup
            ros_pose = to_ros_pose(data_.ball_pickup.pos.x, data_.ball_pickup.pos.y, 0, 0);
            wm.ball_pickup.header.stamp = ros_stamp_from_ts(data_.ball_pickup.ts);
            wm.ball_pickup.valid = data_.ball_pickup.valid;
            wm.ball_pickup.pos = ros_pose;

            // planner planned_path
            nav_msgs::msg::Path path;
            for (auto & pos : data_.planned_path)
            {
              ros_pose = to_ros_pose(pos.x, pos.y, 0, 0);
              wm.planned_path.poses.push_back(ros_pose);
            }

            // planner time_in_own penalty_area
            wm.time_in_own_penalty_area = rclcpp::Duration::from_seconds(data_.time_in_own_penalty_area);

            // planner time_in_opponent_penalty_area
            wm.time_in_opponent_penalty_area = rclcpp::Duration::from_seconds(data_.time_in_opponent_penalty_area);

            // Publish the WorldModel message
            wm_publisher_->publish(wm);  

        }
        else
        {
            // unsuccessful read
						RCLCPP_INFO(this->get_logger(), "No data");
        }

        // Stop robot when not receiving Twist messages anymore
        if ( (now - last_twist_received_) > rclcpp::Duration(1s) )
        {
          robot_->writeVelocity(0, 0, 0);
        }
       
    }

    
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Received Twist");

        last_twist_received_ = this->get_clock()->now();

        geometry_msgs::msg::Vector3Stamped msl_velocity;
        msl_velocity = to_msl_velocity(msg->linear.x, msg->linear.y, 0);
        
        // set velocity in robot coordinates
				robot_->writeVelocity(msl_velocity.vector.x, msl_velocity.vector.y, msg->angular.z);      
    }


    rclcpp::Publisher<rs_bridge_msgs::msg::WorldModel>::SharedPtr wm_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

    rclcpp::Time last_twist_received_;
 
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
