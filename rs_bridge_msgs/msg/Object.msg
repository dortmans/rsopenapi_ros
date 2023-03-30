std_msgs/Header header		# timestamp and frame reference for Pose
string child_frame_id			# frame reference for Twist

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist

float32 confidence				# 0..1
