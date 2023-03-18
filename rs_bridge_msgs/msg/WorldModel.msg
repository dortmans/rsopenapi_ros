# Interrobot data
# reference: rsopenapi/src/include/rtdb_types.h

std_msgs/Header header # timestamp
Metadata metadata

HwStatus hw_status
PlayerStatus player_status

# local
Object self
Object ball
Object[] obstacles

# fused
Object fball
Object[] us
Object[] them
Object selfloc
Object selfloc_omni

# pass
PassDetail pass_detail
PassRequest pass_request

# planner
BallPickup ball_pickup
nav_msgs/Path planned_path
builtin_interfaces/Duration time_in_own_penalty_area
builtin_interfaces/Duration time_in_opponent_penalty_area
