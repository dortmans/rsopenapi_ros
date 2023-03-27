# rsopenapi_ros

A bridge between [RSOPENAPI](https://github.com/RobBurgers/rsopenapi) and ROS 2.

## Installation

Clone and build (assuming your workspace is `~/ros2_ws`):
```
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/dortmans/rsopenapi_ros.git
./rsopenapi_ros/rs_bridge/scripts/install_requirements.sh
cd ..
colcon build --symlink-install --packages-up-to rs_bridge
source ./install/setup.bash
```


## Run simulator and gui.

Open new terminal window and run following commands to start the simulator:
```
cd ~/ros2_ws/src/rsopenapi_ros/rs_bridge/rsopenapi/scripts
./rsim.sh
./rsopenapi.sh -i 1
```

Open a second terminal window and execute following command to start the gui in your browser at [http://localhost:6080](http://localhost:6080/):
```
cd ~/ros2_ws/src/rsopenapi_ros/rs_bridge/rsopenapi/scripts
./gui.sh
```

## Run the RSOPENAPI Bridge.

Print launch arguments:
```
ros2 launch rs_bridge rs_bridge_launch.py -s
```

Run with default arguments:
```
ros2 launch rs_bridge rs_bridge_launch.py
```

Example run with arguments:
```
ros2 launch rs_bridge rs_bridge_launch.py robot:=1 hash:="'0x7d9066e102eb9a4f'"
```




