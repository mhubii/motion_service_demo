# Motion Service
**Note** This is a hacked demonstration and should be treated as such! It is an experimental package meant to be used with `lbr_fri_ros2_stack` and `chatgpt_turtlesim`.

## Installation
Clone this repository into your workspace source and build.

## Usage
Run the robot
```shell
ros2 launch lbr_bringup lbr_bringup.launch.py model:=med7 sim:=true
```

Run the demo
```shell
ros2 run motion_service_demo move_node 
```
This node exposes a service under `/relative_move`, which moves the end-effector.
