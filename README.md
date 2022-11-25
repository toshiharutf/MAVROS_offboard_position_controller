# MAVROS Offboard Multicopter Position Controller demo project
This project is short demo of a position controller for UAVs running PX4 via MAVROS.
The position controller is delivered within a ROS package, which also executes the node that communicates with the real or simulated UAV.

## Getting Started

### Gazebo simulation
1) Execute the Gazebo simulation with the UAV running PX4.
2) Build, source, and run the pos_controller package.
3) Run the pos_controller node.
4) To start the demo, call the service "/pos_controller/start_test" and send a True value. If it is successful, the success flag will be true and a message will be received.
5) Some info messages will be displayed, showing the progress of the waypoint mission.
**Note**: A short video is provided that illustrates how to run the demo with Gazebo.

## Extras
The additional service "/pos_controller/pid_turner" allows real-time PID tuning.
The provided values has been tested with the Gazebo version of the 3DR Iris quadcopter.

## Video of the demo running in Gazebo
https://youtu.be/23s8DY54aME
