# ME495 Embedded Systems Homework 2
Author: Sairam Umakanth

This package spawns a robot that has exact kinematics to the turtlesim_node. In an arena generated in rviz, it drops a brick from a placed location and the turtle catches the brick if it can get there before it falls, and if it cannot, it spurts a message saying that it is "unreachable"

## Quickstart
1. Use `ros2 launch turtle_brick ros2 launch turtle_brick show_turtle.launch.xm` to start the arena and turtle simulation
2. Use `ros2 service call /drop std_srvs/srv/Empty` to drop a brick
3. Here is a video of the turtle when the brick is within catching range

   [me495_hw2_p1.webm](https://github.com/user-attachments/assets/c528552b-bf44-4d7b-ad55-7683048be5dd)

4. Here is a video of the turtle when the brick cannot be caught

   [me495_hw2_p2.webm](https://github.com/user-attachments/assets/054a6581-1cb1-4fc9-9504-e635e040cc60)