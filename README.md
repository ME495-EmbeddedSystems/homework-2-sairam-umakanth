# ME495 Embedded Systems Homework 2
Author: Sairam Umakanth

This package spawns a robot that has exact kinematics to the turtlesim_node. In an arena generated in rviz, it drops a brick from a placed location and the turtle catches the brick if it can get there before it falls, and if it cannot, it spurts a message saying that it is "unreachable"

## Quickstart
1. Use `ros2 launch turtle_brick ros2 launch turtle_brick show_turtle.launch.xm` to start the arena and turtle simulation
2. Use `ros2 service call /drop std_srvs/srv/Empty` to drop a brick
3. Here is a video of the turtle when the brick is within catching range
   [me495_hw2_demo1.webm](https://github.com/user-attachments/assets/ac608951-62cd-4b7f-84da-6897f5cc723c)

4. Here is a video of the turtle when the brick cannot be caught

   [me495_hw2_demo1.webm](https://github.com/user-attachments/assets/ac608951-62cd-4b7f-84da-6897f5cc723c)