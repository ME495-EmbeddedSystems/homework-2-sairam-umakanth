'''
Acts as interface between turtlesim and the robot. Sends velocity commands to turtle and sends corresponding 
transforms to the robot to move it. Also sends joint states to operate robot orientation changes.

PUBLISHERS:
turtle1/cmd_vel Twist           :   Velocity of turtle
joint_state JointStates         :   Joint state object that sends joint state information to robot
odom Odometry                   :   Robot speed, position and orientation with respect to odom frame

SUBSCRIBERS:
goal_pose PoseStamped           :   Goal position parameters of the turtle
tilt Tilt                       :   Tilt angle of platform joint
pose Pose                       :   Current position parameters of the turtle

PARAMETERS:
platform_height double          :   Height of platform above the ground 
gravity_accel double            :   Acceleration due to gravity
wheel_radius double             :   Wheel radius
max_velocity double             :   Max velocity of turtle and robot 
'''


import rclpy
import math
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Empty
from enum import Enum, auto
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from turtle_brick_interfaces.msg import Tilt
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class turtle_robot(Node):
    '''
    Main node that interfaces between robot and turtle by subscribing to turtle's current pose and goal pose and 
    broadcasting corrseponding transforms and joint states for robot, and command velocity for turtle.
    '''
    def __init__(self):
        super().__init__('turtle_robot')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.declare_parameter('platform_height', 0.5)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('gravity_accel', 9.81)
        self.platform_height = self.get_parameter('platform_height').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.gravity_accel = self.get_parameter('gravity_accel').value

        self.cmd_vel = Twist()
        self.odom = Odometry()
        self.curr_pose = Pose()

        self.odom_base_tf = TransformStamped()
        self.odom_base_tf.header.stamp = self.get_clock().now().to_msg()
        self.odom_base_tf.header.frame_id = 'odom'
        self.odom_base_tf.child_frame_id = 'base_link'
        self.odom_base_tf.transform.translation.z = 2.0*self.wheel_radius + 0.5 + 0.5
    

        world_odom_tf = TransformStamped()
        world_odom_tf.header.stamp = self.odom_base_tf.header.stamp
        world_odom_tf.header.frame_id = 'world'
        world_odom_tf.child_frame_id = 'odom'
        world_odom_tf.transform.translation.x = 5.4445
        world_odom_tf.transform.translation.y = 5.4445
        # 5.445 was found to be the spawn point of the turtle 

        self.joint_states = JointState()
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.name = ["base_to_stem","stem_to_wheel","pstem_to_platform"]
        self.joint_states.position = [0, 0, 0]
        self.tilt = Tilt()
        self.wheel_orientation = 0.0

        self.get_logger().info("test4")

        '''Creating publishers'''
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy. TRANSIENT_LOCAL)
        self.js_pub = self.create_publisher(JointState, "joint_states", qos)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.base_broadcaster = TransformBroadcaster(self)
        self.base_broadcaster.sendTransform(self.odom_base_tf)
        self.static_broadcaster.sendTransform(world_odom_tf)
        
        '''Creating subscribers'''
        self.goal_sub = self.create_subscription(PoseStamped, "goal_pose", self.goal_sub_callback,10)
        self.tilt_sub = self.create_subscription(Tilt, "tilt", self.tilt_sub_callback, 10)
        self.pose_sub = self.create_subscription(Pose, "turtle1/pose",self.pose_sub_callback,10)

        ''' Creating a timer with a callback'''    
        timer_period = 1.0 / 100.0
        self.dt = timer_period
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def goal_sub_callback(self,msg):
        '''
        Callback function for goal pose subscriber

        Gets live goal position for the turtle/robot to move to

        ARGS:
        msg: Position parameters for goal position
        '''
        x = -(msg.pose.position.x - self.curr_pose.x)
        y = -(msg.pose.position.y - self.curr_pose.y)
        self.wheel_orientation = math.atan2(y,x)
        dist = ((x)**2 + (y)**2)**0.5
        if dist < 0.3:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.linear.y = 0.0
        else:
            self.cmd_vel.linear.x = -self.max_velocity * math.cos(math.atan2(y,x))
            self.cmd_vel.linear.y = -self.max_velocity * math.sin(math.atan2(y,x))
        
        self.vel_pub.publish(self.cmd_vel)
        #self.get_logger().info("t5")
        #self.joint_states.velocity = [0, self.max_velocity/self.wheel_radius, 0]


    def tilt_sub_callback(self,msg):
        '''
        Callback function for tilt angle subscriber

        Gets joint state coordinate of platform joint 

        ARGS:
        msg: Tilt angle of platform joint
        '''
        self.tilt = msg
        

    def pose_sub_callback(self,msg):
        '''
        Callback function for pose subscriber

        Gets live position coordinates of the robot

        ARGS:
        msg: Position parameters of turtle
        '''
        self.odom_base_tf.header.stamp = self.get_clock().now().to_msg()
        self.curr_pose = msg
        x = msg.x - 5.445
        y = msg.y - 5.445
        self.odom_base_tf.transform.translation.x = x
        self.odom_base_tf.transform.translation.y = y

        self.odom.header.stamp = self.odom_base_tf.header.stamp
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        self.odom.pose.pose.position.x = float(x)
        self.odom.pose.pose.position.y = float(y)
        self.odom.pose.pose.position.z = float(self.odom_base_tf.transform.translation.z)
        self.odom.twist.twist = self.cmd_vel

    def timer_callback(self):
        '''Timer callback function'''
        # Broadcast frame to base at the end of its
        #self.get_logger().info("test2")
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.odom_base_tf.header.stamp = self.get_clock().now().to_msg()
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.name = ["base_to_stem","stem_to_wheel","pstem_to_platform"]
        self.joint_states.position = [
            self.wheel_orientation, self.max_velocity/self.wheel_radius*self.dt, self.tilt.tilt_angle]
        self.base_broadcaster.sendTransform(self.odom_base_tf)
        self.js_pub.publish(self.joint_states)
        self.odom_pub.publish(self.odom)
        #self.get_logger().info("test3")


def main(args=None):
    '''main function'''
    rclpy.init(args=args)
    node = turtle_robot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
