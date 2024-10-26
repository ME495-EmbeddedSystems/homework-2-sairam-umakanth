'''
Add doc string
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

'''
class State(Enum):
    
    Current state of the system.
    Determines what the main timer function should be doing on each iteration
    
    STOPPED = auto(),
    MOVING = auto(),
'''

class turtle_robot(Node):
    '''
    doc string
    '''
    def __init__(self):
        super().__init__('turtle_robot')
        self.cb_group = MutuallyExclusiveCallbackGroup()
        #self.state = State.STOPPED
        self.static_broadcaster = StaticTransformBroadcaster(self)
    
        self.declare_parameter('platform_height', 0.5)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('gravity_accel', 9.81)
        platform_height = self.get_parameter('platform_height').value
        wheel_radius = self.get_parameter('wheel_radius').value
        max_velocity = self.get_parameter('max_velocity').value
        gravity_accel = self.get_parameter('gravity_accel').value
    

        self.odom_base_tf = TransformStamped()
        self.odom_base_tf.header.stamp = self.get_clock().now().to_msg()
        self.odom_base_tf.header.frame_id = 'odom'
        self.odom_base_tf.child_frame_id = 'base_link'
        self.odom_base_tf.transform.translation.z = 2*wheel_radius + 0.5 + 0.5
    

        world_odom_tf = TransformStamped()
        world_odom_tf.header.stamp = self.odom_base_tf.header.stamp
        world_odom_tf.header.frame_id = 'world'
        world_odom_tf.child_frame_id = 'odom'
        world_odom_tf.transform.translation.x = 5.4445
        world_odom_tf.transform.translation.y = 5.4445

        self.joint_states = JointState()

        self.get_logger().info("test4")

        '''Creating publishers'''
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy. TRANSIENT_LOCAL)
        self.js_pub = self.create_publisher(JointState, "joint_states", qos)
        #self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        #self.vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.base_broadcaster = TransformBroadcaster(self)
        self.base_broadcaster.sendTransform(self.odom_base_tf)
        self.static_broadcaster.sendTransform(world_odom_tf)
        
        '''Creating subscribers'''
        #self.goal_sub = self.create_subscription(PoseStamped, "goal_pose", self.goal_sub_callback,10)
        #elf.tilt_sub = self.create_subscription(Tilt, "tilt", self.tilt_sub_callback, 10)
        self.pose_sub = self.create_subscription(Pose, "turtle1/pose",self.pose_sub_callback,10)

        ''' Creating a timer with a callback'''    
        timer_period = 1.0 / 100.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    #def goal_sub_callback(self,msg):
        #self.state = State.MOVING

    def pose_sub_callback(self,msg):
        self.odom_base_tf.header.stamp = self.get_clock().now().to_msg()
        x = 5.4445 - msg.x
        y = 5.4445 - msg.y
        self.odom_base_tf.transform.translation.x = x
        self.odom_base_tf.transform.translation.y = y
        self.joint_states.header.stamp = self.odom_base_tf.header.stamp
        self.joint_states.name = ["base_to_stem","stem_to_wheel","pstem_to_platform"]
        self.joint_states.position = [math.pi - msg.theta, 0, 0]


    def timer_callback(self):
        # Broadcast frame to base at the end of its
        #self.get_logger().info("test2")
        self.base_broadcaster.sendTransform(self.odom_base_tf)
        self.js_pub.publish(self.joint_states)
        #self.get_logger().info("test3")


def main(args=None):
    '''main function'''
    rclpy.init(args=args)
    node = turtle_robot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
