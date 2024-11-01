'''
Implements control algorithm for robot brick catcher. If brick is catchable, robot drives towards 
the brick, collects it on its platform, drive back to the start and deposits it. If brick is uncatchable,
a message reading "Unreachable" shows up on Rviz and the robot remains stationary

PUBLISHERS:
goal_pose PoseStamped           :   Goal position parameters that the turtle/robot needs to drive to
tilt Tilt                       :   Tilt angle of platform joint
unreachable/visualization_marker Marker
                                :   Text marker indicating that the brick is unreachable

PARAMETERS:
platform_height double          :   Height of platform above the ground 
gravity_accel double            :   Acceleration due to gravity
wheel_radius double             :   Wheel radius
max_velocity double             :   Max velocity of turtle and robot 
'''

from rclpy.node import Node
from rclpy.parameter import Parameter
from enum import Enum, auto
import rclpy
import math
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from turtle_brick_interfaces.msg import Tilt
from visualization_msgs.msg import Marker
from rclpy.duration import Duration
from builtin_interfaces.msg import Duration as DurationMsg

class State(Enum):
    '''
    Current state of the system.
    Determines what the main timer function should be doing on each iteration
    '''
    NOACTION = auto(), # Brick has not dropped yet, no action
    FALLING = auto(),  # Brick is falling, robot drives to brick 
    WAITING = auto(),  # Waiting for brick to fall to platform, if robot reaches brick
    RETURN = auto(),   # Brick is on platform, robot returns to origin
    DEPOSIT = auto(),  # Robot is at origin, deposits brick

class Catcher(Node):
    '''
    Listens to TF frames from rvizz and manages state of robot and brick and provides 
    information as necessary
    '''

    def __init__(self):
        super().__init__('catcher')
        self.state = State.NOACTION
        self.declare_parameter('platform_height', 0.5)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('gravity_accel', 9.81)
        self.platform_height = self.get_parameter('platform_height').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.gravity_accel = self.get_parameter('gravity_accel').value

        self.prev_height = -9000.0
        self.curr_height = -9000.0

        # The buffer stores received tf frames
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.goal_pose_pub = self.create_publisher(PoseStamped, "goal_pose", 10)
        self.tilt_pub = self.create_publisher(Tilt, "tilt", 10)
        self.marker_pub = self.create_publisher(Marker, "unreachable/visualization_marker", 10)

        freq = 250.0
        timer_period = 1.0 / freq
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        '''Callback function for timer'''
        try:
            #self.get_logger().info("another test")
            self.base_to_odom = self.buffer.lookup_transform('base_link','odom',rclpy.time.Time().to_msg())
            #self.get_logger().info("another test 2")
            self.platform_to_brick = self.buffer.lookup_transform('platform', 'brick',rclpy.time.Time().to_msg())  
            self.world_to_brick = self.buffer.lookup_transform('world', 'brick',rclpy.time.Time().to_msg())  
            self.world_to_odom = self.buffer.lookup_transform('world','odom', rclpy.time.Time().to_msg())
            self.world_to_platform = self.buffer.lookup_transform('world','platform', rclpy.time.Time().to_msg())
            
            self.prev_height = self.curr_height
            self.curr_height = self.platform_to_brick.transform.translation.z
            goal_pose = PoseStamped()
            #self.get_logger().info(f'{self.curr_height}, {self.prev_height}')
            if self.prev_height == self.curr_height and self.state == State.NOACTION:
                self.state = State.NOACTION
            elif 0.2 < abs(self.prev_height-self.curr_height) <= 0.4:
                self.get_logger().info(f'{abs(self.prev_height-self.curr_height)}')
                self.state = State.FALLING
                #self.get_logger().info('pre falling test')
            elif (self.platform_to_brick.transform.translation.x < 0.3
                and self.platform_to_brick.transform.translation.y < 0.3
                and self.platform_to_brick.transform.translation.z < 0.5):
                    #self.get_logger().info("t2")
                    self.state = State.RETURN
            elif (self.state == State.RETURN and
                0.5 > abs(self.base_to_odom.transform.translation.x) >= 0.0 
                and 0.5 > abs(self.base_to_odom.transform.translation.y) >= 0.0):
                self.state = State.DEPOSIT
            
            if self.state == State.NOACTION:
                tilt = Tilt()
                tilt.tilt_angle = 0.0
                self.tilt_pub.publish(tilt)
                self.get_logger().info("No action")
                #goal_pose.header.stamp = self.get_clock().now().to_msg()
                #goal_pose.pose.position.x = self.world_to_platform.transform.translation.x
                #goal_pose.pose.position.y = self.world_to_platform.transform.translation.y
                #self.goal_pose_pub.publish(goal_pose)
                #pass
            elif self.state == State.FALLING:
                #self.get_logger().info("falling test")
                dist = self.get_dist(self.platform_to_brick.transform)
                time_taken = (dist-0.5)/self.max_velocity
                if self.curr_height >= 0.0:
                    fall_time = (self.curr_height*2/self.gravity_accel)**0.5
                else:
                    fall_time = 0.0
                self.get_logger().info(f'{time_taken},{fall_time}')
                if time_taken < fall_time:
                    goal_pose.header.stamp = self.get_clock().now().to_msg()
                    goal_pose.pose.position.x = self.world_to_brick.transform.translation.x
                    goal_pose.pose.position.y = self.world_to_brick.transform.translation.y
                    self.goal_pose_pub.publish(goal_pose)
                else:
                    self.publish_unreachable_marker()
                    self.state = State.NOACTION
                self.get_logger().info("Falling")
                    #self.state = State.NOACTION
            elif self.state == State.RETURN:
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = self.world_to_odom.transform.translation.x
                goal_pose.pose.position.y = self.world_to_odom.transform.translation.y
                self.goal_pose_pub.publish(goal_pose)
                self.get_logger().info("Return")
            elif self.state == State.DEPOSIT:
                tilt = Tilt()
                tilt.tilt_angle = 0.7854
                self.tilt_pub.publish(tilt)
                self.get_logger().info("Deposit")

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().debug(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f'Connectivity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().debug(f'Extrapolation exception: {e}')

    def get_dist(self,tf):
        '''
        Function that calculates distance between two frames based on their transform
        
        ARGS:
        tf: Transformation matrix between two frames
        '''
        distance = (tf.translation.x**2 + tf.translation.y**2)**0.5
        return distance
    
    def publish_unreachable_marker(self):
        ''' Function to publish "Unreachable" marker'''
        # Create a text marker message
        marker = Marker()
        marker.header.frame_id = "world"  # Frame of reference
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.ns = "catcher"
        marker.id = 9
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position of the text marker (adjust as needed)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.0  # Place it slightly above the ground
        
        # Set the text, color, and scale
        marker.text = "Unreachable"
        marker.scale.z = 0.5  # Text height
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Set the lifetime to 3 seconds
        marker.lifetime.sec = 3
        
        # Publish the marker
        self.marker_pub.publish(marker)
    
def catcher_entry(args=None):
    '''Catcher entry'''
    rclpy.init(args=args)
    node = Catcher()
    rclpy.spin(node)
    rclpy.shutdown()
