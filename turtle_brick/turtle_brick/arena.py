'''
Maintains arena characteristics. Spawns arena walls and brick, drops brick and manages brick/world physics. Contains
two services to drop the brick and place the brick at anywhere in the space of the arena in rviz

PUBLISHERS:
walls/visualization_marker_array MarkerArray    :   Marker array for arena walls
brick/visualization_marker                      :   Marker for brick

SUBSCRIBERS:
tilt Tilt                                       :   Tilt angle for platform joint

SERVICES OFFERED:
place_point PlacePoint                          :   Places turtle at a point provided by user
drop Empty                                      :   Drops brick

PARAMETERS:
platform_height double                          :   Height of platform above the ground 
gravity_accel double                            :   Acceleration due to gravity
wheel_radius double                             :   Wheel radius
max_velocity double                             :   Max velocity of turtle and robot 
'''

from geometry_msgs.msg import TransformStamped, Point
import rclpy
import math
from rclpy.node import Node
from enum import Enum, auto
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
from turtle_brick.physics import World
from turtle_brick_interfaces.srv import PlacePoint
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros
from rclpy.duration import Duration

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

class arena(Node):
    def __init__(self): 
        super().__init__('arena')
        self.buffer = Buffer()
        self.state = State.NOACTION
        self.listener = TransformListener(self.buffer, self)
        self.prev_height = -9000.0
        self.curr_height = -9000.0
        self.slide_vel = 0.0

        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('gravity_accel', 9.81)
        self.declare_parameter('wheel_radius',0.5)
        self.declare_parameter('platform_height', 1.0)
        self.max_velocity = self.get_parameter('max_velocity').value
        self.gravity_accel = self.get_parameter('gravity_accel').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.platform_height = self.get_parameter('platform_height').value
        self.brick_initialization = Point()
        self.brick_initialization.x = 5.445
        self.brick_initialization.y = 7.0
        self.brick_initialization.z = 7.0
        # Define QoS settings and callback group
        self.cb_group = MutuallyExclusiveCallbackGroup()
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Create publisher for MarkerArray
        self.marker_pub = self.create_publisher(MarkerArray, 'walls/visualization_marker_array', markerQoS)

        # Create MarkerArray and populate with markers
        marker_array = MarkerArray()

        # Define dimensions and wall positions
        wall_length = 5.445 * 2
        wall_thickness = 0.25
        wall_height = 2.0

        # Define positions and orientations for the four walls around the area
        walls = [
            {'position': (wall_length / 2, 0.0), 'orientation': (0.0, 0.0, 0.0, 1.0), 'scale': (wall_length, wall_thickness, wall_height)},  # Bottom wall
            {'position': (wall_length, wall_length / 2), 'orientation': (0.0, 0.0, 0.707, 0.707), 'scale': (wall_length, wall_thickness, wall_height)},  # Right wall
            {'position': (wall_length / 2, wall_length), 'orientation': (0.0, 0.0, 1.0, 0.0), 'scale': (wall_length, wall_thickness, wall_height)},  # Top wall
            {'position': (0.0, wall_length / 2), 'orientation': (0.0, 0.0, -0.707, 0.707), 'scale': (wall_length, wall_thickness, wall_height)}  # Left wall
        ]

        # Create each wall marker and add to MarkerArray
        for i, wall in enumerate(walls):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i + 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Set wall dimensions
            marker.scale.x = wall['scale'][0]
            marker.scale.y = wall['scale'][1]
            marker.scale.z = wall['scale'][2]

            # Set wall position
            marker.pose.position.x = wall['position'][0]
            marker.pose.position.y = wall['position'][1]
            marker.pose.position.z = wall_height / 2

            # Set wall orientation
            marker.pose.orientation.x = wall['orientation'][0]
            marker.pose.orientation.y = wall['orientation'][1]
            marker.pose.orientation.z = wall['orientation'][2]
            marker.pose.orientation.w = wall['orientation'][3]

            # Set wall color
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            # Add the marker to the array
            marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)

        self.world_to_brick = TransformStamped()
        self.world_to_brick.header.stamp = self.get_clock().now().to_msg()
        self.world_to_brick.header.frame_id = 'world'
        self.world_to_brick.child_frame_id = 'brick'
        self.world_to_brick.transform.translation.x = self.brick_initialization.x
        self.world_to_brick.transform.translation.y = self.brick_initialization.y
        self.world_to_brick.transform.translation.z = self.brick_initialization.z
        self.brick_tf_broadcaster = TransformBroadcaster(self)
        self.brick_tf_broadcaster.sendTransform(self.world_to_brick)

        self.brick_pub = self.create_publisher(Marker, "brick/visualization_marker", markerQoS)
        self.get_logger().info("brick test 1")
        self.brick = Marker()
        self.brick.header.frame_id = 'brick'
        self.brick.header.stamp = self.world_to_brick.header.stamp
        self.brick.id = 5
        self.brick.type = Marker.CUBE
        self.brick.action = Marker.ADD
        self.brick.scale.x = 0.4
        self.brick.scale.y = 0.2
        self.brick.scale.z = 0.2
        self.brick.pose.position.x = 0.0
        self.brick.pose.position.y = 0.0
        self.brick.pose.position.z = 0.0
        self.brick.pose.orientation.x = 1.0
        self.brick.pose.orientation.y = 0.0
        self.brick.pose.orientation.z = 0.0
        self.brick.pose.orientation.w = 0.0
        self.brick.color.r = 0.5
        self.brick.color.g = 0.0
        self.brick.color.b = 0.0
        self.brick.color.a = 1.0
        self.brick_pub.publish(self.brick)

        # Timer 
        freq = 250.0
        timer_period = 1.0 / freq
        self.dt = timer_period
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.world = World(brick=self.brick_initialization, gravity=self.gravity_accel, radius=self.wheel_radius, dt=timer_period)
        self.place_service = self.create_service(PlacePoint, 'place_point', self.place_callback)
        self.load_service = self.create_service(Empty, 'drop', self.drop_callback)
        self.falling = False



    def place_callback(self, request, response):
        location = request.place_point
        self.world.brick = location
        self.brick_falling = False
        self.update_brick_marker()
        response.success = True
        return response

    def drop_callback(self, request, response):
        self.state = State.FALLING
        return response
    
    def update_brick_marker(self):
        self.world_to_brick.header.stamp = self.get_clock().now().to_msg()
        self.world_to_brick.transform.translation.x = float(self.world.brick.x)
        self.world_to_brick.transform.translation.y = float(self.world.brick.y)
        self.world_to_brick.transform.translation.z = float(self.world.brick.z)
        self.brick.header.stamp = self.world_to_brick.header.stamp

    def timer_callback(self):
        try:

            #self.get_logger().info("another test")
            # get the latest transform between left and right
            # (rclpy.time.Time() means get the latest information)
            #self.get_logger().info(f'in timer callback...  self.falling: {self.falling}')

            self.platform_to_brick = self.buffer.lookup_transform('platform','brick',rclpy.time.Time().to_msg())
            self.world_to_platform = self.buffer.lookup_transform('world','platform',rclpy.time.Time().to_msg())
            #self.base_to_odom = self.buffer.lookup_transform('base','odom',rclpy.time.Time().to_msg())
            self.get_logger().info('l1093413')
            #elif self.state == State.FALLING and self.curr_height > 0.1:
                #self.state = State.WAITING
            #elif self.curr_height == 0.1:
                #self.state = State.RETURN
            if self.state == State.NOACTION:
                self.get_logger().info("No action")
                pass
            elif self.state == State.FALLING:
                #self.get_logger().info("t1")
                if (self.platform_to_brick.transform.translation.x < 0.3
                and self.platform_to_brick.transform.translation.y < 0.3
                and self.platform_to_brick.transform.translation.z < 0.5):
                    #self.get_logger().info("t2")
                    self.state = State.RETURN
                elif self.platform_to_brick.transform.translation.z < -self.platform_height:
                    self.state = State.NOACTION
                self.get_logger().info("Falling")
                    #self.get_logger().info("t3")
                self.world.drop()
            elif self.state == State.RETURN:
                self.world_to_brick.transform.translation.z = self.world_to_platform.transform.translation.z + 0.1
                self.world.brick.z = self.world_to_brick.transform.translation.z
                self.world_to_brick.transform.translation.x = (
                    self.world_to_platform.transform.translation.x + self.platform_to_brick.transform.translation.x
                )
                self.world.brick.x = self.world_to_brick.transform.translation.x
                self.world_to_brick.transform.translation.y = (
                    self.world_to_platform.transform.translation.y + self.platform_to_brick.transform.translation.y
                )            
                self.world.brick.y = self.world_to_brick.transform.translation.y
                if (
                5.445+0.5 > self.world_to_platform.transform.translation.x >= 5.445
                and 5.445+0.5 > self.world_to_platform.transform.translation.y >= 5.445):
                    self.state = State.DEPOSIT
                self.get_logger().info("Return")
            elif self.state == State.DEPOSIT:
                self.slide_vel += abs(self.gravity_accel*self.dt)
                if self.world.brick.y < self.world_to_platform.transform.translation.y + 1.0:
                    self.world.brick.z -= self.slide_vel * math.sin(0.7854)
                    self.world.brick.y += self.slide_vel * math.cos(0.7854)
                else:
                    pass
                self.get_logger().info("desposit")
                pass
            self.world_to_platform.header.stamp = self.get_clock().now().to_msg()
            #self.base_to_odom.header.stamp = self.get_clock.now().to_msg()
            self.platform_to_brick.header.stamp = self.get_clock().now().to_msg()
            

        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f'Connectivity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f'Extrapolation exception: {e}')
        finally:
            self.get_logger().info("in finally")
        self.world_to_brick.header.stamp = self.get_clock().now().to_msg()
        self.brick_tf_broadcaster.sendTransform(self.world_to_brick)
        
        self.update_brick_marker()
        self.brick_pub.publish(self.brick)
        

def arena_entry(args=None):
    '''main function'''
    rclpy.init(args=args)
    node = arena()
    rclpy.spin(node)
    rclpy.shutdown()

