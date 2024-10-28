from geometry_msgs.msg import TransformStamped, Point
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
from turtle_brick.physics import World
from turtle_brick_interfaces.srv import PlacePoint
from tf2_ros.transform_broadcaster import TransformBroadcaster


class arena(Node):
    def __init__(self): 
        super().__init__('arena')
        self.declare_parameter('max_velocity', 2.0)
        self.declare_parameter('gravity_accel', 9.81)
        self.declare_parameter('wheel_radius',0.5)
        self.max_velocity = self.get_parameter('max_velocity').value
        self.gravity_accel = self.get_parameter('gravity_accel').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.brick_initialization = Point()
        self.brick_initialization.x = 2.0
        self.brick_initialization.y = 2.0
        self.brick_initialization.z = 6.0
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
        self.falling = True
        return response
    
    def update_brick_marker(self):
        self.world_to_brick.header.stamp = self.get_clock().now().to_msg()
        self.world_to_brick.transform.translation.x = float(self.world.brick.x)
        self.world_to_brick.transform.translation.y = float(self.world.brick.y)
        self.world_to_brick.transform.translation.z = float(self.world.brick.z)
        self.brick.header.stamp = self.world_to_brick.header.stamp

    def timer_callback(self):
        if self.falling:
            self.world.drop()
            self.update_brick_marker()
        self.brick_tf_broadcaster.sendTransform(self.world_to_brick)
        self.brick_pub.publish(self.brick)
        

def arena_entry(args=None):
    '''main function'''
    rclpy.init(args=args)
    node = arena()
    rclpy.spin(node)
    rclpy.shutdown()

