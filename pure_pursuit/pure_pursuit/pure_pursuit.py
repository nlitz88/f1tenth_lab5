import rclpy
from rclpy.node import Node
from rclpy.time import Time

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__("pure_pursuit")
        
        

        # TODO: Create transform listener(s). Need one to listen for the
        # transform from odom to base_link, and then one from map to odom.
        # Create those transform listeners here. NOTE: There doesn't seem to be
        # an odom frame in simulation, so may have to have a different node for
        # the physical robot? OR, could just use some logic from the parameters
        # that the node is set up with to configure how it is set up.

        # NOTE: Maybe for the simulation, the bridge node's /ego_racecar/pose
        # messages simulate the pose that we'd get by using a particle filter
        # and computing the corrected pose using the map-->odom +
        # odom-->base_link transforms manually. SO, if that's true, for
        # simulation, only need to subscribe to the /ego_racecar/pose topic.

        # WELL, maybe rather than subscribing to the pose message, we can use a
        # function "get_pose" that 
        # a.) For simulation, just subscribes to whatever topic publishes the
        # pose.
        # b.) for the real robot, just 
        
        # For simulation (and the real robot as well), I think the TF library
        # completely simplifies this whole process. Why? Because we are just
        # asking it to "give us the transform from the map frame to the base
        # link frame." We don't care if there's an odom frame between those two
        # frames or not--that's up to TF2 to know what the transform tree looks
        # like!

        # Therefore, we can just ask for that transform in all contexts, and TF
        # will figure out how to get us there.

        # AND, in this specific case, asking for the transform from map to base
        # link means we'll be taking the odometry generated odom-->base_link
        # transformation (the "continuous" (high frequency)) odometry estimated
        # position and adding the localization-determined offset to its
        # accurate/real position. This gives us that continuously updated,
        # smooth, yet periodically corrected position that we want. This is our
        # pose!
        #

        # TODO: Also, though--just keep in mind that it's possible that the
        # f1tenth car packages contain nodes that already take care of this
        # transformation and just give us the pose on that pose topic--so be
        # prepared to convert that function to just grab it from a topic that's
        # already out there.

        # Set up node parameters.
        self.declare_parameters(namespace="",
                                parameters=[
                                    ("lookahead_distance_m", rclpy.Parameter.Type.DOUBLE),
                                    ("max_longitudinal_velocity_ms", rclpy.Parameter.Type.DOUBLE),
                                    ("min_longitudinal_velocity_ms", rclpy.Parameter.Type.DOUBLE),
                                    ("map_frame", "map"),
                                    ("car_frame", "base_link")
                                ])

        # Create local copies of the parameters.
        # TODO: Create a paramater update callback function and data structure
        # to get these parameters.
        self.__map_frame = self.get_parameter("map_frame").value
        self.__car_frame = self.get_parameter("car_frame").value
        
        # First, create a transform listener to listen for transform messages.
        # These will be used to determine the car's localized pose.
        self.__transform_buffer = Buffer()
        self.__transform_listener = TransformListener(buffer=self.__transform_buffer, node=self)

        # Temporary pose publisher.
        self.__control_timer = self.create_timer(timer_period_sec=0.1, callback=self.__control_callback)
        # NOTE: Not sure if this is really appropriate--is it this node's
        # responsibility to be publishing the car's pose? Doesn't feel like it.
        self.__pose_publisher = self.create_publisher(msg_type=PoseStamped, topic="/ego_racecar/pose", qos_profile=10)

    def __get_car_pose(self) -> PoseStamped:
        """Gets the car's pose in the map frame.

        Raises:
            exc: Raises Exception if it fails to obtain the transformations that
            are needed to compute the pose.

        Returns:
            PoseStamped: Timestamped Pose of the car.
        """

        # First, try to obtain transform from map frame to car frame. This is
        # effectively the bose of the robot, as the robot start's at the origin
        # of the map frame.
        try:
            transform: TransformStamped = self.__transform_buffer.lookup_transform(target_frame=self.__map_frame,
                                                                                   source_frame=self.__car_frame,
                                                                                   time=Time())
        except Exception as exc:
            self.get_logger().warning(f"Failed to obtain transformation from {self.__map_frame} frame to {self.__car_frame} frame needed to determine car pose.")
            raise exc
        # Now, create PoseStamped object from transform.
        new_pose = PoseStamped()
        new_pose.header.frame_id = transform.header.frame_id
        new_pose.header.stamp = self.get_clock().now().to_msg()
        # Extract position from transform.
        new_pose.pose.position.x = transform.transform.translation.x
        new_pose.pose.position.y = transform.transform.translation.y
        new_pose.pose.position.z = transform.transform.translation.z
        # Extract rotation from transform.
        new_pose.pose.orientation.x = transform.transform.rotation.x
        new_pose.pose.orientation.y = transform.transform.rotation.y
        new_pose.pose.orientation.z = transform.transform.rotation.z
        new_pose.pose.orientation.w = transform.transform.rotation.w
        # BEFORE MOVING AHEAD WITH THIS, TRY TO VERIFY THE LOGIC INTUITIVELY.
        return new_pose

    def get_next_target_point(self, current_pose: PoseStamped, path: Path) -> Pose:
        """Function that will take the robot's current pose in the map frame and
        determine what the next target point should be.

        Specifically, this implementation looks at which point in the path array
        is closest the lookahead distance away from the vehicle.

        Args:
            current_pose (PoseStamped): The robot's current pose in the map
            frame / w.r.t. the map frame.
            path (Path): A sequence of robot poses, each with respect to the map
            frame as well.

        Returns:
            Pose: The point in the path chosen as the next target point.
        """
        # Create a vectorized version of the (x,y) positions in the path using
        # numpy.

        # Apply a euclidean distance elementwise across the vector of (x,y) path
        # positions.

        # Find the point that is closest to the current pose. We'll start our
        # search for the point closest to the lookahead distance from this
        # point.
        
        pass

    def transform_target_to_vehicle_frame(self, target_point):
        pass
        # Maybe this function would subscribe to tf? Need to identify the normal
        # way to accomplish this part. Do you subscribe asynchronously? Or do
        # you get it as a service?
        # Just get it via doing a lookup on the transform buffer--this function
        # abstracts away all the computation for you.

    def get_steering_angle_to_target(self, target_point_vframe):
        pass

    def get_current_velocity(self, vehicle_position_in_map):
        pass

    def publish_drive_message(self, velocity, steering_angle):
        pass

    def __control_callback(self) -> None:

        # Attempt to get/compute current pose based on the most recently
        # received transforms.

        # Attempt to publish the pose (is this necessary?) 
        # TODO: Question: is it commonplace to have a separate node that takes
        # the transform and spits out a PoseStamped message? or is just expected
        # that each node should just be computing this on its own.
        # However, then what about visualizations?

        # With the pose, call the function to compute the next target waypoint
        # using the obtained pose.
        try:
            car_pose = self.__get_car_pose()
        except Exception as exc:
            self.get_logger().warning(f"Failed to obtain car's pose from transform")
        else:
            self.__pose_publisher.publish(car_pose)
            self.get_logger().info(f"Successfully published car's pose.")

        # 
        pass

    def pose_callback(self, pose_msg):
        pass
        # TODO: find the current waypoint to track using methods mentioned in lecture

        # TODO: transform goal point to vehicle frame of reference

        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle.

    # Are we expected to run this controller on a timer? As, there isn't really
    # a particular topic that we'd have it subscribed to--unless there was a
    # separate node that was solely responsible for computing the pose.


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()