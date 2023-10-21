import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__("pure_pursuit")
        
        # Set up node parameters.
        self.declare_parameters(namespace="",
                                parameters=[
                                    ("lookahead_distance_m", rclpy.Parameter.Type.DOUBLE),
                                    ("max_longitudinal_velocity_ms", rclpy.Parameter.Type.DOUBLE),
                                    ("min_longitudinal_velocity_ms", rclpy.Parameter.Type.DOUBLE)
                                ])

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
        # smooth, yet periodically corrected position that we want. This is our pose!

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

    def get_steering_angle_to_target(self, target_point_vframe):
        pass

    def get_current_velocity(self, vehicle_position_in_map):
        pass

    def publish_drive_message(self, velocity, steering_angle):
        pass

    def pose_callback(self, pose_msg):
        pass
        # TODO: find the current waypoint to track using methods mentioned in lecture

        # TODO: transform goal point to vehicle frame of reference

        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle.

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()