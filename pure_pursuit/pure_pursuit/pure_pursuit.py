from threading import Lock
import rclpy
from rclpy.node import Node
from rclpy.time import Time

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

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
        self.declare_parameters(namespace=self.get_namespace(),
                                parameters=[
                                    ("lookahead_distance_m", rclpy.Parameter.Type.DOUBLE),
                                    ("max_longitudinal_velocity_ms", rclpy.Parameter.Type.DOUBLE),
                                    ("min_longitudinal_velocity_ms", rclpy.Parameter.Type.DOUBLE),
                                    ("controller_frequency_hz", 100) 
                                ])
        # Grab values of parameters for local use after being declared.
        # TODO: Need to set up a parameter update callback function.
        self.__lookahead_distance_m = self.get_parameter("lookahead_distance_m").value
        self.__max_longitudinal_velocity_ms = self.get_parameter("max_longitudinal_velocity_ms").value
        self.__min_longitudinal_velocity_ms = self.get_parameter("min_longitudinal_velocity_ms").value
        self.__controller_frequency = self.get_parameter("controller_frequency").value

        # Set up timer for controlling how frequently pure pursuit commands new
        # control values.
        self.__control_timer = self.create_timer(timer_period_sec=1.0/float(self.__controller_frequency),
                                                 callback=self.__control_callback)
        
        # Create a subscriber for the vehicle's pose. 
        # TODO: Not sure if pure pursuit needs to recalculate its values every
        # time it receives a new pose message--may be able to run this at a
        # lower frequency. In which case, I think this subscriber would just
        # update a synchronized variable that we maintain the pose in, and then
        # the timer is what invokes the actual pure pursuit control logic.
        self.__pose_subscriber = self.create_subscription(msg_type=PoseWithCovarianceStamped,
                                                          topic="/pose",
                                                          callback=self.__pose_callback, 
                                                          qos_profile=10)
        # Instance variable and accompanying mutex to cache most recently
        # obtained pose.
        self.__pose: PoseWithCovarianceStamped = None
        self.__pose_lock = Lock()

        # Create a drive publisher to command the resulting drive values.
        # TODO: May have to update this topic (and a number of other things)
        # depending on how these namespaces work.
        self.__drive_publisher = self.create_publisher(msg_type=AckermannDriveStamped,
                                                       topic=f"/drive",
                                                       qos_profile=10)

    def get_next_target_point(self, 
                              current_pose: PoseWithCovarianceStamped, 
                              path: Path) -> Pose:
        """Function that will take the robot's current pose in the map frame and
        determine what the next target point should be.

        Specifically, this implementation looks at which point in the path array
        is closest the lookahead distance away from the vehicle sequentially
        after the point that is currently closest to the car.

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

        # TODO: find the current waypoint to track using methods mentioned in lecture

        # TODO: transform goal point to vehicle frame of reference

        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle.

        pass

    def __pose_callback(self, new_pose: PoseWithCovarianceStamped) -> None:

        # Store the most recent pose.
        with self.__pose_lock:
            self.__pose = new_pose
        self.get_logger().debug("Received new pose")

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()