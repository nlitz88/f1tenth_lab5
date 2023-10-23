from threading import Lock
import rclpy
from rclpy.node import Node
from rclpy.time import Time

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from copy import deepcopy



class PurePursuit(Node):
    """ 
    Node implementing the pure pursuit controller.
    """
    
    def __init__(self):
        super().__init__("pure_pursuit")

        # Set up node parameters.
        self.declare_parameters(namespace="",
                                parameters=[
                                    ("lookahead_distance_m", rclpy.Parameter.Type.DOUBLE),
                                    ("max_longitudinal_velocity_ms", rclpy.Parameter.Type.DOUBLE),
                                    ("min_longitudinal_velocity_ms", rclpy.Parameter.Type.DOUBLE),
                                    ("controller_frequency_hz", 100)
                                    # MAY HAVE TO ADD PARAMETERS FOR CAR_FRAME
                                    # AND MAP_FRAME or something like that here
                                    # for the transformation in step 2.
                                ])
        # Grab values of parameters for local use after being declared.
        # TODO: Need to set up a parameter update callback function.
        self.__lookahead_distance_m = self.get_parameter("lookahead_distance_m").value
        self.__max_longitudinal_velocity_ms = self.get_parameter("max_longitudinal_velocity_ms").value
        self.__min_longitudinal_velocity_ms = self.get_parameter("min_longitudinal_velocity_ms").value
        self.__controller_frequency = self.get_parameter("controller_frequency_hz").value

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
        # TODO: Need to look into how this is done conventionally, but I would
        # imagine that it'd be best for this subscriber to be running in its own
        # thread (if it's not already). Look into threading options if that's
        # not enabled by default.
        self.__pose_subscriber = self.create_subscription(msg_type=PoseWithCovarianceStamped,
                                                          topic="pose",
                                                          callback=self.__pose_callback, 
                                                          qos_profile=10)
        # Instance variable and accompanying mutex to cache most recently
        # obtained pose.
        self.__pose: PoseWithCovarianceStamped = None
        self.__pose_lock = Lock()

        # Create a subscriber for the path as well.
        self.__path_subscriber = self.create_subscription(msg_type=Path,
                                                          topic="path",
                                                          callback=self.__path_callback,
                                                          qos_profile=10)
        self.__path: Path = None
        self.__path_lock = Lock()

        # Create a drive publisher to command the resulting drive values.
        # TODO: May have to update this topic (and a number of other things)
        # depending on how these namespaces work.
        self.__drive_publisher = self.create_publisher(msg_type=AckermannDriveStamped,
                                                       topic=f"drive",
                                                       qos_profile=10)
        
        # Create a transform listener to listen for transform messages.
        # These will be used to transform points in the map frame to the
        # car's reference frame (base_link).
        self.__transform_buffer = Buffer()
        self.__transform_listener = TransformListener(buffer=self.__transform_buffer, node=self)

    def __get_next_target_point(self, 
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
        poses = np.array(path.poses)

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

        # TODO: find the current waypoint to track using methods mentioned in
        # lecture.
        # Before anything, grab the most recent copy of the path and pose.
        # TODO: Is there a better way of doing this? Would it be better to hold
        # both locks through the duration of this callback? Or just grab a copy
        # like this?
        # NOTE: If this node isn't multithreaded by default, then there's not
        # really a need for this--as there's only one thread and it
        with self.__path_lock: 
            newest_path = deepcopy(self.__path)
        with self.__pose_lock:
            newest_pose = deepcopy(self.__pose)

        target_point = self.__get_next_target_point(current_pose=newest_pose,
                                                    path=newest_path)

        # TODO: transform goal point to vehicle frame of reference


        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle.

        pass

    def __pose_callback(self, new_pose: PoseWithCovarianceStamped) -> None:
        """Callback to store the most recently received pose message.
        
        Args:
            new_pose (PoseWithCovarianceStamped): Pose received by pose
            subscriber.
        """
        with self.__pose_lock:
            self.__pose = new_pose

    def __path_callback(self, new_path: Path) -> None:
        """Callback to store the most recently received path message.

        Args:
            new_path (Path): Path received by path subscriber.
        """
        with self.__path_lock:
            self.__path = new_path

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()