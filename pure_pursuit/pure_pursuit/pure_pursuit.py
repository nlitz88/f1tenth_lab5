from threading import Lock
import rclpy
from rclpy.node import Node
from rclpy.time import Time

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from copy import deepcopy

from pure_pursuit.pure_pursuit_helpers import get_next_target_point, compute_steering_angle

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
                                    ("controller_frequency_hz", 100),
                                    ("car_frame", "base_link"),
                                    ("map_frame", "map")
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
        self.__car_frame = self.get_parameter("car_frame").value
        self.__map_frame = self.get_parameter("map_frame").value

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

        # Create publisher for pure pursuit's current target_pose.
        self.__target_point_publisher = self.create_publisher(msg_type=PointStamped, 
                                                              topic="target_point",
                                                              qos_profile=10)

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

    def __publish_target_pose(self, target_pose: PoseStamped) -> None:
        """Publish the target pose as a PointStamped via the
        target_point_publisher.

        Args:
            target_pose (PoseStamped): The pose that the 2D PointStamped will be
            derived from.
        """
        target_point = PointStamped()
        target_point.header.frame_id = self.__map_frame
        target_point.header.stamp = self.get_clock().now().to_msg()
        target_point.point.x = target_pose.pose.position.x
        target_point.point.y = target_pose.pose.position.y
        self.__target_point_publisher.publish(target_point)

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
        
        if newest_path is None or newest_pose is None:
            return
        # Determine what the next target point should be based on the car's
        # current pose relative to the map and the path in the map.
        # NOTE: The desired path waypoint Pose is returned, but for now, we're
        # only really interested in the (x,y) position inside.
        target_pose = get_next_target_point(current_pose=newest_pose, 
                                            path=newest_path,
                                            lookahead_distance_m=self.__lookahead_distance_m)
        # TODO: NOTE: MAY HAVE TO UPDATE TIMESTAMP OF TARGET POSE so that we get
        # the transform of the pose to the base_link at the time this callback
        # was called?
        # Publish position Point derived from target_pose to this node's
        # target_point publisher.
        self.__publish_target_pose(target_pose=target_pose)

        # TODO: transform goal point to vehicle frame of reference
        # NOTE: Okay, it doesn't look like the transform function works out of
        # the box for the latest on foxy. However, I think the functions to
        # actually perform the transform DO exist.
        # May be able to use those underlying functions--but I have a feeling
        # I'll need to manually install a newer version of pykdl or something
        # like that.
        # TODO: Work on this after the peer review. This is in the critical
        # path. Until I'm done preparing for the peer review, write and unit
        # test the function that computes the steering angle once we have that
        # transformed point.
        target_pose_in_car_frame: PoseStamped = self.__transform_buffer.transform(object_stamped=target_pose,
                                                                     target_frame=self.__car_frame)


        # TODO: calculate curvature/steering angle
        steering_angle = compute_steering_angle(target_point_y=target_pose_in_car_frame.pose.position.y)


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