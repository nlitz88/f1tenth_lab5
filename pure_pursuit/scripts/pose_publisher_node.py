#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

class PosePublisher(Node):
    """Node solely dedicated to publishing the pose (position, orientation) of
    the source frame's origin (with respect to / from the perspective of / in
    terms of) the target frame.

    The PosePublisher derives the desired pose from two key parameters: The
    source_frame and the target_frame.

    source_frame (str): The frame that you wish to get the pose of from the
        perspective of the target frame. For instance, if you wanted to know the
        pose of your vehicle in a map frame, then the source frame would be your
        vehicle's frame (like base_link).

    target_frame (str): The frame that you wish to represent the pose of the
        source frame in terms of. In the above example, the map would be the
        target frame.

    The pose publisher subscribes to the odom topic in the node's namespace by
    default and publishes to the pose topic within the node's namespace by
    default.
    """

    def __init__(self):
        super().__init__("car_pose_publisher")

        # Set up node parameters.
        self.declare_parameters(namespace="", 
                                parameters=[
                                    ("source_frame", "base_link"),
                                    ("target_frame", "map"),
                                ])
        # Create local copies of the parameters.
        # TODO: Create a paramater update callback function and data structure
        # to get these parameters.
        self.__source_frame = self.get_parameter("source_frame").value
        self.__target_frame = self.get_parameter("target_frame").value

        # Create a transform listener to listen for transform messages.
        # These will be used to determine the car's localized pose.
        self.__transform_buffer = Buffer()
        self.__transform_listener = TransformListener(buffer=self.__transform_buffer, node=self)

        # Create subscriber for odometry messages.
        # NOTE: Take note of how I'm NOT using the absolute topic name /odom,
        # but instead a relative topic name "odom". This allows for the topic to
        # be "resolved" within this node's namespace automatically.
        self.__odom_subscriber = self.create_subscription(msg_type=Odometry, 
                                                          topic=f"odom",
                                                          callback=self.__odom_callback,
                                                          qos_profile=10)
        # Create publisher we'll use to actually publish the pose obtained from
        # the transforms.
        self.__pose_publisher = self.create_publisher(msg_type=PoseWithCovarianceStamped,
                                                      topic=f"pose",
                                                      qos_profile=10)
    
    def __odom_callback(self, odom_message: Odometry) -> None:
        # NOTE: Considering going this route where I try to publish at about the
        # same speed as geometry. Basically, doing it this way would allow me to
        # also include Covariance from the Odometry in a
        # PoseWithCovarianceStamped message.

        # Let's write out this function in steps first, and if I see an obvious,
        # intuitive way to modularize, go for that. Otherwise, it'll stay like
        # this until it needs refactored. Can't spend more time on this right
        # now.

        # 1. Try to get the transform from the source frame to the target frame.
        #    Return if an exception is thrown and the transformation can't be
        #    obtained.
        try:
            transform: TransformStamped = self.__transform_buffer.lookup_transform(target_frame=self.__target_frame,
                                                                                   source_frame=self.__source_frame,
                                                                                   time=Time())
        except Exception as exc:
            self.get_logger().warning(f"Failed to obtain transformation from {self.__source_frame} frame to {self.__target_frame} frame needed to determine pose.\n{str(exc)}")
            return
        else:
            self.get_logger().debug(f"Successfully obtained transformation from {self.__source_frame} frame to {self.__target_frame} frame needed to determine pose.")
        # 2. If transform successfully obtained, build up the new
        #    PoseWithCovarianceStamped message.
        new_pose = PoseWithCovarianceStamped()
        # new_pose.header.frame_id = transform.header.frame_id # TODO: Shouldn't this be the child_frame_id of the transform? As the header frame id == id of frame that you're transforming from?
        # Interpret position from transform.
        new_pose.header.frame_id = transform.child_frame_id
        new_pose.header.stamp = odom_message.header.stamp
        new_pose.pose.pose.position.x = transform.transform.translation.x
        new_pose.pose.pose.position.y = transform.transform.translation.y
        new_pose.pose.pose.position.z = transform.transform.translation.z
        # Interpret rotation from transform.
        new_pose.pose.pose.orientation.x = transform.transform.rotation.x
        new_pose.pose.pose.orientation.y = transform.transform.rotation.y
        new_pose.pose.pose.orientation.z = transform.transform.rotation.z
        new_pose.pose.pose.orientation.w = transform.transform.rotation.w
        # Get covariance from odometry and assign that here.
        new_pose.pose.covariance = odom_message.pose.covariance
        # Publish new pose.
        self.__pose_publisher.publish(new_pose) 

def main(args = None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()