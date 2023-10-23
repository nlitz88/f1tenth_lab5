#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
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
    """

    def __init__(self):
        super().__init__("pose_publisher")

        # Set up node parameters.
        self.declare_parameters(namespace=self.get_namespace(), 
                                parameters=[
                                    ("source_frame", "base_link"),
                                    ("target_frame", "map"),
                                    ("publishing_frequency", 100)
                                ])
        # Create local copies of the parameters.
        # TODO: Create a paramater update callback function and data structure
        # to get these parameters.
        self.__source_frame = self.get_parameter(f"{self.get_namespace()}.source_frame").value
        self.__target_frame = self.get_parameter(f"{self.get_namespace()}.target_frame").value

        # Create a transform listener to listen for transform messages.
        # These will be used to determine the car's localized pose.
        self.__transform_buffer = Buffer()
        self.__transform_listener = TransformListener(buffer=self.__transform_buffer, node=self)

        # Create a timer to kick off getting the pose and publishing it.
        self.__pose_timer = self.create_timer(timer_period_sec=1/100.0, 
                                              callback=self.__pose_timer_callback)
        
        self.__odom_subscriber = self.create_subscription(msg_type=)
        # Create publisher we'll use to actually publish the pose obtained from
        # the transforms.
        self.__pose_publisher = self.create_publisher(msg_type=PoseStamped,
                                                      topic=f"/{self.get_namespace()}/pose",
                                                      qos_profile=10)

    def __get_car_pose(self) -> PoseStamped:
        # TODO: Update to also include covariance!!! Not sure how to do this
        # yet. May just revert back to using regular pose if I run out of time.
        # I feel like I'd have to also subscribe to the car's odom topic, as
        # that's where the covariance is going to come from.
        """Gets the car frame's (base_link frame's) pose in the map frame.

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
            transform: TransformStamped = self.__transform_buffer.lookup_transform(target_frame=self.__target_frame,
                                                                                   source_frame=self.__source_frame,
                                                                                   time=Time())
        except Exception as exc:
            self.get_logger().warning(f"Failed to obtain transformation from {self.__map_frame} frame to {self.__car_frame} frame needed to determine car pose.")
            raise exc
        # Now, create PoseStamped object from transform.
        new_pose = PoseStamped()
        new_pose.header.frame_id = transform.header.frame_id
        new_pose.header.stamp = self.get_clock().now().to_msg()
        # Extract position from transform.
        transform.transform.
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
    
    def __pose_timer_callback(self):
        
        
        pass
        
        

def main(args = None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()