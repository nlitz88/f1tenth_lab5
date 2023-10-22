#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class PosePublisher(Node):
    """Node solely dedicated to publishing the pose of the origin of the
    provided target frame with respect to / from the perspective of the provided
    source frame. Uses the TF2 library under the hood to grab the transform,
    which ends up just being the pose.
    """

    def __init__(self):
        super().__init__("pose_publisher", )

        # Set up node parameters.
        

def main(args = None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()


if __name__ == "__main__":
    main()