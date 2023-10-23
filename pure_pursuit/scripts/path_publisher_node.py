#!/usr/bin/env python3

from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped

import csv

class PathFilePublisher2D(Node):

    def __init__(self, filepath: Optional[str] = None) -> None:
        super().__init__(node_name="path_file_publisher")
        
        # Initialize Parameters.
        # NOTE: This call to declare parameters will take any parameter values
        # provided alongside executing this node. I.e., in a launch file, in a
        # params file, as cli args in the ros run command, etc.
        self.declare_parameters(
            namespace="",
            parameters=[
                ("filepath", filepath)
            ],
            ignore_override=False
        )
        self.__filepath = self.get_parameter("filepath").value

        # Before anything else: attempt to load the path from the file at the
        # specified filepath. If able to load the file, create a "Path" object
        # from the sequence of points.
        try:
            with open(self.__filepath, 'r') as path_file:
                csv_reader = csv.reader(path_file)
                self.__path = Path()
                for row in csv_reader:
                    if len(row) == 2:
                        point = Point(x=float(row[0]), y=float(row[1]))
                        pose = Pose(position=point)
                        pose_stamped = PoseStamped(pose=pose)
                        pose_stamped.header.frame_id = "map"
                        self.__path.poses.append(pose_stamped)
                        self.__path.header.frame_id = "map"
        except OSError as exc:
            self.get_logger().error(f"Failed to open path file {self.__filepath}")
            raise exc
        self.get_logger().info(f"Successfully created new Path message from {len(self.__path.poses)} waypoints found in path file {self.__filepath}")
        
        # Create publisher. Will publish at a fixed interval. Additionally, will
        # latch it so that any time a node subscribes, it'll get the latest path
        # published.
        self.__path_publisher = self.create_publisher(msg_type=Path, 
                                                      topic="path", 
                                                      qos_profile=10)

        # Create a timer that will generate interrupts to trigger the publisher.
        self.create_timer(timer_period_sec=0.1, 
                          callback=self.__publish_path)

        # NOTE: technically, because this is being read in from a file, it
        # technically should never need to be republished. However, to simulate
        # a path planner periodically being run to create a new path, this node
        # also publishes the same path periodically.

    def __publish_path(self):
        """Publishes the internal copy of the path loaded from file earlier.
        """
        self.__path_publisher.publish(self.__path)

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathFilePublisher2D()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()