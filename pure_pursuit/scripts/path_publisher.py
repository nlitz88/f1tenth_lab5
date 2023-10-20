#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose

import csv

class PathFilePublisher2D(Node):

    def __init__(self, filepath: str) -> None:
        super().__init__(node_name="path_file_publisher")

        # Before anything else: attempt to load the path from the file at the
        # specified filepath. If able to load the file, create a "Path" object
        # from the sequence of points.
        try:
            with open(filepath, 'r') as path_file:
                csv_reader = csv.reader(path_file)
                self.__path = Path()
                for row in csv_reader:
                    point = Point(x=row[0], y=row[1])
                    pose = Pose(position=point)
                    self.__path.poses.append(pose)
        except OSError as exc:
            self.get_logger().error(f"Failed to open path file {filepath}")
            raise exc
        self.get_logger().info(f"Successfully created new Path message from {len(self.__path.poses)} waypoints found in path file {filepath}")
        
        # Create a timer that will generate interrupts to trigger the publisher.
        self.create_timer(timer_period_sec=Duration(seconds=0.1), 
                          callback=self.__publish_path)

        # Create publisher. Will publish at a fixed interval. Additionally, will
        # latch it so that any time a node subscribes, it'll get the latest path
        # published.
        self.__path_publisher = self.create_publisher(msg_type=Path, 
                                                      topic="/path_from_file", 
                                                      qos_profile=10)

        # NOTE: technically, because this is being read in from a file, it
        # technically should never need to be republished. However, to simulate
        # a path planner periodically being run to create a new path, this node
        # also publishes the same path periodically.

    def __publish_path(self):
        """Publishes the internal copy of the path loaded from file earlier.
        """
        self.__path_publisher.publish(self.__path)

def main(*args, **kwargs):
    rclpy.init(args=args)
    path_publisher = PathFilePublisher2D(filepath="rats")
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()