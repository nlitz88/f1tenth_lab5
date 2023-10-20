#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose

import csv

class PathFilePublisher(Node):

    def __init__(self, filepath: str) -> None:
        super().__init__(node_name="path_file_publisher")

        # Before anything else: attempt to load the path from the file at the
        # specified filepath. If able to load the file, create a "Path" object
        # from the sequence of points.
        try:
            with open(filepath, 'r') as path_file:
                csv_reader = csv.reader(path_file)
                path = Path()
                for row in csv_reader:
                    point = Point(x=row[0], y=row[1])
                    pose = Pose(position=point)
                    path.poses.append(pose)
        except OSError as exc:
            self.get_logger().error(f"Failed to open path file {filepath}")
            raise exc
        self.get_logger().info(f"Successfully created new Path message from {len(path.poses)} waypoints found in path file {filepath}")
        
        # Create a timer that will generate interrupts to trigger the publisher.

        # Create publisher. Will publish at a fixed interval. Additionally, will
        # latch it so that any time a node subscribes, it'll get the latest path
        # published.

        # NOTE: technically, because this is being read in from a file, it
        # technically should never need to be republished. However, to simulate
        # a path planner periodically being run to create a new path, this node
        # also publishes the same path periodically.

    def path_from_file

def main(*args, **kwargs):
    pass

if __name__ == "__main__":
    main()