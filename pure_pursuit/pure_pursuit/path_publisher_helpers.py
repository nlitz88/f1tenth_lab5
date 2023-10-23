import pathlib
import csv
from typing import Optional

from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped

def generate_path_from_file(filepath: str, frame_id: Optional[str] = "map") -> Path:
    """Generates a Path object containing a list of poses derived from the file
    at the provided filepath.

    Args:
        filepath (str): Filesystme filepath to the csv file containing the 2D
        waypoints.
        frame_id (Optional[str], optional): The id of the frame that you wish
        for the poses within the path to be with respect to/in. Defaults to
        "map".

    Returns:
        Path: The collection of waypoints in the specified frame wrapped up in a
        Path object.
    """
    with open(filepath, 'r') as path_file:
        csv_reader = csv.reader(path_file)
        path = Path()
        for row in csv_reader:
            if len(row) == 2:
                point = Point(x=float(row[0]), y=float(row[1]))
                pose = Pose(position=point)
                pose_stamped = PoseStamped(pose=pose)
                pose_stamped.header.frame_id = "map"
                path.poses.append(pose_stamped)
                path.header.frame_id = "map"
    return path