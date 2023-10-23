import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped


def numpy_array_from_path(path: Path) -> np.ndarray:
    """Takes a Path object and returns a 2D numpy array 

    Args:
        path (Path): _description_

    Returns:
        np.ndarray: _description_
    """
    return np.array([[pose.position.x, pose.position.y] for pose in path.poses])


# NOTE: This function is like the integration of each step / smaller function.
# So, if we can test each function separately and independently verify each
# piece, then it becomes much easier to bring those components together and
# integration test them.

def get_next_target_point(current_pose: PoseWithCovarianceStamped, 
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