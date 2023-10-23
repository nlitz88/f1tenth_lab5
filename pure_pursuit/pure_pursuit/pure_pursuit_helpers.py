from typing import Tuple
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
    return np.array([[float(pose.position.x), float(pose.position.y)] for pose in path.poses])

def euclidean_distance(vector_a: np.ndarray, vector_b: np.ndarray) -> float:
    """Tiny helper function to compute euclidean distance using numpy.
    Fast method found on
    https://stackoverflow.com/questions/1401712/how-can-the-euclidean-distance-be-calculated-with-numpy
    
    Args:
        row (np.ndarray): numpy array row.

    Returns:
        float: The euclidean distance between vector_a and vector_b.
    """
    return np.linalg.norm(vector_a-vector_b)

def get_distance_to_each_point(current_position: Tuple[float, float], 
                               numpy_path: np.ndarray) -> np.ndarray:
    """Given your current 2D position in a given frame of reference and a numpy
    array of 2D positions in that same frame that describe a path (a sequence of
    waypoints), this function computes the distance from your current position
    to EACH of the nodes in the provided path. Returns a numpy array with as
    many distances as there are points (rows) in the input array numpy_path.
    I.e., the ith distance in the returned array will be the euclidean distance
    from the provided current_position to the ith waypoint in the input
    numpy_path array.

    Args:
        current_position (np.ndarray): Your current position (x,y) in a
        given frame as a 1D ndarray.
        numpy_path (np.ndarray): Array of 2D waypoints comprising a path in the
        same frame as the current_position.

    Returns:
        np.ndarray: The array of distances from your current position to each of
        the positions/waypoints in the provided numpy_path.
    """
    return np.apply_along_axis(euclidean_distance, 1, numpy_path, current_position)

def subtract_lookahead_distance(lookahead_distance_m: float, distances_m: np.ndarray) -> np.ndarray:
    """Subtracts the lookahead distance from each element in distances. I.e.,
    just a simple wrapper for element-wise subtraction.

    Args:
        lookahead_distance_m (float): The distance to be subtracted from each
        distance in the provided distances array.
        distances_m (np.ndarray): The array of distances.

    Returns:
        np.ndarray: The "normalized" distances. I.e., the distances with the
        lookahead distance subtracted from each.
    """
    return np.subtract(distances_m, lookahead_distance_m)


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