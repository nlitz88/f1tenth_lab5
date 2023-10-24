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

def find_smallest_past_index(distances_m: np.ndarray, starting_index: int) -> int:
    """Returns the index of the smallest element in the array of distances
    AFTER the starting index. I.e., not including the starting index.

    Args:
        distances_m (np.ndarray): Array of distance values.
        after_index (int): Index that search for smallest will begin after.

    Returns:
        int: Index of the smallest element that comes after the starting_index.
    """
    return np.argmin()


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

    # NOTE: Could stop here. In the simplest implementation of pure pursuit
    # (I.e., I think the approach taken in the original paper), the closest
    # point is used as the next target point. SO, if all else fails, could take
    # the index found as the smallest at this point and use that point as the
    # next target.

    # Next, to improve our controller, we want to find which point AFTER the
    # closest point in our path (sequentially) is the closest to being one
    # lookahead distance away from the car.
    # 
    # To do this, first get a view of the path with only the elements of the
    # path that come after the point closest to the car.

    # Next, subtract the lookahead distance from each of these
    # points--"normalizing" them.

    # Finally, select the point with the smallest value among them--this is the
    # node whose distance is closest to one lookahead distance away.


    # NOTE: Other potential problems I'm thinking of:
    # 1. Our path is a loop, really. Therefore, if at the start of the path,
    #    could find that the point behind our car (the last point in the path)
    #    is closer to one lookahead distance than points "in front of" the
    #    vehicle. I.e., we'd prefer our car to pick the target that's in front
    #    of it.
    #   POSSIBLE SOLUTION: Could I filter out points that aren't > than +/- 90
    #   degrees from the x-axis of the car?
    #   NOTE: I think it's worth trying to implement this without taking this
    #   into account first and seeing how it works. If it's a problem, then I
    #   can try this addition to fix things.

    # 2. Our path is a loop--but the path list ends. Therefore, when we're at
    #    the end of the path, how do we get it to see the waypoints in front of
    #    it at the beginning of the path?
    #   POSSIBLE SOLUTION: Could build a custom interface around the path list
    #   so that the path list is reconstructed with the closest point being the
    #   first element in this new list, and the point behind it sequentially in
    #   the original list would be at the end of this new path list or view.
    #   NOTE: This may be unique to racing, as the path is a loop. However,
    #   check to see if this is addressed in the paper. I feel like it would
    #   almost have to be addressed.
    #   TODO: OR, consider making the path publisher node aware of the car's
    #   position in the map, and (like a real path planning node would) publish
    #   a new path with the starting point at its current position. I feel like
    #   this is the "most" correct way of doing this, as a path planner is
    #   constantly updating its path to get from some pose to the goal pose.
    #   However, I guess the only difference is that we're constantly moving the
    #   goal pose around the track? (I.e., like it would be chasing its tail).

    #   NOTE: Just to reiterate though: I feel like I kind of need to do it the
    #   way I'm describing here, as the pure pursuit node itself shouldn't
    #   really be concerned with the path. It should just be worried about
    #   getting to the next target point ALONG THE PATH IT CURRENTLY HAS. That's
    #   it. ESPECIALLY because the end of the path and the beginning of it
    #   connecting is a special case--that's not necessarily normal--and I don't
    #   think it's in scope.
    #   TODO: Vamping on this idea: I think it would be the role of a
    #   "behaviorial planner" node specifically for this car/application that
    #   would be responsible for changing the goal waypoint for the global
    #   path planner--or itself implementing the logic to say "okay, the car has
    #   reached the last waypoint of the track global path--time to move its
    #   local planner's goal node back to the first point in the original path!"
    #   Upon the goal point changing, that's when the global planner reroutes or
    #   replans. Check if there's a ROS REP that specifies how this should work.
    #   Also, the behavioral planner could be implemented in a stateful way such
    #   that if it's in "race" state, it just updates that goal pose to keep the
    #   car with a planned path around the track, but if a button is pressed to
    #   finish the race or something, then it could place the goal pose right
    #   after the finish line and stop right after that, for instance.
    
    pass