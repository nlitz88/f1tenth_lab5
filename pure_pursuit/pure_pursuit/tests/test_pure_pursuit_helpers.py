import csv
from pure_pursuit.pure_pursuit import *
from pure_pursuit.pure_pursuit_helpers import *
from pure_pursuit.path_publisher_helpers import generate_path_from_file
import unittest
from geometry_msgs.msg import Point
from nav_msgs.msg import Path

# class TestNumpyArrayFromPath(unittest.TestCase):

#     def setUp(self) -> None:
#         """Builds up a path from the specified filepath.
#         """
#         self.path = generate_path_from_file(filepath=Path().cwd().parent.parent.parent/"path_files"/"levine_rough_path.csv")

#     def test_shape(self):
#         print(self.path)

class TestNumpyArrayFromPath(unittest.TestCase):
    def test_conversion(self):
        # Create a sample Path with Pose instances
        path = Path()
        for i in range(5):
            pose = Pose()
            pose.position = Point(x=float(i), y=float(i), z=float(0))
            path.poses.append(pose)

        # Convert the Path to a NumPy array
        result = numpy_array_from_path(path)

        # Define the expected NumPy array
        expected = np.array([[0, 0], [1, 1], [2, 2], [3, 3], [4, 4]])

        # Check if the result matches the expected array
        self.assertTrue(np.array_equal(result, expected))

    def test_empty_path(self):
        # Test with an empty Path
        path = Path()
        result = numpy_array_from_path(path)
        self.assertTrue(len(result) == 0)

    def test_path_with_negative_values(self):
        # Test with a Path containing negative values
        path = Path()
        for i in range(-3, 4):
            pose = Pose()
            pose.position = Point(x=float(i), y=float(i), z=float(0))
            path.poses.append(pose)
        result = numpy_array_from_path(path)
        expected = np.array([[-3, -3], [-2, -2], [-1, -1], [0, 0], [1, 1], [2, 2], [3, 3]])
        self.assertTrue(np.array_equal(result, expected))

    def test_path_with_large_values(self):
        # Test with a Path containing large values
        path = Path()
        for i in range(1000, 1005):
            pose = Pose()
            pose.position = Point(x=float(i), y=float(i), z=float(0))
            path.poses.append(pose)
        result = numpy_array_from_path(path)
        expected = np.array([[1000, 1000], [1001, 1001], [1002, 1002], [1003, 1003], [1004, 1004]])
        self.assertTrue(np.array_equal(result, expected))

    def test_path_with_float_coordinates(self):
        # Test with a Path containing floating-point coordinates
        path = Path()
        for i in range(5):
            pose = Pose()
            x = 0.5 * i  # Using floating-point x coordinates
            y = 1.5 * i  # Using floating-point y coordinates
            pose.position = Point(x=x, y=y, z=0.0)
            path.poses.append(pose)

        result = numpy_array_from_path(path)

        # Define the expected NumPy array with floating-point values
        expected = np.array([[0.0, 0.0], [0.5, 1.5], [1.0, 3.0], [1.5, 4.5], [2.0, 6.0]])

        # Check if the result matches the expected array
        self.assertTrue(np.allclose(result, expected, rtol=1e-8))
        
class TestEuclideanDistance(unittest.TestCase):
    def test_euclidean_distance_same_vector(self):
        vector = np.array([1.0, 2.0, 3.0])
        self.assertEqual(euclidean_distance(vector, vector), 0.0)

    def test_euclidean_distance_different_vectors(self):
        vector_a = np.array([1.5, 2.5, 3.5])
        vector_b = np.array([4.0, 5.0, 6.0])
        expected_distance = np.sqrt(np.sum((vector_a - vector_b) ** 2))
        self.assertEqual(euclidean_distance(vector_a, vector_b), expected_distance)

    def test_euclidean_distance_zero_vectors(self):
        vector_a = np.array([0.0, 0.0, 0.0])
        vector_b = np.array([0.0, 0.0, 0.0])
        self.assertEqual(euclidean_distance(vector_a, vector_b), 0.0)

    def test_euclidean_distance_mixed_data_types(self):
        vector_a = np.array([1.0, 2.5, 3.0])
        vector_b = np.array([4.0, 5, 6.5])
        expected_distance = np.sqrt(np.sum((vector_a - vector_b) ** 2))
        self.assertEqual(euclidean_distance(vector_a, vector_b), expected_distance)

class TestGetDistanceToEachPoint(unittest.TestCase):
    def test_distance_to_each_point(self):
        current_position = np.array([1.0, 2.0])
        numpy_path = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
        expected_distances = np.array([0.0, np.sqrt(8.0), np.sqrt(32.0)])
        distances = get_distance_to_each_point(current_position, numpy_path)
        np.testing.assert_array_almost_equal(distances, expected_distances, decimal=6)

    def test_distance_to_each_point_single_point_path(self):
        current_position = np.array([1.0, 2.0])
        numpy_path = np.array([[3.0, 4.0]])
        expected_distance = np.sqrt(8.0)
        distances = get_distance_to_each_point(current_position, numpy_path)
        self.assertEqual(distances[0], expected_distance)

    def test_distance_to_each_point(self):
        current_position = np.array([1.0, 2.0])
        numpy_path = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
        expected_distances = np.array([0.0, np.sqrt(8.0), np.sqrt(32.0)])
        distances = get_distance_to_each_point(current_position, numpy_path)
        np.testing.assert_array_almost_equal(distances, expected_distances, decimal=6)

    def test_distance_to_each_point_long_path(self):
        current_position = np.array([0.0, 0.0])
        numpy_path = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0], [7.0, 8.0], [9.0, 10.0]])
        expected_distances = np.array([np.sqrt(5.0), np.sqrt(25.0), np.sqrt(61.0), np.sqrt(113.0), np.sqrt(181.0)])
        distances = get_distance_to_each_point(current_position, numpy_path)
        np.testing.assert_array_almost_equal(distances, expected_distances, decimal=6)

    def test_distance_to_each_point_negative_coordinates(self):
        current_position = np.array([-2.0, -3.0])
        numpy_path = np.array([[-2.0, -3.0], [1.0, 1.0], [-5.0, -6.0]])
        expected_distances = np.array([0.0, np.sqrt(25.0), np.sqrt(18)])
        distances = get_distance_to_each_point(current_position, numpy_path)
        np.testing.assert_array_almost_equal(distances, expected_distances, decimal=6)