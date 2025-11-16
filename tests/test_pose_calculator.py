# tests/test_pose_calculator.py

import numpy as np
import pytest

from src.kinematics.pose_calculator import (
    calculate_angle_between_vectors,
    calculate_palm_normal_vector,
    calculate_signed_angle_3d,
)

# --- Test Fixtures (Reusable Test Data) ---


@pytest.fixture
def sample_landmarks_right_hand():
    """Provides a mock list of landmarks for a simple right hand pose."""

    landmarks = [None] * 21

    # Mock Landmark class to simulate MediaPipe's structure
    class MockLandmark:
        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    # Define key points for a flat hand on the XY plane
    landmarks[0] = MockLandmark(0, 0, 0)  # Wrist
    landmarks[5] = MockLandmark(0, 1, 0)  # Index MCP (along Y axis)
    landmarks[17] = MockLandmark(1, 0, 0)  # Pinky MCP (along X axis)
    return landmarks


@pytest.fixture
def handedness_right():
    """Provides a mock handedness object for a right hand."""

    # Mock Category class
    class MockCategory:
        def __init__(self, name):
            self.category_name = name

    return MockCategory("Right")


# --- Test Cases ---


def test_calculate_angle_between_vectors():
    """Tests the unsigned angle calculation with various scenarios."""
    # 1. Orthogonal vectors (90 degrees)
    vec_x = np.array([1, 0, 0])
    vec_y = np.array([0, 1, 0])
    assert np.isclose(calculate_angle_between_vectors(vec_x, vec_y), 90.0)

    # 2. Parallel vectors (0 degrees)
    vec_a = np.array([1, 2, 3])
    vec_b = np.array([2, 4, 6])
    assert np.isclose(calculate_angle_between_vectors(vec_a, vec_b), 0.0)

    # 3. Anti-parallel vectors (180 degrees)
    vec_c = np.array([-1, -2, -3])
    assert np.isclose(calculate_angle_between_vectors(vec_a, vec_c), 180.0)

    # 4. Zero vector case
    vec_zero = np.array([0, 0, 0])
    assert np.isclose(calculate_angle_between_vectors(vec_a, vec_zero), 0.0)


def test_calculate_signed_angle_3d():
    """Tests the signed angle calculation for rotation direction."""
    vec_x = np.array([1, 0, 0])
    vec_y = np.array([0, 1, 0])
    axis_z = np.array([0, 0, 1])  # Rotation axis is Z

    # 1. Counter-clockwise rotation (from X to Y around Z) should be positive
    assert np.isclose(calculate_signed_angle_3d(vec_x, vec_y, axis_z), 90.0)

    # 2. Clockwise rotation (from Y to X around Z) should be negative
    assert np.isclose(calculate_signed_angle_3d(vec_y, vec_x, axis_z), -90.0)

    # 3. Rotation axis is opposite
    axis_neg_z = np.array([0, 0, -1])
    assert np.isclose(calculate_signed_angle_3d(vec_x, vec_y, axis_neg_z), -90.0)

    # 4. Collinear vectors should result in 0 angle
    vec_collinear = np.array([2, 0, 0])
    assert np.isclose(calculate_signed_angle_3d(vec_x, vec_collinear, axis_z), 0.0)


def test_calculate_palm_normal_vector(sample_landmarks_right_hand, handedness_right):
    """
    Tests the palm normal vector calculation.

    For our sample right hand, the points are:
    - Wrist at (0,0,0)
    - Index MCP at (0,1,0) -> vec_wrist_to_index = (0,1,0)
    - Pinky MCP at (1,0,0) -> vec_wrist_to_pinky = (1,0,0)

    The cross product of (0,1,0) x (1,0,0) is (0,0,-1).
    Since it's a 'Right' hand, the vector should be flipped to (0,0,1).
    """
    expected_normal = np.array([0, 0, 1.0])

    calculated_normal = calculate_palm_normal_vector(
        sample_landmarks_right_hand, handedness_right
    )

    # Use np.allclose for comparing floating point arrays
    assert np.allclose(calculated_normal, expected_normal)
