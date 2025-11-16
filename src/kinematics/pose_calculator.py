"""
Performs kinematic calculations for the bionic hand.

This module contains functions to:
- Calculate geometric properties from MediaPipe hand landmarks (e.g., palm normal).
- Estimate finger curl angles.
- Compute the signed angle between 3D vectors for wrist rotation.
- Map calculated angles to target servo motor angles.
"""

from typing import List, Literal, Optional

import numpy as np
from mediapipe.tasks.python.components.containers.category import Category
from mediapipe.tasks.python.components.containers.landmark import NormalizedLandmark
from mediapipe.tasks.python.vision import HandLandmarkerResult
from numpy.typing import NDArray

# --- Constants and Type Definitions ---

# Type alias for finger names for improved type hinting.
FingerName = Literal["THUMB", "INDEX", "MIDDLE", "RING", "PINKY"]

FINGER_INDICES = {
    "THUMB": [1, 2, 3, 4],
    "INDEX": [5, 6, 7, 8],
    "MIDDLE": [9, 10, 11, 12],
    "RING": [13, 14, 15, 16],
    "PINKY": [17, 18, 19, 20],
}

ANGLE_TO_SERVO_MAPPING = {
    "THUMB": {"input_range": [0, 50], "output_range": [0, 180]},
    "INDEX": {"input_range": [-45, -2], "output_range": [0, 180]},
    "MIDDLE": {"input_range": [-45, -2], "output_range": [0, 180]},
    "RING": {"input_range": [-45, -5], "output_range": [0, 180]},
    "PINKY": {"input_range": [-45, -5], "output_range": [0, 180]},
    "WRIST": {"input_range": [-90, 90], "output_range": [0, 180]},
}


# --- Utility Functions ---


def _landmark_to_array(landmark: NormalizedLandmark) -> NDArray[np.float64]:
    """Converts a MediaPipe NormalizedLandmark to a NumPy array."""
    return np.array([landmark.x, landmark.y, landmark.z])


def _normalize_vector(vector: NDArray[np.float64]) -> NDArray[np.float64]:
    """Returns the unit vector of a given vector, or a zero vector if norm is zero."""
    norm = np.linalg.norm(vector)
    if norm == 0:
        return np.array([0.0, 0.0, 0.0])
    return vector / norm


# --- Core Geometric Calculation Functions ---


def calculate_angle_between_vectors(vec1: NDArray, vec2: NDArray) -> float:
    """
    Calculates the unsigned angle between two 3D vectors in degrees.

    Args:
        vec1 (NDArray): The first vector.
        vec2 (NDArray): The second vector.

    Returns:
        float: The angle in degrees, ranging from 0 to 180.
    """
    if np.all(vec1 == 0) or np.all(vec2 == 0):
        return 0.0

    vec1_unit = _normalize_vector(vec1)
    vec2_unit = _normalize_vector(vec2)

    dot_product = np.dot(vec1_unit, vec2_unit)
    # Clipping is essential to handle floating point inaccuracies
    clipped_dot_product = np.clip(dot_product, -1.0, 1.0)

    angle_rad = np.arccos(clipped_dot_product)
    return np.degrees(angle_rad)


def calculate_signed_angle_3d(vec1: NDArray, vec2: NDArray, axis: NDArray) -> float:
    """
    Calculates the signed angle from vec1 to vec2 around a specified rotation axis.

    The sign indicates the direction of rotation (e.g., positive for counter-clockwise
    based on the right-hand rule with the given axis).

    Args:
        vec1 (NDArray): The starting vector.
        vec2 (NDArray): The ending vector.
        axis (NDArray): The axis of rotation.

    Returns:
        float: The signed angle in degrees, typically ranging from -180 to 180.
    """
    unsigned_angle = calculate_angle_between_vectors(vec1, vec2)

    cross_prod = np.cross(vec1, vec2)

    # The sign of the dot product between the cross product and the rotation axis
    # determines the direction of rotation.
    direction_sign = np.sign(np.dot(cross_prod, axis))

    # If vectors are collinear, the cross product is zero, so the sign is zero.
    if direction_sign == 0:
        return 0.0

    return unsigned_angle * direction_sign


def calculate_palm_normal_vector(
    landmarks: List[NormalizedLandmark], handedness: Category
) -> NDArray[np.float64]:
    """
    Calculates the normal vector of the palm.

    The normal vector is computed using three points on the palm (wrist, index MCP,
    and pinky MCP) to define a plane. The direction is adjusted based on handedness
    to ensure it consistently points "out" of the palm.

    Args:
        landmarks (List[NormalizedLandmark]): A list of all 21 hand landmarks.
        handedness (Category): The detected handedness ('Left' or 'Right').

    Returns:
        NDArray[np.float64]: A unit normal vector representing the palm's orientation.
    """
    p0_wrist = _landmark_to_array(landmarks[0])
    p5_index_mcp = _landmark_to_array(landmarks[5])
    p17_pinky_mcp = _landmark_to_array(landmarks[17])

    # Two vectors define the plane of the palm
    vec_wrist_to_index = p5_index_mcp - p0_wrist
    vec_wrist_to_pinky = p17_pinky_mcp - p0_wrist

    # The cross product gives the vector perpendicular to the palm plane
    palm_normal = np.cross(vec_wrist_to_index, vec_wrist_to_pinky)

    # The cross product direction depends on the order of vectors.
    # We flip it for the right hand to maintain a consistent coordinate system
    # (e.g., normal always points away from the palm).
    if handedness.category_name == "Right":
        palm_normal = -palm_normal

    return _normalize_vector(palm_normal)


def get_finger_curl_angle(
    landmarks: List[NormalizedLandmark], finger_name: FingerName, palm_normal: NDArray
) -> float:
    """
    Estimates the curl angle for a specific finger.

    - For the Thumb: Calculates the angle between the metacarpal and proximal phalange.
    - For other fingers: Calculates the angle between the finger's proximal phalange
    and the palm's normal vector. A negative angle typically indicates a curl.

    Args:
        landmarks (List[NormalizedLandmark]): A list of all 21 hand landmarks.
        finger_name (FingerName): The name of the finger to analyze.
        palm_normal (NDArray): The normal vector of the palm.

    Returns:
        float: The calculated curl angle in degrees.
    """
    indices = FINGER_INDICES[finger_name]

    if finger_name == "THUMB":
        p_cmc = _landmark_to_array(landmarks[indices[0]])
        p_mcp = _landmark_to_array(landmarks[indices[1]])
        p_ip = _landmark_to_array(landmarks[indices[2]])

        vec_ref = p_mcp - p_cmc
        vec_move = p_ip - p_mcp

        angle = calculate_angle_between_vectors(vec_ref, vec_move)
        return angle

    else:
        p_mcp = _landmark_to_array(landmarks[indices[0]])
        p_pip = _landmark_to_array(landmarks[indices[1]])

        vec_finger = p_pip - p_mcp

        norm_finger = np.linalg.norm(vec_finger)
        if norm_finger == 0:
            return 0.0
        vec_finger_unit = vec_finger / norm_finger

        dot_product = np.dot(vec_finger_unit, palm_normal)
        dot_product = np.clip(dot_product, -1.0, 1.0)

        angle_rad = np.arcsin(dot_product)
        return np.degrees(angle_rad)


def calculate_servo_angles(
    detection_result: Optional[HandLandmarkerResult], reference_palm_normal: NDArray
) -> Optional[List[int]]:
    """
    Calculates all six servo angles based on the hand landmark detection result.

    This function orchestrates the entire process:
    1. Extracts landmarks and handedness.
    2. Calculates the current palm normal.
    3. Calculates the curl angle for each of the five fingers.
    4. Calculates the signed wrist rotation angle relative to a reference normal.
    5. Maps all calculated angles to servo values (0-180).

    Args:
        detection_result (Optional[HandLandmarkerResult]): The full result from MediaPipe.
        reference_palm_normal (NDArray): The stored normal vector of the initial
                                         hand pose, used as the "zero" for wrist rotation.

    Returns:
        Optional[List[int]]: A list of 6 integer servo angles, or None if calculation fails.
    """
    if not detection_result or not detection_result.hand_landmarks:
        return None

    landmarks = detection_result.hand_landmarks[0]
    handedness = detection_result.handedness[0]

    current_palm_normal = calculate_palm_normal_vector(landmarks, handedness[0])

    # If the palm normal is a zero vector, we cannot proceed.
    if np.all(current_palm_normal == 0):
        return None

    # --- Finger Angle Calculation ---
    servo_angles = []
    finger_names: List[FingerName] = ["THUMB", "INDEX", "MIDDLE", "RING", "PINKY"]
    for name in finger_names:
        angle_deg = get_finger_curl_angle(landmarks, name, current_palm_normal)
        mapping = ANGLE_TO_SERVO_MAPPING[name]
        servo_angle = np.interp(
            angle_deg, mapping["input_range"], mapping["output_range"]
        )
        servo_angles.append(servo_angle)

    # --- Wrist Angle Calculation ---
    # The wrist's rotation axis is defined by the direction of the hand itself.
    p0_wrist = _landmark_to_array(landmarks[0])
    p9_middle_mcp = _landmark_to_array(landmarks[9])
    palm_direction = _normalize_vector(p9_middle_mcp - p0_wrist)

    if np.all(palm_direction == 0):
        # Fallback to a default wrist angle if direction can't be determined
        servo_angles.append(90)
    else:
        wrist_angle_deg = calculate_signed_angle_3d(
            reference_palm_normal, current_palm_normal, palm_direction
        )
        mapping = ANGLE_TO_SERVO_MAPPING["WRIST"]
        wrist_servo_angle = np.interp(
            wrist_angle_deg, mapping["input_range"], mapping["output_range"]
        )
        servo_angles.append(wrist_servo_angle)

    # Final clipping and conversion to integer
    final_angles = [int(np.clip(angle, 0, 180)) for angle in servo_angles]
    return final_angles
