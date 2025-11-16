"""
Main application file for the Vision-Controlled Bionic Hand.

This script initializes the camera, MediaPipe hand landmark detection,
serial communication, and runs the main state machine loop for controlling
the bionic hand.

Controls:
    'r': Start/Stop the hand recognition process.
    'q': Quit the application.
"""

import os
import time
from enum import Enum, auto

import cv2 as cv
import mediapipe as mp
import numpy as np
from picamera2 import Picamera2

# Local imports for project-specific modules
from src.visualization.drawing import draw_landmarks_on_image
from src.kinematics import pose_calculator
from src.communication.serial_handler import SerialHandler

# ---Type Aliases and Constants ---
BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode


# --- Application State Machine ---
class State(Enum):
    """Defines the operational states of the application."""

    IDLE = auto()
    RUNNING = auto()
    WAITING = auto()


# --- Global Variables for MediaPipe Callback ---
LATEST_RESULT: HandLandmarkerResult = None
INFERENCE_TIME_MS: int = 0


def result_callback(
    result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int
):
    """
    Callback function for MediaPipe's async hand detection.

    This function is executed in a separate thread by MediaPipe and is responsible
    for updating the global variables with the latest detection result and
    inference time.

    Args:
        result (HandLandmarkerResult): The hand landmark detection result.
        output_image (mp.Image): The output image with landmarks (not used here).
        timestamp_ms (int): The timestamp of the frame when detection was initiated.
    """
    global LATEST_RESULT, INFERENCE_TIME_MS
    LATEST_RESULT = result
    INFERENCE_TIME_MS = int(time.perf_counter() * 1000) - timestamp_ms


def main():
    """
    Main function to run the hand tracking and control application.
    """
    # --- Configuration ---
    CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480
    SMOOTHING_FACTOR = 0.2
    WAITING_TIMEOUT_S = 2.0
    DEFAULT_SERVO_ANGLES = [0, 180, 180, 180, 180, 90]

    # --- Initialization ---
    print("Initializing camera...")
    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()

    print("Initializing MediaPipe Hand Landmarker...")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_asset_path = os.path.join(script_dir, "..", "models", "hand_landmarker.task")
    print(f"Attempting to load model from: {model_asset_path}")
    options = HandLandmarkerOptions(
        num_hands=1,
        min_hand_detection_confidence=0.5,
        base_options=BaseOptions(model_asset_path),
        running_mode=VisionRunningMode.LIVE_STREAM,
        result_callback=result_callback,
    )

    print("Initializing serial communication...")

    try:
        serial_comm = SerialHandler(port="/dev/serial0", baudrate=115200)
    except Exception as e:
        print(f"Failed to connect serial port:{e}. Running in visual-only mode.")
        serial_comm = None

    # --- State and Control Varianbles ---
    current_state = State.IDLE
    waiting_start_time = 0
    is_reference_pose_set = False
    reference_palm_normal = None
    smoothed_angles = np.array(DEFAULT_SERVO_ANGLES, dtype=float)
    fps = 0

    print("Initialization complete. Starting main loop...")
    print("Controls: Press 'r' to start/stop, 'q' to quit.")

    with HandLandmarker.create_from_options(options) as landmarker:
        while True:
            frame_start_time = time.perf_counter()

            # --- Frame Capture and Processing ---
            frame = picam2.capture_array()
            annotated_frame = frame.copy()
            raw_servo_angles = None

            # --- Handle User Input ---
            key = cv.waitKey(40)
            if key == ord("q"):
                print("'Q' Pressed. Exiting program...")
                break

            if key == ord("r"):
                if current_state in [State.RUNNING, State.WAITING]:
                    print("State changed: -> IDLE")
                    current_state = State.IDLE
                    is_reference_pose_set = False
                    if serial_comm:
                        serial_comm.send_angles(DEFAULT_SERVO_ANGLES)
                else:  # current_state is IDLE
                    print("State changed: IDLE -> RUNNING")
                    current_state = State.RUNNING

            # --- State Machine Logic ---
            if current_state != State.IDLE:
                # Perform detection for both RUNNING and WAITING states
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
                landmarker.detect_async(mp_image, int(time.perf_counter() * 1000))

            if current_state == State.RUNNING:
                if LATEST_RESULT is not None and LATEST_RESULT.hand_landmarks:
                    landmarks = LATEST_RESULT.hand_landmarks[0]
                    handedness = LATEST_RESULT.handedness[0]
                    if not is_reference_pose_set:
                        print("Capturing palm as reference...")
                        reference_palm_normal = (
                            pose_calculator.calculate_palm_normal_vector(
                                landmarks, handedness[0]
                            )
                        )
                        is_reference_pose_set = True

                    # Calculate and send servo angles
                    annotated_frame = draw_landmarks_on_image(
                        annotated_frame, LATEST_RESULT
                    )
                    raw_servo_angles = pose_calculator.calculate_servo_angles(
                        LATEST_RESULT, reference_palm_normal
                    )
                    print(raw_servo_angles)

                    if raw_servo_angles is not None:
                        for i in range(6):
                            smoothed_angles[i] = (
                                SMOOTHING_FACTOR * raw_servo_angles[i]
                                + (1 - SMOOTHING_FACTOR) * smoothed_angles[i]
                            )
                        final_angles_to_send = [int(angle) for angle in smoothed_angles]
                        print(f"Sending angles: {final_angles_to_send}")
                        if serial_comm:
                            serial_comm.send_angles(final_angles_to_send)
                else:
                    # Hand lost, transition to WAITING state
                    print("No Hand detected. Switch state RUNNING to WATING...")
                    current_state = State.WAITING
                    waiting_start_time = time.perf_counter()

            elif current_state == State.WAITING:
                if LATEST_RESULT is not None and LATEST_RESULT.hand_landmarks:
                    # Hand re-detected, transition back to RUNNING
                    print("Hand detected. Switch state WAITING to RUNNING...")
                    current_state = State.RUNNING
                else:
                    # Check for timeout to reset the hand
                    elapsed_time = time.perf_counter() - waiting_start_time
                    if elapsed_time >= WAITING_TIMEOUT_S and is_reference_pose_set:
                        print(f"Timeout: ({WAITING_TIMEOUT_S}s. Resetting hand pose.)")
                        if serial_comm:
                            serial_comm.send_angles(DEFAULT_SERVO_ANGLES)
                        is_reference_pose_set = False
                        reference_palm_normal = None
            # --- Display Information on Frame ---
            state_text = f"STATE: {current_state.name}"
            cv.putText(
                annotated_frame,
                state_text,
                (10, 60),
                cv.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
            )
            cv.putText(
                annotated_frame,
                f"FPS: {int(fps)}",
                (10, 30),
                cv.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv.putText(
                annotated_frame,
                f"Inference: {INFERENCE_TIME_MS}ms",
                (150, 30),
                cv.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            cv.imshow("Vision Controlled Bionic Hand", annotated_frame)

            # --- FPS Calculation ---
            total_duration = time.perf_counter() - frame_start_time
            fps = 1 / total_duration if total_duration > 0 else 0

    # --- Cleanup ---
    picam2.stop()
    cv.destroyAllWindows()
    print("Exit the program")
    # Ensure hand is reset on exit
    if serial_comm:
        serial_comm.send_angles(DEFAULT_SERVO_ANGLES)
    print("Application closed.")


if __name__ == "__main__":
    main()
