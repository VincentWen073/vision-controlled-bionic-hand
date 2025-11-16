"""
Handles serial communication with a microcontroller for the bionic hand.

This module provides a SerialHandler class responsible for establishing a
serial connection, formatting data packets with servo angles, and sending them
reliably.
"""

import time
from typing import List, Optional

import serial


class SerialHandler:
    """
    Manages the serial port connection and data transmission.

    This class encapsulates the logic for sending servo angles according to a
    defined protocol: '<a1,a2,a3,a4,a5,a6,checksum>\\n'. It handles connection
    errors and ensures data integrity through a simple XOR checksum.

    Attributes:
        port (str): The serial port identifier (e.g., '/dev/serial0').
        baudrate (int): The communication speed.
        ser (Optional[serial.Serial]): The pySerial object for the serial
                                       connection, or None if connection failed.
    """

    # --- Constants for the communication protocol ---
    PACKET_START_MARKER = b"<"
    PACKET_END_MARKER = b">\n"
    NUM_SERVOS = 6

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 1.0):
        """
        Initializes the serial connection.

        Args:
            port (str): The serial port to connect to (e.g., '/dev/serial0' or 'COM3').
            baudrate (int): The communication speed.
            timeout (float): The read/write timeout in seconds.
        """
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=timeout)
            time.sleep(2)
            print(
                f"Successfully opened serial port {self.port} at {self.baudrate} baud."
            )
        except serial.SerialException as e:
            print(f"Error: Could not open serial port {self.port}. {e}")
            raise e

    def send_angles(self, angles: List[int]) -> bool:
        """
        Formats and sends a list of servo angles over the serial port.

        This method constructs a data packet including a checksum, encodes it,
        and sends it. It performs checks to ensure the connection is active and
        the input data is valid.

        Args:
            angles (List[int]): A list of integer angles for the servos.
                                The length must match NUM_SERVOS.

        Returns:
            bool: True if the command was sent successfully, False otherwise.
        """
        if not self.is_connected():
            # The main loop should be aware that the serial port is not available.
            # Printing this every frame can be noisy, so it's kept minimal.
            return False

        if len(angles) != self.NUM_SERVOS:
            print(
                f"Error: Invalid angle list length. Expected {self.NUM_SERVOS}, got {len(angles)}."
            )
            return False

        try:
            # --- Packet Construction ---
            # 1. Calculate XOR checksum
            checksum = 0
            for angle in angles:
                # Ensure all angles are integers before checksum calculation
                checksum ^= int(angle)

            # 2. Format the data string
            angles_str = ",".join(map(str, angles))

            # 3. Build the final command string and encode it to bytes
            command_str = f"{angles_str},{checksum}"
            packet = (
                self.PACKET_START_MARKER
                + command_str.encode("ascii")
                + self.PACKET_END_MARKER
            )

            # --- Send the Packet ---
            self.ser.write(packet)
            # self.ser.flush() # Optional: ensure all data is sent immediately
            return True

        except (serial.SerialException, IOError) as e:
            print(f"Error writing to serial port: {e}")
            # Consider closing the port or attempting to reconnect here if needed
            return False
        except Exception as e:
            print(f"An unexpected error occurred during serial write: {e}")
            return False

    def is_connected(self) -> bool:
        """Checks if the serial port is open and available."""
        return self.ser is not None and self.ser.is_open

    def close(self):
        """Closes the serial port connection if it is open."""
        if self.is_connected():
            try:
                self.ser.close()
                print(f"Serial port {self.port} closed.")
            except Exception as e:
                print(f"Error closing serial port: {e}")
