from pymavlink import mavutil
import time
import numpy as np
import cv2
import base64

connection = mavutil.mavlink_connection('COM9', 57600, mavlink_version='2.0')
# Assume these variables:
chunks = {}  # seqnr -> raw bytes
expected_packets = None
image_size = None

def on_handshake(handshake_msg):
    global expected_packets, image_size
    expected_packets = handshake_msg.packets
    image_size = handshake_msg.size  # Real JPEG size

def on_encapsulated_data(encapsulated_data_msg):
    seqnr = encapsulated_data_msg.seqnr
    chunk_data = encapsulated_data_msg.data  # Correct field!

    chunks[seqnr] = chunk_data

    if len(chunks) == expected_packets:
        reconstruct_image()

def reconstruct_image():
    global chunks, expected_packets, image_size

    # Concatenate all chunks in order
    image_bytes = b''.join(chunks[i] for i in range(expected_packets))

    # Trim to actual JPEG size (some padding bytes at the end)
    image_bytes = image_bytes[:image_size]

    # Decode JPEG image
    image_array = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

    if image is not None:
        print("Image successfully reconstructed!")
        cv2.imshow("Thermal Image", image)
        cv2.waitKey(0)
    else:
        print("Failed to decode image!")

    chunks.clear()


def reconstruct_image():
    global chunks, expected_packets, image_size

    # Sort chunks by sequence number
    image_bytes = b''.join(chunks[i] for i in range(expected_packets))

    # Trim to actual image size (remove padding)
    image_bytes = image_bytes[:image_size]

    # Now decode the JPEG bytes
    image_array = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)  # or IMREAD_UNCHANGED if you prefer

    if image is not None:
        print("Image successfully reconstructed!")
        # You can now display it
        cv2.imshow("Thermal Image", image)
        cv2.waitKey(0)
    else:
        print("Failed to decode image!")

    # Clear for next image
    chunks.clear()

def handle_named_value_float(message):
    print(f"Received {message.name}: {message.value}")

def handle_gps_raw_int(message):
    latitude = message.lat
    longitude = message.lon
    altitude = message.alt
    velocity = message.vx
    print(f"Latitude: {latitude} Longitude: {longitude} Altitude: {altitude} meters Velocity: {velocity/100.0} m/s")

while True:
    try:
        # Block until a new message is received
        message = connection.recv_match(blocking=True)
        print(f"Raw message: {message}")
        if message:
            print(f"Message: {message}")
            # Check the type of the message and handle accordingly
            if message.get_type() == 'NAMED_VALUE_FLOAT':
                handle_named_value_float(message)
            elif message.get_type() == 'GLOBAL_POSITION_INT':
                handle_gps_raw_int(message)
            elif message.get_type() == 'STATUSTEXT':
                print(f"Status Text: {message.text}")
            elif message.get_type() == 'ENCAPSULATED_DATA':
                on_encapsulated_data(message)
            elif message.get_type() == 'DATA_TRANSMISSION_HANDSHAKE':
                on_handshake(message)
            
    except Exception as e:
        print(f"An error occurred: {e}")
    time.sleep(0.1)  # Sleep to limit looping speed