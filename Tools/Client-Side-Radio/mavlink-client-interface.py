from pymavlink import mavutil
import time

connection = mavutil.mavlink_connection('COM9', 57600, mavlink_version='2.0')

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
            
    except Exception as e:
        print(f"An error occurred: {e}")
    time.sleep(0.1)  # Sleep to limit looping speed