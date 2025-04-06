# laptop_serial_sender.py
import time
from pymavlink import mavutil

# --- Serial Configuration ---
SERIAL_PORT = 'COM3'  # Replace with your laptop's serial port
BAUD_RATE = 57600  # Replace with your configured baud rate
SOURCE_SYSTEM = 200  # Define a unique system ID for your laptop
SOURCE_COMPONENT = 1

def send_float(name,number):
    """Sends a pre-constructed MAVLink message over the serial connection."""
    try:
        # Create a serial connection instance
        mav_serial = mavutil.mavserial(
            device=SERIAL_PORT,
            baud=BAUD_RATE,
        )
        print(f"Connected to serial port: {SERIAL_PORT}")
        name = name.encode('utf-8')  # Encode the name to bytes
        # Write the packed message to the serial port
        mav_serial.mav.named_value_float_send(
            time_boot_ms=int(time.time() * 1000) % 4294967296,
            name=name,
            value=number
        )
        
        print("MAVLink message sent successfully.")

        # Close the serial connection
        mav_serial.close()
        print("Serial connection closed.")

    except Exception as e:
        print(f"Error sending MAVLink message: {e}")

def send_status_text(text):
    """Sends a status text message."""
    try:
        # Create a serial connection instance
        mav_serial = mavutil.mavserial(
            device=SERIAL_PORT,
            baud=BAUD_RATE,
        )
        print(f"Connected to serial port: {SERIAL_PORT}")
        text = text.encode('utf-8')  # Encode the text to bytes
        # Write the packed message to the serial port
        mav_serial.mav.statustext_send(
            severity=0,  # Set severity level (0-255)
            text=text
        )
        
        print("MAVLink status text sent successfully.")

        # Close the serial connection
        mav_serial.close()
        print("Serial connection closed.")

    except Exception as e:
        print(f"Error sending MAVLink status text: {e}")

#send_float("TEST", 1.23)  # Example message to send
send_status_text("Hello from laptop!")  # Example status text to send