from pymodbus.client import ModbusTcpClient
import time

# Motor and MODBUS settings
MOTOR_IP = '192.168.33.1'  # Replace with your motor controller IP
MOTOR_PORT = 502  # Standard MODBUS TCP port
STEPS_PER_REV = 51200  # Example: 2000 steps per revolution (depends on microstepping setting)

# Registers (addresses are in decimal)
MOVE_ABSOLUTE_REGISTER = 0x0043
POSITION_COUNTER_REGISTER = 0x0057
INITIAL_VELOCITY_REG = 0x0089  # Initial velocity register address
MAX_VELOCITY_REG = 0x008B  # Maximum velocity register address

# Define the motor speed you want to set (in steps per second or RPM, adjust as needed)
LOW_SPEED = 1  # Example low speed in steps per second
HIGH_SPEED = 1  # Example high speed in steps per second

# Connect to the motor
client = ModbusTcpClient(MOTOR_IP, port=MOTOR_PORT)
client.connect()

# Define positions
position_0_rev = 0
position_10_rev = 20

def set_motor_speed(initial_velocity, max_velocity):
    # Write to initial velocity register
    client.write_register(INITIAL_VELOCITY_REG, initial_velocity)
    print(f"Initial velocity set to {initial_velocity} steps/sec.")

    # Write to maximum velocity register
    client.write_register(MAX_VELOCITY_REG, max_velocity)
    print(f"Maximum velocity set to {max_velocity} steps/sec.")


# Helper function to move to an absolute position
def move_to_position(position_steps):
    # Move to absolute position by writing 32-bit value
    # pymodbus expects to split 32-bit integers into two 16-bit registers
    high = (position_steps >> 16) & 0xFFFF
    low = position_steps & 0xFFFF
    print([high, low])

    client.write_registers(MOVE_ABSOLUTE_REGISTER, [high, low])

# Set the motor speed to the desired values
set_motor_speed(LOW_SPEED, HIGH_SPEED)

# Wait a moment for the motor to adjust to the new speed
time.sleep(2)


while (True):
    # Move to 10 revolutions
    print("Moving to 10 revolutions...")
    move_to_position(position_10_rev)
    time.sleep(10)

    # Move to 0 revolutions
    print("Moving to 0 revolutions...")
    move_to_position(position_0_rev)
    time.sleep(10)  # Wait for motion to complete (adjust based on motion speed)

# Close connection
client.close()
