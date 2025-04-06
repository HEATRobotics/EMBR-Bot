import serial

ser = serial.Serial('COM3', 57600, timeout=1)


print(ser.name)

while True:
    # Read a line from the serial port
    line = ser.readline().rstrip()

    try:
        # Decode the line to a string
        line = line.decode('utf-8')
    except UnicodeDecodeError:
        # If decoding fails, skip this line
        print("REMOTE", line)
    
    # Print the received line
    # print("REMOTE", line)
    
    # Check if the line contains 'exit' to break the loop
    # if 'exit' in line:
    #     break

ser.close()