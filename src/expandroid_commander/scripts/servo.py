import serial
import time

# Open serial port (replace 'COM3' with the correct port on your system)
ser = serial.Serial("/dev/ttyUSB0", 9600)

# Wait for the Arduino to initialize
time.sleep(2)

# Array to hold servo angles
servo_angles_open = [135, 135, 135, 135, 135]
servo_angles_close = [30, 30, 30, 30, 30]

for i in range(10):
    if i % 2 == 0:
        servo_angles = servo_angles_close
    else:
        servo_angles = servo_angles_open

    # Convert the list of angles to a comma-separated string
    angle_str = ",".join(map(str, servo_angles))

    # Send the comma-separated string followed by a newline character
    ser.write((angle_str + "\n").encode("utf-8"))

    # Wait for a while before sending the next command
    time.sleep(1)

    # Modify servo_angles if needed, for example:
    # servo_angles = [angle + 10 for angle in servo_angles]
