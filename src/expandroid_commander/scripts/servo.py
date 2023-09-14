import serial
import time

# Open serial port (replace 'COM3' with the correct port on your system)
ser = serial.Serial("/dev/ttyUSB0", 9600)

# Wait for the Arduino to initialize
time.sleep(2)

# Array to hold servo angles
servo_angles_open = [135, 45, 135, 45, 135, 45]
servo_angles_close = [20, 150, 20, 150, 20, 150]


def servo_open():
    angle_str = ",".join(map(str, servo_angles_open))
    ser.write((angle_str + "\n").encode("utf-8"))
    time.sleep(1)


def servo_close():
    angle_str = ",".join(map(str, servo_angles_close))
    ser.write((angle_str + "\n").encode("utf-8"))
    time.sleep(1)


servo_close()
# servo_open()
# servo_close()
print("Done")
