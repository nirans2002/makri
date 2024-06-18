
from adafruit_servokit import ServoKit
import time

kit = ServoKit(channels=16)

# Calibration sequence
print("Disconnect power from the ESC, then press Enter to continue.")
input()  # Wait for user confirmation
print("....")
# Set the ESC to maximum throttle
kit.servo[0].angle = 180
kit.servo[1].angle = 180
kit.servo[2].angle = 180

time.sleep(0)
print("Connect power to the ESC, wait for initialization, then press Enter.")
input()  # Wait for user confirmation
print("....")
# Set the ESC to minimum throttle
kit.servo[0].angle = 0
kit.servo[1].angle = 0
kit.servo[2].angle = 0

time.sleep(2)
print("Calibration complete. You can now control the speed of the ESC.")


while True:
    speed = int(input("Enter speed (0-100): "))
    if speed < 0:
        speed = 0
    elif speed > 100:
        speed = 100


    kit.servo[0].angle = speed
    kit.servo[1].angle = speed
    kit.servo[2].angle = speed
    