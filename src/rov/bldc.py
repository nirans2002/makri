# from adafruit_servokit import ServoKit
# import time

# # Initialize ServoKit with the number of channels your ESC supports
# kit = ServoKit(channels=16)

# # Calibration sequence
# print("Disconnect power from the ESC, then press Enter to continue.")
# input()  # Wait for user confirmation
# print("....")
# # Set the ESC to maximum throttle
# kit.servo[0].angle = 180
# time.sleep(2)
# print("Connect power to the ESC, wait for initialization, then press Enter.")
# input()  # Wait for user confirmation
# print("....")
# # Set the ESC to minimum throttle
# kit.servo[0].angle = 0
# time.sleep(2)
# print("Calibration complete. You can now control the speed of the ESC.")

# # Speed control loop
# while True:
#     speed = int(input("Enter speed (0-100): "))
#     if speed < 0:
#         speed = 0
#     elif speed > 100:
#         speed = 100
    
#     throttle = int(speed * 1.8)  # Map speed (0-100) to throttle (0-180)
#     kit.servo[0].angle = speed
#     print(f"Speed set to {speed}%")
