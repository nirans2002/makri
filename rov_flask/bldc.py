from adafruit_servokit import ServoKit
import time
from mpu6050 import mpu6050
import time,math

# Create a new Mpu6050 o
mpu6050= mpu6050(0x68)

# Define a function to read the sensor data
def read_sensor_data():
    # Read the accelerometer values
    accelerometer_data = mpu6050.get_accel_data()

    # Read the gyroscope values
    gyroscope_data = mpu6050.get_gyro_data()

    # Read temp
    temperature = mpu6050.get_temp()

    return accelerometer_data, gyroscope_data, temperature

def calculate_tilt_angles(accel_data):
    x, y, z = accel_data['x'], accel_data['y'], accel_data['z']


    tilt_x = math.atan2(y, math.sqrt(x * x + z * z)) * 180 / math.pi
    tilt_y = math.atan2(-x, math.sqrt(y * y + z * z)) * 180 / math.pi
    tilt_z = math.atan2(z, math.sqrt(x * x + y * y)) * 180 / math.pi
 
    return tilt_x, tilt_y, tilt_z
 
def complementary_filter(pitch, roll, gyro_data, dt, alpha=0.98):
    pitch += gyro_data['x'] * dt
    roll -= gyro_data['y'] * dt
 
    pitch = alpha * pitch + (1 - alpha) * math.atan2(gyro_data['y'], math.sqrt(gyro_data['x'] * gyro_data['x'] + gyro_data['z'] * gyro_data['z'])) * 180 / math.pi
    roll = alpha * roll + (1 - alpha) * math.atan2(-gyro_data['x'], math.sqrt(gyro_data['y'] * gyro_data['y'] + gyro_data['z'] * gyro_data['z'])) * 180 / math.pi
 
    return pitch, roll

# Function to compute PID control output
def compute_pid_output(error, dt):
    global integral, prev_error
    
    # Proportional term
    proportional = Kp * error
    
    # Integral term
    integral += error * dt
    integral_term = Ki * integral
    
    # Derivative term
    derivative = (error - prev_error) / dt
    derivative_term = Kd * derivative
    
    # Update previous error
    prev_error = error
    
    # Compute PID output
    output = proportional + integral_term + derivative_term
    return output

kit = ServoKit(channels=16)

# Calibration sequence
print("Disconnect power from the ESC, then press Enter to continue.")
input()  # Wait for user confirmation
print("....")
# Set the ESC to maximum throttle
kit.servo[0].angle = 180
time.sleep(2)
print("Connect power to the ESC, wait for initialization, then press Enter.")
input()  # Wait for user confirmation
print("....")
# Set the ESC to minimum throttle
kit.servo[0].angle = 0
time.sleep(2)
print("Calibration complete. You can now control the speed of the ESC.")

def linear_map(value, from_low, from_high, to_low, to_high):
    """
    Linearly maps a value from one range to another range.
    
    Args:
        value (float): The value to be mapped.
        from_low (float): The lower bound of the original range.
        from_high (float): The upper bound of the original range.
        to_low (float): The lower bound of the target range.
        to_high (float): The upper bound of the target range.
        
    Returns:
        float: The mapped value.
    """
    # Ensure the value is within the original range
    value = max(min(value, from_high), from_low)
    
    # Map the value to the target range
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

# Speed control loop
# while True:
#     speed = int(input("Enter speed (0-100): "))
#     if speed < 0:
#         speed = 0
#     elif speed > 100:
#         speed = 100
    
 

 
pitch = 0
roll = 0
prev_time = time.monotonic_ns()
 
while True:
    data = read_sensor_data()
    curr_time = time.monotonic_ns()
    dt = (curr_time - prev_time) / 1000000000
 
    tilt_x, tilt_y, tilt_z = calculate_tilt_angles(data[0])
    pitch, roll = complementary_filter(pitch, roll, data[0], dt)
 
    prev_time = curr_time
    time.sleep(0.01)   
    
    Kp = 1.0  # Proportional gain
    Ki = 0.1  # Integral gain
    Kd = 0.1  # Derivative gain

# Initialize variables for PID controller
    integral = 0
    prev_error = 0
    
    # Compute error
    error = -pitch  # Desired pitch is zero
    
    # Compute PID control output
    output = compute_pid_output(error, dt)
    
    # Apply control output (e.g., adjust motor speed or servo position)
    # Example: motor_speed = base_speed + output
    
    # Print PID output for debugging
    print("Pitch: {:.2f} PID: {:.2f}".format(pitch,output))
    time.sleep(0.09)
    
    mapped_value = linear_map(output, -50,0,100,0)
    print("Mapped value:", mapped_value)
       #throttle = int(speed * 1.8)  # Map speed (0-100) to throttle (0-180)
    kit.servo[0].angle = mapped_value
  #  print(f"Speed set to {speed}%")



# Initialize ServoKit with the number of channels your ESC supports





# Example usage
original_value = 50
original_low = 0

original_high = 100
target_low = 10
target_high = 20
