# from machine import Pin, I2C
import smbus2 as smbus
import time
# import utime
import math
from mpu6050 import init_mpu6050, get_mpu6050_data
 
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
i2c = smbus.SMBus(1)

 
i2c = i2c(0, scl=Pin(3), sda=Pin(2), freq=400000)
init_mpu6050(i2c)
 
pitch = 0
roll = 0
prev_time = utime.ticks_ms()
 
while True:
    data = get_mpu6050_data(i2c)
    curr_time = utime.ticks_ms()
    dt = (curr_time - prev_time) / 1000
 
    tilt_x, tilt_y, tilt_z = calculate_tilt_angles(data['accel'])
    pitch, roll = complementary_filter(pitch, roll, data['gyro'], dt)
  
    prev_time = curr_time
 
    #print("Temperature: {:.2f} °C".format(data['temp']))
    #print("Tilt angles: X: {:.2f}, Y: {:.2f}, Z: {:.2f} degrees".format(tilt_x, tilt_y, tilt_z))
    #print("Pitch: {:.2f}".format(pitch))
    #print("Acceleration: X: {:.2f}, Y: {:.2f}, Z: {:.2f} g".format(data['accel']['x'], data['accel']['y'], data['accel']['z']))
    #print("Gyroscope: X: {:.2f}, Y: {:.2f}, Z: {:.2f} °/s".format(data['gyro']['x'], data['gyro']['y'], data['gyro']['z']))
    time.sleep(.1)   
    
    Kp = 1.0  # Proportional gain
    Ki = 0.1  # Integral gain
    Kd = 0.1  # Derivative gain

# Initialize variables for PID controller
    integral = 0
    prev_error = 0
    
    # Code to get pitch from complementary filter
    # Assume pitch is stored in a variable named 'pitch'
    
    # Compute error
    error = -pitch  # Desired pitch is zero
    
    # Compute PID control output
    output = compute_pid_output(error, dt)
    
    # Apply control output (e.g., adjust motor speed or servo position)
    # Example: motor_speed = base_speed + output
    
    # Print PID output for debugging
    print("Pitch: {:.2f} PID: {:.2f}".format(pitch,output))
    utime.sleep(.5)

