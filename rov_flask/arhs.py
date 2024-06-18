import csv
import time
import smbus2 as smbus
import os

# MPU6050 Registers
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
TEMP_OUT_H = 0x41
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# HMC5883L Registers
HMC5883L_ADDR = 0x77
HMC5883L_CONFIG_A = 0xF4
HMC5883L_DATA_X_MSB = 0xF6

# BMP180 Registers
BMP180_ADDR = 0x77
BMP180_CTRL_MEAS = 0xF4
BMP180_DATA = 0xF6

# Offsets for calibration
ACCEL_X_OFFSET = -262.38
ACCEL_Y_OFFSET = 115.34
ACCEL_Z_OFFSET = 17827.048
GYRO_X_OFFSET = -187.79
GYRO_Y_OFFSET = 222.505
GYRO_Z_OFFSET = -107.241

# Initialize I2C
i2c = smbus.SMBus(1)  # Use SMBus(0) for old versions of Raspberry Pi

# Function to initialize MPU6050 sensor
def init_mpu6050(i2c, address=0x68):
    i2c.write_byte_data(address, PWR_MGMT_1, 0)
    time.sleep(0.1)
    i2c.write_byte_data(address, SMPLRT_DIV, 7)
    i2c.write_byte_data(address, CONFIG, 0)
    i2c.write_byte_data(address, GYRO_CONFIG, 0)
    i2c.write_byte_data(address, ACCEL_CONFIG, 0)

# Function to read raw data from sensor
def read_raw_data(i2c, addr, address):
    high = i2c.read_byte_data(address, addr)
    low = i2c.read_byte_data(address, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

# Function to get sensor data
def get_sensor_data(i2c):
    # MPU6050 data
    accel_x = (read_raw_data(i2c, ACCEL_XOUT_H, 0x68) - ACCEL_X_OFFSET) / 16384.0
    accel_y = (read_raw_data(i2c, ACCEL_XOUT_H + 2, 0x68) - ACCEL_Y_OFFSET) / 16384.0
    accel_z = (read_raw_data(i2c, ACCEL_XOUT_H + 4, 0x68) - ACCEL_Z_OFFSET) / 16384.0
    gyro_x = (read_raw_data(i2c, GYRO_XOUT_H, 0x68) - GYRO_X_OFFSET) / 131.0
    gyro_y = (read_raw_data(i2c, GYRO_XOUT_H + 2, 0x68) - GYRO_Y_OFFSET) / 131.0
    gyro_z = (read_raw_data(i2c, GYRO_XOUT_H + 4, 0x68) - GYRO_Z_OFFSET) / 131.0

    # HMC5883L data
    data_x = read_raw_data(i2c, HMC5883L_DATA_X_MSB, HMC5883L_ADDR)
    
    # BMP180 data
    data = i2c.read_i2c_block_data(BMP180_ADDR, BMP180_DATA, 2)
    pressure_raw = (data[0] << 8) + data[1]

    return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, data_x, pressure_raw

# Function to save data to CSV file with timestamp
def save_to_csv(data, filename, start_time):
    with open(filename, 'a', newline='') as csvfile:
        fieldnames = ['timestamp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'mag_x', 'pressure']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        # Write header only if file is empty
        if os.path.getsize(filename) == 0:
            writer.writeheader()

        # Calculate runtime in seconds
        runtime_seconds = time.time() - start_time

        # Write data with runtime in seconds
        data['timestamp'] = runtime_seconds
        writer.writerow(data)

# Main code
if __name__ == "__main__":
    # Initialize MPU6050 sensor
    init_mpu6050(i2c)

    # Record start time
    start_time = time.time()

    # Continuously read and save data
    while True:
        # Read sensor data
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, pressure = get_sensor_data(i2c)
        
        # Save data to CSV
        save_to_csv({'accel_x': accel_x, 'accel_y': accel_y, 'accel_z': accel_z,
                     'gyro_x': gyro_x, 'gyro_y': gyro_y, 'gyro_z': gyro_z,
                     'mag_x': mag_x, 'pressure': pressure}, 'ahrs_data.csv', start_time)

        # Adjust sleep duration as needed
        time.sleep(0.1)
