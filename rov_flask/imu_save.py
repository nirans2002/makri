import csv
import time
import datetime
import smbus2 as smbus
import os

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
TEMP_OUT_H = 0x41
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
i2c = smbus.SMBus(1)  # Use SMBus(0) for old versions of Raspberry Pi

# Offsets for calibration
ACCEL_X_OFFSET = -262.38
ACCEL_Y_OFFSET = 115.34
ACCEL_Z_OFFSET = 17827.048
GYRO_X_OFFSET = -187.79
GYRO_Y_OFFSET = 222.505
GYRO_Z_OFFSET = -107.241
# Function to initialize MPU6050 sensor
def init_mpu6050(i2c, address=0x68):
    i2c.write_byte_data(address, PWR_MGMT_1, 0)
    time.sleep(0.1)
    i2c.write_byte_data(address, SMPLRT_DIV, 7)
    i2c.write_byte_data(address, CONFIG, 0)
    i2c.write_byte_data(address, GYRO_CONFIG, 0)
    i2c.write_byte_data(address, ACCEL_CONFIG, 0)

# Function to read raw data from MPU6050
def read_raw_data(i2c, addr, address=0x68):
    high = i2c.read_byte_data(address, addr)
    low = i2c.read_byte_data(address, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

# Function to get MPU6050 data
def get_mpu6050_data(i2c):
    temp = read_raw_data(i2c, TEMP_OUT_H) / 340.0 + 36.53
    accel_x = (read_raw_data(i2c, ACCEL_XOUT_H) - ACCEL_X_OFFSET) / 16384.0
    accel_y = (read_raw_data(i2c, ACCEL_XOUT_H + 2) - ACCEL_Y_OFFSET) / 16384.0
    accel_z = (read_raw_data(i2c, ACCEL_XOUT_H + 4) - ACCEL_Z_OFFSET) / 16384.0
    gyro_x = (read_raw_data(i2c, GYRO_XOUT_H) - GYRO_X_OFFSET) / 131.0
    gyro_y = (read_raw_data(i2c, GYRO_XOUT_H + 2) - GYRO_Y_OFFSET) / 131.0
    gyro_z = (read_raw_data(i2c, GYRO_XOUT_H + 4) - GYRO_Z_OFFSET) / 131.0

    return {
        'temp': temp,
        'accel_x': accel_x,
        'accel_y': accel_y,
        'accel_z': accel_z,
        'gyro_x': gyro_x,
        'gyro_y': gyro_y,
        'gyro_z': gyro_z
    }

def save_to_csv(data, filename):
    file_exists = os.path.isfile(filename) and os.path.getsize(filename) > 0
    with open(filename, 'a', newline='') as csvfile:
        fieldnames = ['time', 'temp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        # Write header only if file is empty
        if not file_exists:
            writer.writeheader()

        # Write data with timestamp
        runtime_seconds = (time.time() - start_time)

        # Write data with runtime in seconds
        data['time'] = runtime_seconds
        writer.writerow(data)
# Main code
if __name__ == "__main__":
    # Initialize MPU6050 sensor
    init_mpu6050(i2c)
    start_time = time.time()
    # Continuously read and save data
    while True:
        imu_data = get_mpu6050_data(i2c)
        save_to_csv(imu_data, 'imu_data.csv')
        time.sleep(1)  # Adjust sleep duration as needed
