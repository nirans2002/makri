import smbus
import time

bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

MPU6050_ADDR = 0x68

def read_word(reg):
    high = bus.read_byte_data(MPU6050_ADDR, reg)
    low = bus.read_byte_data(MPU6050_ADDR, reg + 1)
    val = (high << 8) + low
    return val

def read_word_2c(reg):
    val = read_word(reg)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def calibrate():
    print("Calibrating MPU6050...")
    accel_x_offset = 0
    accel_y_offset = 0
    accel_z_offset = 0
    gyro_x_offset = 0
    gyro_y_offset = 0
    gyro_z_offset = 0

    num_samples = 1000  # Adjust the number of samples as needed
    
    for _ in range(num_samples):
        accel_x_offset += read_word_2c(0x3b)
        accel_y_offset += read_word_2c(0x3d)
        accel_z_offset += read_word_2c(0x3f)
        gyro_x_offset += read_word_2c(0x43)
        gyro_y_offset += read_word_2c(0x45)
        gyro_z_offset += read_word_2c(0x47)
        time.sleep(0.01)  # Adjust sleep time if needed

    accel_x_offset /= num_samples
    accel_y_offset /= num_samples
    accel_z_offset /= num_samples
    gyro_x_offset /= num_samples
    gyro_y_offset /= num_samples
    gyro_z_offset /= num_samples

    print("Calibration complete.")
    print("Accelerometer offsets (X, Y, Z):", accel_x_offset, accel_y_offset, accel_z_offset)
    print("Gyroscope offsets (X, Y, Z):", gyro_x_offset, gyro_y_offset, gyro_z_offset)

if __name__ == "__main__":
    calibrate()
