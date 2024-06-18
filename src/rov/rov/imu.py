import rclpy
from rclpy.node import Node
from cstm_msgs.msg import Thruster
from std_msgs.msg import String
from mpu6050 import mpu6050
import time,math
from adafruit_servokit import ServoKit

mpu6050= mpu6050(0x68)

def linear_map(self,value, from_low, from_high, to_low, to_high):
    # Ensure the value is within the original range
    value = max(min(value, from_high), from_low)
    # Map the value to the target range
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

class ImuControlNode(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Thruster, '/thrusters', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.thrusterF = 0
         
        self.pitch = 0
        self.roll = 0
        self.prev_time = time.monotonic_ns()

        self.Kp = 1.0  # Proportional gain
        self.Ki = 0.1  # Integral gain
        self.Kd = 0.1  # Derivative gain

        self.integral = 0
        self.prev_error = 0


    def timer_callback(self):
        self.data = self.read_sensor_data()
        print(self.data)
        self.curr_time = time.monotonic_ns()
        self.dt = (self.curr_time - self.prev_time) / 1000000000
    
        # self.tilt_x, self.tilt_y, self.tilt_z = self.calculate_tilt_angles()
        # self.pitch, self.roll = self.complementary_filter(self.pitch, self.roll, self.data[0], self.dt)
    
        # self.prev_time = self.curr_time
        # time.sleep(0.01)   
            # Compute error
        # self.error = -self.pitch  # Desired pitch is zero

         
        # Compute PID control output
        # self.output = self.compute_pid_output(self.error, self.dt)
        
        # Apply control output (e.g., adjust motor speed or servo position)
        # Example: motor_speed = base_speed + output
        
        # Print PID output for debugging
        # print("Pitch: {:.2f} PID: {:.2f}".format(pitch,output))
        # time.sleep(0.09)
        
        # self.mapped_value = self.linear_map(self.output, -50,0,100,0)
        # print("Mapped value:", mapped_value)
           #throttle = int(speed * 1.8)  # Map speed (0-100) to throttle (0-180)
        # kit.servo[0].angle = mapped_value
        # print(f"Speed set to {speed}%")
#         msg = Thruster()
#         msg.thruster_f = self.mapped_value
#         self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.thruster_f)

    

# Define a function to read the sensor data
    def read_sensor_data(self):
        # Read the accelerometer values
        self.accel_data = mpu6050.get_accel_data()

        # Read the gyroscope values
        self.gyro_data = mpu6050.get_gyro_data()

        # Read temp
        self.temperature = mpu6050.get_temp()


        # return self.accelerometer_data, self.gyroscope_data, self.temperature

    def calculate_tilt_angles(self):
        self.x, self.y, self.z = self.accel_data['x'], self.accel_data['y'], self.accel_data['z']
        self.tilt_x = math.atan2(self.y, math.sqrt(self.x * self.x + self.z * self.z)) * 180 / math.pi
        self.tilt_y = math.atan2(-self.x, math.sqrt(self.y * self.y + self.z * self.z)) * 180 / math.pi
        self.tilt_z = math.atan2(self.z, math.sqrt(self.x * self.x + self.y * self.y)) * 180 / math.pi
    
        # return tilt_x, tilt_y, tilt_z
    
    def complementary_filter(self,pitch, roll, gyro_data, dt, alpha=0.98):
        self.pitch += self.gyro_data['x'] * dt
        self.roll -= self.gyro_data['y'] * dt
        self.pitch = alpha * self.pitch + (1 - alpha) * math.atan2(self.gyro_data['y'], math.sqrt(self.gyro_data['x'] * self.gyro_data['x'] + self.gyro_data['z'] * self.gyro_data['z'])) * 180 / math.pi
        self.roll = alpha * self.roll + (1 - alpha) * math.atan2(-self.gyro_data['x'], math.sqrt(self.gyro_data['y'] * self.gyro_data['y'] + self.gyro_data['z'] * self.gyro_data['z'])) * 180 / math.pi
        # return pitch, roll

    # Function to compute PID control output
    def compute_pid_output(self,dt):
        self.integral, self.prev_error
        # Proportional term
        self.proportional = self.Kp * self.error
        # Integral term
        self.integral += self.error * dt
        self.integral_term = self.Ki * self.integral
        # Derivative term
        self.derivative = (self.error - self.prev_error) / dt
        self.derivative_term = self.Kd * self.derivative
        # Update previous error
        self.prev_error = self.error

        # Compute PID output
        self.output = self.proportional + self.integral_term + self.derivative_term
        # return output


def main(args=None):
    rclpy.init(args=args)
    imu_controller = ImuControlNode()
    rclpy.spin(imu_controller)
    imu_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
