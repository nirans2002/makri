import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cstm_msgs.msg import Thruster
# from adafruit_servokit import ServoKit
import time


# kit = ServoKit(channels=16)
# thruster_F  = kit.servo[0]
# thruster_R  = kit.servo[1]
# thruster_L  = kit.servo[2]

# for debugging purposes
class DebugThruster():

    def __init__(self):
        self.angle = 0


class BldcPWMNode(Node):
    def __init__(self):
        super().__init__('bldc_pwm_sub')
        self.subscription = self.create_subscription(
            Thruster,
            '/thrusters',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bldcF = 0
        self.bldcR = 0
        self.bldcL= 0

        self.thruster_F  = DebugThruster()
        self.thruster_R  = DebugThruster()
        self.thruster_L  = DebugThruster()
        # self.kit = ServoKit(channels=16)
        # self.thruster_F  = self.kit.servo[0]
        # self.thruster_R  = self.kit.servo[1]
        # self.thruster_L  = self.kit.servo[2]

    def controlHardware(self):
        self.thruster_F.angle = self.bldcF*1.8
        self.thruster_R.angle = self.bldcR*1.8
        self.thruster_L.angle = self.bldcL*1.8
        self.get_logger().info(f'F: {self.thruster_F.angle} R: {self.thruster_R.angle} L: {self.thruster_L.angle}')

    # def controlHardware(self):
    #     # self.softChangePWM(self.thruster_F.angle, self.bldcF)
    #     # self.softChangePWM(thruster_R.angle, self.bldcR)
    #     # self.softChangePWM(thruster_L.angle, self.bldcL)
    #     curr_F = self.thruster_F.angle
    #     curr_R = self.thruster_R.angle
    #     curr_L = self.thruster_L.angle
        
    # # def softChangePWM(self,cur_Val, target_Val):
    #     inc = 5
    #     delay =0.2
        
    #     if curr_R < self.bldcR:
    #         for pwm in range(curr_R, int(self.bldcR)+inc, inc):
    #             # pwm += inc
    #             time.sleep(delay)
    #             self.thruster_R.angle = pwm
    #             self.get_logger().info(f'R: {self.thruster_R.angle}')
    #     if curr_R > self.bldcR:
    #         for pwm in range(curr_R, self.bldcR-inc,-inc):
    #             # pwm -= 5
    #             time.sleep(delay)
    #             self.thruster_F.angle = pwm
    #             self.get_logger().info(f'R: {self.thruster_R.angle}')
                
    #     if curr_R == self.thruster_R:
    #         pass
        
        

    def listener_callback(self,msg):
        self.bldcF = msg.thruster_f
        self.bldcR = msg.thruster_r
        self.bldcL = msg.thruster_l
        # self.calc_thruster_pwm()
        self.controlHardware()
        # self.get_logger().info(f'F: {self.bldcF} R: {self.bldcR} L: {self.bldcL}')
    
    # def calc_thruster_pwm(self):
    #     self.bldcF_s = self.bldcF*1.8
    #     self.bldcR_s = self.bldcR*1.8
    #     self.bldcL = self.bldcL*1.8
    #     # self.get_logger().info(f'F: {self.bldcF} R: {self.bldcR} L: {self.bldcL}')

    
    def calibration(self):
        pass
    #     # Calibration sequence for the ESC
    #     print("Disconnect power from the ESC, then press Enter to continue.")
    #     # input()  # Wait for user confirmation
    #     time.sleep(2)
    #     print("....")
    #     # Set the ESC to maximum throttle
    #     thruster_F.angle = 180
    #     thruster_R.angle = 180
    #     thruster_L.angle = 180
    #     time.sleep(5)
    #     print("Connect power to the ESC, wait for initialization, then press Enter.")
    #     # input()  # Wait for user confirmation
    #     time.sleep(2)
    #     print("....")
    #     # Set the ESC to minimum throttle
    #     thruster_F.angle = 0
    #     thruster_R.angle = 0
    #     thruster_L.angle = 0
    #     time.sleep(5)
    #     print("Calibration complete. You can now control the speed of the ESC.")



def main(args=None):
    rclpy.init(args=args)

    bldc_pwm = BldcPWMNode()
    # run calibration
    bldc_pwm.calibration()

    rclpy.spin(bldc_pwm)

    bldc_pwm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()