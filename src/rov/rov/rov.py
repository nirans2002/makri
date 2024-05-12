import rclpy
from rclpy.node import Node
from cstm_msgs.msg import Thruster
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math


constrain = lambda n, minn, maxn: max(min(maxn, n), minn)

class RovControl(Node):

    def __init__(self):
        super().__init__('Rov_controller')
        self.thruster_velocity = self.create_publisher(Thruster, '/thrusters', 10)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_listen_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.thrusterVel_F,self.thrusterVel_R,self.thrusterVel_L = 0,0,0

    def cmd_vel_listen_callback(self, msg):
        '''listen to the cmd_vel topic and write the pwm to the escs'''
        self.x_vel = msg.linear.x
        self.y_angl = msg.angular.y
        self.theta = msg.angular.z
        # self.pitch = msg.angular.x
        # self.get_logger().info(f'x: {self.x_vel} y:{self.y_vel} \n z:{self.theta} p:{self.pitch}' )
        self.calc_thruster_vel()
    
    def calc_thruster_vel(self):
        '''calculate the thruster velocities from the cmd_vel'''
        self.thrusterVel_F = math.floor(self.y_angl*5)
        self.thrusterVel_R = constrain(math.ceil(self.x_vel*1 - self.theta*1)*5,0,100 )
        self.thrusterVel_L = constrain(math.ceil(self.x_vel*1 + self.theta*1)*5,0,100 )
        self.publishThrusterVel()
        self.get_logger().info(f'F: {self.thrusterVel_F} R: {self.thrusterVel_R} L: {self.thrusterVel_L}')

    def publishThrusterVel(self):
        # self.get_logger().info(f'debug: publishThrusterVel')
        msg = Thruster()
        msg.thruster_f,msg.thruster_r,msg.thruster_l = self.thrusterVel_F,self.thrusterVel_R,self.thrusterVel_L
        self.get_logger().info(f'F: {msg.thruster_f} R: {msg.thruster_r} L: {msg.thruster_l}')
        self.thruster_velocity.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)



def main(args=None):
    rclpy.init(args=args)
    rov_controller = RovControl()
    rclpy.spin(rov_controller)
    rov_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()