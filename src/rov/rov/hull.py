import rclpy
from rclpy.node import Node

from std_msgs.msg import String
# from custom_messages import HullConditionsMsg

import time
import board
import adafruit_dht
sensor = adafruit_dht.DHT11(board.D4)



class HullConditions(Node):
    '''publishes the conditions inside the hull like temperature, humidity etc'''
    def __init__(self):
        super().__init__('hull_conditions')
        self.publisher_ = self.create_publisher(String, 'hull_temperature', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.hull_temperature =  0.0
        self.hull_humidity = 0.0

    def timer_callback(self):
        '''callback function for the timer'''
        self.get_dht_data()
        msg = String()
        msg.data = str(self.hull_temperature)
        # msg.data.humidity = self.hull_humidity
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def get_dht_data(self):
        ''' return the temperature and the humidity from the sensor'''
        try:
        # Print the values to the serial port
            self.hull_temperature = sensor.temperature
        # temperature_f = temperature_c * (9 / 5) + 32
            self.hull_humidity = sensor.humidity
            # return temperature_c,humidity

        except RuntimeError as error:
            # Errors happen fairly often, DHT's are hard to read, just keep going
        # print(error.args[0])
        # time.sleep(2.0)
        # return None, None, None     
            pass

def main(args=None):
    rclpy.init(args=args)
    hull_onditions_pub = HullConditions()
    rclpy.spin(hull_onditions_pub)
    hull_onditions_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()