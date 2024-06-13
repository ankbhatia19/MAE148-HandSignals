import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Servo module from custom interface
from std_msgs.msg import Float64
#from custom_interfaces.msg import Servo
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
# serioal module for serial communication with arduino
import serial
# time module
import time



#serial port for the Arduino that drives the servo
#ARDUINO_PORT = "/dev/ttyACM1"
ARDUINO_PORT = "/dev/ttyUSB0"


class ServoControl(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('servo_control')
        # logger level
        rclpy.logging.set_logger_level('servo_control', rclpy.logging.LoggingSeverity.DEBUG)
        # create the subscriber object
        # in this case, the subscriptor will be subscribed on /servo topic with a queue size of 10 messages.
        # use the Servo module for /servo topic
        # send the received info to the listener_callback method.
        self.subscriber = self.create_subscription(
            Float64,
            '/Servo',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read data.
        # create srrial port for communication with the arduino
        self.serial_port = serial.Serial(ARDUINO_PORT, 115200)

    def listener_callback(self, msg):
        # get data
        servo_angle = msg.data
        # print the log info in the terminal
        self.get_logger().debug('Sending to serial port: {}'.format(servo_angle))
        self.serial_port.write((str(servo_angle)+'\n').encode())
        # print response from arduino
        time.sleep(0.1)
        if self.serial_port.in_waiting > 0: #see if there are content in serial port
            self.get_logger().debug('Arduino Response: {}'.format(
                                   self.serial_port.read(self.serial_port.in_waiting).decode("utf-8")))


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    servo_control = ServoControl()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(servo_control)
    # Explicity destroy the node
    servo_control.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
