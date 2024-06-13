import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
# import the Servo, HandSignal, PalletInfo interface from the custom_interfaces package
#from custom_interfaces.msg import Servo, HandSignal, PalletInfo
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from yolov8_msgs.msg import DetectionArray
import time

class ForkliftDriver(Node):

    def __init__(self):
        # call super() in the constructor to initialize the Node object
        super().__init__('forklift_driver')
        # Logger level configuration
        rclpy.logging.set_logger_level('forklift_driver', rclpy.logging.LoggingSeverity.DEBUG)
        # create the publisher objects
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.steering_publisher = self.create_publisher(Float64, 'commands/servo/position', 10)
        #self.motor_publisher = self.create_publisher(Float64, 'commands/motor/speed', 10)
        self.servo_publisher = self.create_publisher(Float64, '/Servo', 10)
        # create subscriber objects
        #self.handsignal_subscriber = self.create_subscription(
        #    String,
        #    '/HandSignal',
        #    self.handsignal_callback,
        #    QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read some sensor data.
        self.handsignal_subscriber = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.handsignal_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        #self.pallet_subscriber = self.create_subscription(Float64,'/cam_position',self.pallet_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        #variables
        self.speed = 0.0
        self.steering = 0.0
        self.last_hand_sig = 0.0
        self.last_pallet = 0.0
        # pallet tracking
        self.is_tracking = True
        self.e = 0.0
        self.last_e = 0.0
        self.diff_e = 0.0
        self.sum_e = 0.0
        #pid constants
        self.kp = 1.0
        self.kd = 0.0
        self.ki = 0.0
        # define the timer callback
        self.timer = self.create_timer(0.02, self.timer_callback)

    def cam_pos_from_file(self):
        #file = open('/root/workspaces/148_ws/camera_info.txt')
        try:
            file = open('/root/workspaces/148_ws/camera_info.txt')
            content = file.read().strip()
            file.close()
            #self.get_logger().debug("read from file: " + str(content))
            return float(content)
        except Exception:
            return None


    def timer_callback(self):
        #cam_pos = self.cam_pos_from_file()
        #if cam_pos != None and cam_pos >= 0:
        #    self.e = (cam_pos - 320.0) / 320.0
        servo_msg = Float64()
        if self.is_tracking:
            servo_msg.data = 95.0 #fork down
            cam_pos = self.cam_pos_from_file()
            #if time.time() - self.last_pallet <= 1.0:
            if cam_pos != None and cam_pos >= 0:
                self.e = (cam_pos - 320.0) / 320.0
                # pid control
                self.diff_e = self.e - self.last_e
                self.sum_e += self.e
                self.last_e = self.e
                u = self.kp*self.e + self.kd*self.diff_e + self.ki*self.sum_e
                self.speed = 5.0
                self.steering = u
                # Display the message on the console
                self.get_logger().info('Tracking Pallet: e={}, steering={}'.format(self.e, self.steering))
            else:
                self.speed = 0.0
                self.get_logger().info('Not Tracking, no pallet info')
        else:
            servo_msg.data = 135.0 #fork up
            # clear control variables
            self.e = 0.0
            self.last_e = 0.0
            self.diff_e = 0.0
            self.sum_e = 0.0
            self.distance = 0.0
        #publish to move robot
        twist_msg = Twist()
        self.last_hand_sig = time.time()
        if time.time() - self.last_hand_sig <= 10.0: #non zero if hand signal is recent
            twist_msg.linear.x = self.speed
            twist_msg.angular.z = self.steering
        self.twist_publisher.publish(twist_msg)
        self.servo_publisher.publish(servo_msg)
        #self.get_logger().info('Publishing: {}'.format(twist_msg))

    def handsignal_callback(self, msg):
        # create messages
        #steering_msg = Float64()
        #motor_msg = Float64()
        #servo_msg = Float64()
        self.get_logger().debug("Received: {}".format(msg))
        if len(msg.detections) > 0:
            #get commands from hand signals
            detection = msg.detections[0]
            class_name = detection.class_name
            x_pos = detection.bbox.center.position.x
            y_pos = detection.bbox.center.position.y
            self.last_hand_sig = time.time()
            self.get_logger().debug("Class: {}, x: {}, y: {}".format(class_name, x_pos, y_pos))
            #motor rpm
            #motor_msg.data = (240.0 - y_pos) * 10.0
            #self.motor_publisher.publish(motor_msg)
            #fork up/down
            if not class_name == "Closed":
                #servo_msg.data = 45.0 #fork down
                #self.servo_publisher.publish(servo_msg)
                self.is_tracking = True
            else:
                #servo_msg.data = 135.0 #fork up
                #self.servo_publisher.publish(servo_msg)
                self.is_tracking = False
            #motor rpm
            self.speed = (240.0 - y_pos) / 20.0
            #steering
            self.steering = (x_pos-320) / 320.0
            if -0.05 < self.steering and self.steering < 0.05:
                self.steering = 0.0 #center deadzone for going straght
                #self.steering_publisher.publish(steering_msg)
        # check the hand signal message
        #if msg.data.upper() == 'FORWARD':
        #    # motion
        #    motor_msg.data = 1000.0 #1000 rpm
        #    self.motor_publisher.publish(motor_msg)
        #    if not self.is_tracking:
        #        steering_msg.data = 0.5 #go straight
        #        self.steering_publisher.publish(steering_msg)
        #    # print debug info
        #    self.get_logger().debug('Received: {}. Going Forward'.format(msg.data))
        #elif msg.data.upper() == 'BACKWARD':
        #    # motion
        #    motor_msg.data = -1000.0 #-1000 rpm
        #    self.motor_publisher.publish(motor_msg)
        #    if not self.is_tracking:
        #        steering_msg.data = 0.5 #go straight
        #        self.steering_publisher.publish(steering_msg)
        #    # print debug info
        #    self.get_logger().debug('Received: {}. Going Backward'.format(msg.data))
        #elif msg.data.upper() == 'LEFT':
        #    # motion
        #    steering_msg.data = 0.25 #go left
        #    motor_msg.data = 1000.0 #1000 rpm
        #    self.steering_publisher.publish(steering_msg)
        #    self.motor_publisher.publish(motor_msg)
        #    # print debug info
        #    self.get_logger().debug('Received: {}. Going Left'.format(msg.data))
        #elif msg.data.upper() == 'RIGHT':
        #    # motion
        #    steering_msg.data = 0.75 #go right
        #    motor_msg.data = 1000.0 #1000 rpm
        #    self.steering_publisher.publish(steering_msg)
        #    self.motor_publisher.publish(motor_msg)
        #    # print debug info
        #    self.get_logger().debug('Received: {}. Going Right'.format(msg.data))
        #elif msg.data.upper() == 'DOWN':
        #    # motion
        #    servo_msg.data = 30.0 #fork down
        #    self.servo_publisher.publish(servo_msg)
        #    self.is_tracking = True
        #    # print debug info
        #    self.get_logger().debug('Received: {}. Fork Down'.format(msg.data))
        #elif msg.data.upper() == 'UP':
        #    # motion
        #    servo_msg.data = 90.0 #fork up
        #    self.servo_publisher.publish(servo_msg)
        #    self.is_tracking = False
        #    # print debug info
        #    self.get_logger().debug('Received: {}. Fork Up'.format(msg.data))
        #else:
        #    # warning for invalid hand signal string
        #    self.get_logger().warn('Invalid Signal: {}'.format(msg.data))
        #    return None # exit the function
        # publish things
        #self.steering_publisher.publish(steering_msg)
        #self.motor_publisher.publish(motor_msg)
        #self.servo_publisher.publish(servo_msg)
        return None

    def pallet_callback(self, msg):
        self.e = (msg.data - 320.0) / 320.0
        self.last_pallet = time.time()
        #self.distance = msg.distance
        self.get_logger().debug('Pallet Callback received: {}.'.format(msg))

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    forklift_driver = ForkliftDriver()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(forklift_driver)
    # Explicity destroys the node
    forklift_driver.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
