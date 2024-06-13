import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from std_msgs.msg import Float64
#
from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np

class CameraTest(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('camera_test')
        #
        rclpy.logging.set_logger_level('camera_test', rclpy.logging.LoggingSeverity.DEBUG)
        # create the publisher object
        # in this case, the publisher will publish on /cmd_vel Topic with a queue size of 10 messages.
        # use the Twist module for /cmd_vel Topic
        self.x_pos_publisher = self.create_publisher(Float64, '/cam_position', 10)
        # create a timer sending two parameters:
        # - the duration between 2 callbacks (0.5 seconds)
        # - the timer function (timer_callback)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.x_position = 0.0
        self.confidence = 0.0
        #camera
        self.get_logger().info('Starting oak camera')
        self.rf = RoboflowOak(model="pallet-detection-eavuo", confidence=0.05, overlap=0.5,
                              version="2", api_key="Q7tRq94bZcAdlwh5xN5Q", rgb=True,
                              depth=True, device=None, blocking=True)
        self.get_logger().info('Camera started')

    def timer_callback(self):
        # camera
        result, frame, raw_frame, depth = self.rf.detect()
        predictions = result["predictions"]
        if len(predictions) > 0:
            self.x_position = predictions[0].x
            self.confidence = predictions[0].confidence
        # Here you have the callback method
        msg = Float64()
        msg.data = self.x_position
        if self.confidence >= 0.5:
            self.x_pos_publisher.publish(msg)
            self.get_logger().info('Confidence={}, Publishing:{}'.format(self.confidence, msg))
        else:
            self.get_logger().info('Low confidence, not publishing')

    def publish_x_pos(self, x_pos):
        msg = Float64()
        msg.data = 350.0
        self.x_pos_publisher.publish(msg)
        # Display the message on the console
        self.get_logger().debug("Print: ".format(x_pos))


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    camera_test = CameraTest()

    depth_info = ""
    # instantiating an object (rf) with the RoboflowOak module
    #rf = RoboflowOak(model="pallet-detection-eavuo", confidence=0.05, overlap=0.5,
    #version="2", api_key="VDGDzv7hcvWTuN7bF3PB", rgb=True,
    #depth=True, device=None, blocking=True)
    # Running our model and displaying the video output with detection
    if True:
        pass
        #t0 = time.time()
        # The rf.detect() function runs the model inference
        #result, frame, raw_frame, depth = rf.detect()
        #predictions = result["predictions"]
        #{
        #    predictions:
        #    [ {
        #        x: (middle),
        #        y:(middle),
        #        width:
        #        height:
        #        depth: ###->
        #        confidence:
        #        class:
        #        mask: {
        #    ]
        #}
        #frame - frame after preprocs, with predictions
        #raw_frame - original frame from your OAK
        #depth - depth map for raw_frame, center-rectified to the center camera
        
        # timing: for benchmarking purposes
        #t = time.time()-t0
        #print("FPS ", 1/t)
        #print("PREDICTIONS ", [p.json() for p in predictions])

        # setting parameters for depth calculation
        # comment out the following 2 lines out if you're using an OAK without Depth
        #max_depth = np.amax(depth)
        # cv2.imshow("depth", depth/max_depth)
        # displaying the video feed as successive frames
        #cv2.imshow("frame", frame)
        #camera_test.publish_x_pos(-1.23)
        # publish x_position 
        #if len(predictions) > 0:
        #    #camera_test.publish_x_pos(predictions[0].x)
        #    camera_test.x_position = predictions[0].x
        #    camera_test.confidence = predictions[0].confidence
        # ros2 node
        #rclpy.spin_once(camera_test)
    rclpy.spin(camera_test)
    # Explicity destroys the node
    camera_test.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
