#!/usr/bin/env python3


# Publishes camera frames on a topic raw_image_out from a UVC compiant camera
#
# camera_driver.py
#
# Nathan Martin



# Basic Ros2 setup
import rclpy
from rclpy.node import Node

# New imports for camera node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



#
class CameraDriver(Node):
	def __init__(self):

		# Initialize the parent class of name oscope
		super().__init__('camera_driver')

		# Declaring parameters
		self.declare_parameter('frame_id', 'camera')	
		self.declare_parameter('device_id', 0)	
		self.declare_parameter('fps', 15.0)		
		self.declare_parameter('encoding', 'bgr8')
		self.declare_parameter('frame_width', 1280)
		self.declare_parameter('frame_height', 720)

		# Pull out parameters to determine capture parameters
		self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
		dev = self.get_parameter('device_id').get_parameter_value().integer_value
		fps = self.get_parameter('fps').get_parameter_value().double_value
		self.encoding_type = self.get_parameter('encoding').get_parameter_value().string_value
		width = self.get_parameter('frame_width').get_parameter_value().integer_value
		height = self.get_parameter('frame_height').get_parameter_value().integer_value
		


		# Create a publisher, and assign it to a member variable. 
		self.pub = self.create_publisher(Image, 'raw_image_out', 10)

		# Get FPS parameter to controll timer
		timer_period = 1 / fps

		# Create a VideoCapture object
		self.capture = cv2.VideoCapture(dev, cv2.CAP_V4L2)

		if not self.capture.isOpened():
			self.get_logger().error('Failed to open camera')
			rclpy.shutdown()
			return
		
		# Set capture parameters
		self.capture.set(cv2.CAP_PROP_FPS, fps)
		self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
		self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
		self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

		# Use CV Bridge to convert between ROS 2 and OpenCV images
		self.bridge = CvBridge()

		# Throw away a few frames so the next happen without delay
		for _ in range(5):
			self.capture.grab()

		# Create a timer using the frequency parameter
		self.timer = self.create_timer(timer_period, self.callback)


	# This callback will be called every time the timer fires.
	def callback(self):
		
		# Capture frame by frame
		ret, frame = self.capture.read()
		

		if not ret:
			self.get_logger().warn('capture.read() returned False')
			return

		
		# Make an image message, and fill in the information.
		image_message = self.bridge.cv2_to_imgmsg(frame, encoding= self.encoding_type)
		image_message.header.stamp = self.get_clock().now().to_msg()
		image_message.header.frame_id = self.frame_id



		# Publish the message
		self.pub.publish(image_message)
		


# Basic ROS2 Setup function used for 1 Hz frequecy sin 
def main(args=None):
	
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.
	publisher = CameraDriver()

	# Handover to ROS2
	rclpy.spin(publisher)

	# Make sure we shutdown everything cleanly.	
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	
	main()