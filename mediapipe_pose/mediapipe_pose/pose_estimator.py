#!/usr/bin/env python3


# pose estimator runs MediaPipe's BlazePose on images and republishes pose landmarks
#
# pose_estimator.py
#
# Nathan Martin
#




# Basic Ros2 setup
import rclpy
from rclpy.node import Node

# Import Custom pose msgs
from pose_interfaces.msg import Landmark, PoseLandmarks
from pose_interfaces.srv import EnablePose

# New imports for camera node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Mediapipe import
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

class MediaPipePoseNode(Node):
	"""
	Subscribes to camera images, runs MediaPipe BlazePose, 
	and republishes both raw landmarks and a debug overlay.
	"""

	def __init__(self):
			# Initialize the parent class.
			super().__init__('mediapipe_pose')

			# Pose model parameters
			self.declare_parameter('model_complexity', 1)                 # 0=lite, 1=full, 2=heavy
			self.declare_parameter('min_detection_confidence', 0.5)      # ≥ this to accept a new detection
			self.declare_parameter('min_tracking_confidence', 0.5)       # ≥ this to continue tracking

			# Enable/disable flag (can also be set via your EnablePose.srv)
			self.declare_parameter('enable_pose', True)

			# Topic names (so you can remap without code changes)
			self.declare_parameter('input_image_topic', '/raw_image_out')
			self.declare_parameter('pose_topic', '/pose/landmarks')

			# We're not publishing this single landmark
			self.declare_parameter('single_landmark_topic', '/pose/landmark')

			# Rate at which we want to publish in seconds
			self.declare_parameter('publish_rate_hz', 1.0)

			# Read them back into members
			complexity = self.get_parameter('model_complexity').value
			det_conf   = self.get_parameter('min_detection_confidence').value
			track_conf = self.get_parameter('min_tracking_confidence').value
			self.pose_enabled = self.get_parameter('enable_pose').value

			in_topic   = self.get_parameter('input_image_topic').value
			out_topic  = self.get_parameter('pose_topic').value
			single_tm  = self.get_parameter('single_landmark_topic').value

			desired_s = self.get_parameter('publish_rate_hz').value

			# Now use those when creating your subscribers/publishers/services
			self.pose_sub = self.create_subscription(Image, in_topic, self.image_callback, 10)

			self.pose_pub = self.create_publisher(PoseLandmarks, out_topic, 10)

			self.land_pub = self.create_publisher(Landmark, single_tm, 10)

			# Publisher for the overlaid image
			self.debug_pub = self.create_publisher(Image, 'pose/debug_image', 10)

			# Service for enabling the pose estimator
			self.enable_srv = self.create_service(EnablePose, 'enable_pose', self.service_callback)

			# Initialize MediaPipe with the read parameters
			mp_pose = mp.solutions.pose
			self.pose = mp_pose.Pose(
				static_image_mode=False,
				model_complexity=complexity,
				min_detection_confidence=det_conf,
				min_tracking_confidence=track_conf
			)

			self.bridge = CvBridge()

			self.last_pub_time = self.get_clock().now()
			self.publish_period = desired_s


	





	def image_callback(self, msg: Image):
		if not self.pose_enabled:
			return

		# Convert ROS→OpenCV
		try:
			cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		except CvBridgeError as e:
			self.get_logger().error(f'Bridge error: {e}')
			return

		# Run BlazePose
		cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
		results = self.pose.process(cv_rgb)
		if not results.pose_landmarks:
			return

		# 2) Draw landmarks on the OpenCV image
		annotated = cv_bgr.copy()
		try:
			mp_drawing.draw_landmarks(
				annotated,
				results.pose_landmarks,
				mp.solutions.pose.POSE_CONNECTIONS,
				landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style()
			)
		except Exception as e:
			self.get_logger().error(f'Error drawing landmarks: {e}')

		# 3) Convert annotated image back to ROS and publish
		try:
			debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
			debug_msg.header = msg.header
			self.debug_pub.publish(debug_msg)
		except CvBridgeError as e:
			self.get_logger().error(f'Bridge error: {e}')

		# This throttles publication to a specific rate
		now = self.get_clock().now()
		elapsed = (now - self.last_pub_time).nanoseconds * 1e-9
		if elapsed < self.publish_period:
			return
		
		self.last_pub_time = now

		# Publish the landmark message 
		pose_msg = PoseLandmarks()
		pose_msg.header = msg.header
		pose_msg.header.frame_id = 'camera_optical_frame' 

		world = results.pose_world_landmarks
		if not world:
			return

		for lm in world.landmark:
			pose_msg.landmarks.append(
				Landmark(
					x=lm.x,
					y=lm.y,
					z=lm.z,
					visibility=lm.visibility
				)
			)

		self.pose_pub.publish(pose_msg)


	def service_callback(self, request, response):
		# Update internal flag
		self.pose_enabled = request.enable

		# Always succeed (or add your own checks here)
		response.success = True
		response.message = (
			f"Pose detection {'enabled' if self.pose_enabled else 'disabled'}"
		)

		# Log once at INFO
		self.get_logger().info(response.message)
		return response



# This is a entry point.	
def main(args=None):

	# Initialize rclpy. 
	rclpy.init(args=args)

	# Make a node class.
	pose = MediaPipePoseNode()

	# Handover to ROS2
	rclpy.spin(pose)

	# Clean Shutdown
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':

	main()
