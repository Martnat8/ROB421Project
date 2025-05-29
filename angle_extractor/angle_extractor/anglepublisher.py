#!/usr/bin/env python3

# angleextractor.py
#
# Nathan Martin
#
# This node subscribes to a mediapipe landmark topic and calculates angles for SAMI positioning


import rclpy
import json
from rclpy.node import Node

import math
import numpy as np

from pose_interfaces.msg import PoseLandmarks
from std_msgs.msg import String



class AnglePublisher(Node):
	'''
	This node subscribes to a topic publishing Mediapipe landmark nodes, calculates the angles for joints
	on the SAMI robot and republishes them.
	'''
	def __init__(self):
		# Initialize the parent class.
		super().__init__('AnglePublisher')

		# Create the subscriber
		self.sub = self.create_subscription(PoseLandmarks, '/pose/landmarks', self.callback, 10)

		# Create the publisher (CHECK THIS OUTPUT)
		self.pub = self.create_publisher(String, '/joint_angles', 10)


	# This callback will be called whenever we receive a new message on the topic.
	def callback(self, msg: PoseLandmarks):

		# Extract landmarks from message
		lm = msg.landmarks

		angles = {}

		# Calculate elbow angles
		# Left side: shoulder=11, elbow=13, wrist=15
		# Right side: shoulder=12, elbow=14, wrist=16
		angles['LeftElbow']  = self.compute_angle(lm[11], lm[13], lm[15])
		angles['RightElbow'] = self.compute_angle(lm[12], lm[14], lm[16])

		# Calculate elbow rotation using plane normals (twist about upper arm axis)
		# Left side: shoulder=11, elbow=13, wrist=15
		# Using: A=wrist(15), B=elbow(13), C=shoulder(11), D=hip(23)
		# angles['LeftElbow'] = self.compute_rotation(lm[15], lm[13], lm[11], lm[23])

		# # Right side: shoulder=12, elbow=14, wrist=16
		# # Using: A=wrist(16), B=elbow(14), C=shoulder(12), D=hip(24)
		# angles['RightElbow'] = self.compute_rotation(lm[16], lm[14], lm[12], lm[24])



		# Calculate shoulder angles
		# Left side: shoulder=11, elbow=13, hip=23
		# Right side: shoulder=12, elbow=14, hip=24
		angles['LeftShoulder']  = self.compute_angle(lm[13], lm[11], lm[23])
		angles['RightShoulder'] = self.compute_angle(lm[14], lm[12], lm[24])



		# Calculate bicep rotation angle
		# Left side: hip=23, shoulder=11, elbow=13, wrist=15
		# Right side: hip=24, shoulder=12, elbow=14, wrist=16
		angles['LeftBicep']  = self.compute_rotation(lm[23], lm[11], lm[13], lm[15])
		angles['RightBicep'] = self.compute_rotation( lm[24], lm[12], lm[14], lm[16])

		# Calculate chest rotation angle
		# Left side: opposite_shoulder=12, shoulder=11, hip=23, elbow=13
		# Right side: opposite_shoulder=11, shoulder=12, hip=24, elbow=14
		angles['LeftChest']  = self.compute_rotation(lm[12], lm[11], lm[23], lm[13])
		angles['RightChest'] = self.compute_rotation(lm[11], lm[12], lm[24], lm[14])

        # Convert all angles to integers
		angles = {joint: int(angle) for joint, angle in angles.items()}

		# Publish angles
		out_msg = String()
		out_msg.data = json.dumps(angles)
		self.pub.publish(out_msg)


	# Pass in two vectors who share a point B, calculate inner angle between
	def compute_angle(self, A, B, C):

		# Calculate vectors from landmark points
		vAB = np.array([A.x, A.y, A.z]) - np.array([B.x, B.y, B.z])
		vCB = np.array([C.x, C.y, C.z]) - np.array([B.x, B.y, B.z])
		
		# cosine rule to find the interrior angle between two vectors
		cos_theta = np.dot(vAB, vCB) / (np.linalg.norm(vAB) * np.linalg.norm(vCB))
	
		# Clamp to prevent floating point errors
		cos_theta = max(-1.0, min(1.0, cos_theta))

		# Return the angle
		theta = math.degrees(math.acos(cos_theta))
		return theta
	

	# Used to calculate the angle of twist of SAMI bicep by using two planes and 
	# the angle between their normals.
	# TODO: clamp 

	def compute_rotation(self, A, B, C, D):

		# Extract 3D points from landmarks
		p_A = np.array([A.x, A.y, A.z])
		p_B = np.array([B.x, B.y, B.z])
		p_C = np.array([C.x, C.y, C.z])
		p_D = np.array([D.x, D.y, D.z])

		# Build shared axis vector
		vBC = p_C - p_B

		# Two more vectors to define planes
		vBA = p_A - p_B
		vCD = p_D - p_C

		# Create normal vectors to planes defined by vectors
		n1 = np.cross(vBC, vBA)
		n2 = np.cross(vBC, vCD)

		# Normalize normal vectors
		n1_norm = np.linalg.norm(n1)
		n2_norm = np.linalg.norm(n2)

		if n1_norm == 0 or n2_norm == 0:
			return 0.0
		n1 /= n1_norm
		n2 /= n2_norm

		# Compute the angle between the two normals
		cos_phi = np.dot(n1,n2)
		cos_phi = max(-1.0, min(1.0, cos_phi))
		phi = math.degrees(math.acos(cos_phi))

		# cross(n1,n2) points along vBC if the twist is positive
		cross_n1_n2 = np.cross(n1, n2)

		# Dot product points along (pos) or opposite (neg) depending on the rotation
		sign = np.sign(np.dot(vBC / np.linalg.norm(vBC), cross_n1_n2))
		# multiply magnitude by sign
		return sign * phi


	# Calculate the angle between a vector and the normal of a plane 
	def vector_plane(self, A, B, C, D, E):

		# Extract 3D points from landmarks
		p_A = np.array([A.x, A.y, A.z])
		p_B = np.array([B.x, B.y, B.z])
		p_C = np.array([C.x, C.y, C.z])
		p_D = np.array([D.x, D.y, D.z])
		p_E = np.array([E.x, E.y, E.z])

		# Build the vector
		v_AB = p_A - p_B

		# Find the normal of the plane
		v_BC = p_B - p_C
		v_CD = p_C - p_D
		V_DE



# This is a entry point.	
def main(args=None):

	# Initialize rclpy. 
	rclpy.init(args=args)

	# Make a node class.
	angles = AnglePublisher()

	# Handover to ROS2
	rclpy.spin(angles)

	# Clean Shutdown
	rclpy.shutdown()

# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':

	main()
