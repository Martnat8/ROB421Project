#!/usr/bin/env python3

# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

 
from std_msgs.msg import String

import json 

class JointAnglesCorrected(Node):
        def __init__(self):
                super().__init__('robot_joint_publisher')

                self.sub = self.create_subscription(String, '/joint_angles', self.callback, 10)
                self.pub = self.create_publisher(String, '/joint_angles_corrected', 10)

                # "motor_id: [home min max] This is the order of the Angles
                self.joint_positions = {
                        "RightChest": [135, 60, 180], "RightShoulder": [85, 70, 240], "RightBicep": [115, 115, 180], "RightElbow": [90, 120, 155],
                        "LeftChest": [115, 30, 180], "LeftShoulder": [180, 30, 195], "LeftBicep": [115, 20, 180],"LeftElbow": [105, 60, 180] 
                        }
        def callback(self, msg: String):
                angles_in = json.loads(msg.data)
                angles_out = {}
                for joint, angle in angles_in.items():
                        home, minimum, maximum = self.joint_positions[joint]
                        corrected = home + angle
                        # clamp to [minimum, maximum]
                        if corrected < minimum:
                                corrected = minimum
                        elif corrected > maximum:
                                corrected = maximum
                        angles_out[joint] = corrected
                out_msg = String()
                out_msg.data = json.dumps(angles_out)
                self.pub.publish(out_msg)

# This is a entry point.	
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	corrected = JointAnglesCorrected()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(corrected)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()
    

        
