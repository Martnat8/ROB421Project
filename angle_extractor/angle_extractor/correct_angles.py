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

                # "motor_id: [servohome min max] This is the order of the Angles
                self.joint_positions = {
                        "RightChest": [135, 80, 150], "RightShoulder": [85, 70, 240], "RightBicep": [115, 115, 180], "RightElbow": [90, 20, 155],
                        "LeftChest": [115, 60, 140], "LeftShoulder": [180, 30, 195], "LeftBicep": [115, 20, 180],"LeftElbow": [105, 60, 180] 
                        }
                
                # self.joint_positions = {
                #         "LeftChest": [135, 60, 180], "LeftShoulder": [85, 70, 240], "LeftBicep": [115, 115, 180], "LeftElbow": [90, 20, 155],
                #         "RightChest": [115, 30, 180], "RightShoulder": [180, 30, 195], "RightBicep": [115, 20, 180],"RightElbow": [105, 60, 180] 
                #         }


                # This correlates with the CCW and CW values on the SAMI table
                self.dir_map = {
                        # Left side
                        "LeftChest": 1, "LeftShoulder": -1, "LeftBicep": 1, "LeftElbow": 1, "LeftGripper": 1, "LeftHip": 1, "LeftKnee": 1, "LeftAnkle": 1,
                        # Right side
                        "RightChest": 1, "RightShoulder": 1, "RightBicep": 1, "RightElbow": -1, "RightGripper": 1, "RightHip": 1, "RightKnee": 1, "RightAnkle": 1,
                        # Head
                        "HeadNod": 1, "HeadTurn": 1, "HeadTilt": 1,
                        # Torso
                        "TorsoBow": -1, "TorsoTilt": 1,
                        }

                # The home angles in real space frame
                self.home_angles = { 
                        "RightChest": 0, "RightShoulder": 0, "RightBicep":90, "RightElbow": 110,
                        "LeftChest": 0, "LeftShoulder": 0, "LeftBicep": -90,"LeftElbow": 90 
                        }

        def callback(self, msg: String):

                # Some defence against bad messages ( I let chatGPT convice me of this utility it's worth looking over again)
                # The thought is that if the message isn't in the correct format or there's an error these catch that and
                # prevent the node from crashing outright
                try:
                        angles_in = json.loads(msg.data)
                        if not isinstance(angles_in, dict):
                                raise ValueError("Expected a dict of joint angles")
                        
                except (json.JSONDecodeError, ValueError) as e:
                        self.get_logger().error(f"Bad joint_angles message: {e}")
                        return

                for joint, angle in angles_in.items():
                        if joint not in self.joint_positions:
                                self.get_logger().warn(f"Unknown joint: {joint}")
                                continue
                        if not isinstance(angle, (int, float)):
                                self.get_logger().warn(f"Non-numeric angle for {joint}: {angle}")
                                continue

                # Unpack message and correct angles for republish
                angles_out = {}

                for joint, angle in angles_in.items():
                        servohome, minimum, maximum = self.joint_positions[joint]


                        # Only grabs the multiplier if we have that joint
                        direction = self.dir_map.get(joint, 1)

                        # direction maps world → servo
                        delta = direction * (angle - self.home_angles[joint])
                        raw = servohome + delta


                        # clamp into [minimum, maximum]
                        corrected = max(minimum, min(raw, maximum))
                        angles_out[joint] = corrected
                        print(f"{joint}: input={angle}, servohome={servohome}, worldhome={self.home_angles[joint]}, delta={delta}, raw={raw}, corrected={angles_out[joint]}")

                        
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
    

        
