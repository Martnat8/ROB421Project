#!/usr/bin/env python3

# Motion control for SAMI while listening to the /joint_angles_corrected for angle commands
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json 

from move_sami.read_json import JamieControl

class MoveSami(Node):
        def __init__(self):
                super().__init__('move_sami')


                self.robot = JamieControl()
                self.robot.initialize_serial_connection()

                self.sub = self.create_subscription(String, '/joint_angles_corrected', self.callback, 10)

                # When testing we can create a new joint ID map so that we only publish to the desired joints
                self.joint_id_map = {"RightChest": 4, "RightShoulder": 5, "RightBicep": 6, "RightElbow": 7,
                        "LeftChest": 8,  "LeftShoulder": 9, "LeftBicep": 10,  "LeftElbow": 11}

                
        def callback(self, msg: String):

                angles_in = json.loads(msg.data)

                # Map names → IDs, casting angles to int
                id_angles = {
                        self.joint_id_map[name]: int(angle)
                        for name, angle in angles_in.items()
                        if name in self.joint_id_map}
                
                joint_ids    = list(id_angles.keys())       # [4, 5, 6, …]
                joint_angles = list(id_angles.values())     # [37.2, 82.5, …]
                joint_time   = 1      

                if not id_angles:
                        self.get_logger().warn("No known joints in incoming message – skipping")
                        return

                self.robot.send_joint_command(joint_ids, joint_angles, joint_time)
                        


# This is a entry point.	
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	motion = MoveSami()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(motion)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()
    

        
