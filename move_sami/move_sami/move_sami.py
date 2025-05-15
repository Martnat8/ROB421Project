#!/usr/bin/env python3

# Motion control for SAMI while listening to the /joint_angles_corrected for angle commands
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json 

from ament_index_python.packages import get_package_share_directory
from move_sami.read_json import JamieControl

class MoveSami(Node):
        def __init__(self):
                super().__init__('move_sami')

                # find where move_sami was installed
                pkg_share = os.path.join('/home/Teft/ROB421Project_ws/src/move_sami')
                cfg_dir   = os.path.join(pkg_share, 'config')

                # point JamieControl at the installed JSONs
                joint_cfg = os.path.join(cfg_dir, 'Joint_config.json')
                emote_cfg = os.path.join(cfg_dir, 'Emote.json')
                self.robot = JamieControl(
                joint_config_file=joint_cfg,
                emote_file=emote_cfg,)

                # Make sure to adjust the port in 'read_json.py' !!
                self.robot.initialize_serial_connection()

                self.sub = self.create_subscription(String, '/joint_angles_corrected', self.callback, 10)

                # When testing we can create a new joint ID map so that we only publish to the desired joints
                # self.joint_id_map = {"RightChest": 4, "RightShoulder": 5, "RightBicep": 6, "RightElbow": 7,
                #         "LeftChest": 8,  "LeftShoulder": 9, "LeftBicep": 10,  "LeftElbow": 11}

                self.joint_id_map = {
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
        
        def destroy_node(self):
                # first clean up your Arduino link
                self.robot.close_connection()
                # then do ROS2’s normal teardown
                super().destroy_node()


# This is a entry point.	
def main(args=None):
    rclpy.init(args=args)
    motion = MoveSami()

    try:

        rclpy.spin(motion)
    finally:

        motion.destroy_node()
        rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	main()
    

        
