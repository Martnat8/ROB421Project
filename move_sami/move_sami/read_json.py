#!/usr/bin/env python3

import time
import serial
import json
from move_sami.audio_manager import AudioManager

class JamieControl:
    def __init__(self, 
                 arduino_port='/dev/ttyUSB0', 
                 baud_rate=115200,
                 joint_config_file='Joint_config.json',
                 emote_file='Emote.json',
                 audio_folder='audio',
                 starting_voice='Matt',
                 audio_file_encoding='.mp3'):
        self.arduino_port = arduino_port
        self.baud_rate = baud_rate
        self.ser = None
        self.joint_map = self.load_joint_config(joint_config_file)
        self.emote_mapping = self.load_emote_mapping(emote_file)
        # Initialize the audio manager so that audio clips can be played.
        self.audio_manager = AudioManager(starting_voice, audio_folder, audio_file_encoding)

    def load_joint_config(self, joint_config_file):
        with open(joint_config_file, 'r') as file:
            config = json.load(file)
        joint_map = {}
        for joint in config["JointConfig"]:
            joint_map[joint["JointName"]] = joint["JointID"]
        return joint_map

    def load_emote_mapping(self, emote_file):
        with open(emote_file, 'r') as file:
            data = json.load(file)
        return data["Emotes"]

    def initialize_serial_connection(self):
        try:
            self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=1)
            time.sleep(2)
            print("Serial connection established.")
            packet = [0x3C, 0x50, 0x01, 0x45, 0x3E]
            self.ser.write(bytearray(packet))
            print("Sent packet:", bytearray(packet))
            if self.ser.in_waiting > 0:
                msg = self.ser.readline().decode()
                print("Arduino response:", msg)
        except serial.SerialException as e:
            print("Error connecting to Arduino:", e)

    def send_joint_command(self, joint_ids, joint_angles, joint_time):
        if len(joint_ids) != len(joint_angles):
            raise ValueError("Mismatch in joint IDs and angles.")
        packet = [0x3C, 0x4A, joint_time]
        for jid, angle in zip(joint_ids, joint_angles):
            packet.extend([jid, angle])
        packet.append(0x3E)
        self.ser.write(bytearray(packet))
        print("Sent joint command:", bytearray(packet))

    def send_emote(self, emote_id):
        packet = [0x3C, 0x45, emote_id, 0x3E]
        self.ser.write(bytearray(packet))
        print("Sent emote command:", bytearray(packet))
        time.sleep(1)

    def close_connection(self):
        if self.ser:
            self.ser.close()
            print("Serial connection closed.")

    def read_json(self, filename):
        with open(filename, 'r') as file:
            data = json.load(file)
        for keyframe in data["Keyframes"]:
            # Process Audio if enabled.
            if keyframe.get("HasAudio") == "True":
                # Use "AudioClip" if provided; otherwise fall back to the "Expression" or a default.
                audio_clip = keyframe.get("AudioClip", keyframe.get("Expression", "default_audio"))
                print("Processing audio keyframe – playing:", audio_clip)
                self.audio_manager.send_audio(audio_clip)
            # Process Emote if enabled.
            if keyframe.get("HasEmote") == "True":
                expression = keyframe.get("Expression", "Neutral")
                emote_value = self.emote_mapping.get(expression, 0)
                self.send_emote(emote_value)
            # Process Joint Commands if enabled.
            if keyframe.get("HasJoints") == "True":
                joint_ids = []
                joint_angles = []
                joint_time = keyframe.get("JointMoveTime", 1)
                for joint in keyframe["JointAngles"]:
                    joint_ids.append(self.get_joint_id(joint["Joint"]))
                    joint_angles.append(joint["Angle"])
                self.send_joint_command(joint_ids, joint_angles, joint_time)
            time.sleep(keyframe.get("WaitTime", 1000) / 1000)

    def get_joint_id(self, joint_name):
        return self.joint_map.get(joint_name, 0)

if __name__ == "__main__":
    # For testing purposes; adjust paths as necessary.
    robot = JamieControl(audio_folder="audio", starting_voice="Matt")
    robot.initialize_serial_connection()
    time.sleep(1)
    # Ensure your Wave.json has at least one keyframe with HasAudio set to "True"
    robot.read_json("Wave.json")
    time.sleep(1)
    robot.close_connection()
