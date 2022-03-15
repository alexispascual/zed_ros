#!/usr/bin/env python3

import rospy
import os
import cv2
import pyzed.sl as sl

from sensor_msgs.msg import Joy

class ZedCamera(object):
    """docstring for ZedCamera"""

    def __init__(self):
        super(ZedCamera, self).__init__()

        # Initialize button mapping
        self.capture_image_button = 5
        self.initialize_camera_button = 9

        # Initialize zed object
        self.zed = None

        # Initialize save directory
        self.save_directory = os.path.join(os.getcwd(), "images")

        if not os.path.isdir(self.save_directory):
            rospy.loginfo("Creating save Directory")
            os.makedirs(self.save_directory)

        # Initialize zed_ros node
        rospy.init_node('zed_ros', anonymous=True)
        
        # Initialize joy message subscribers
        rospy.Subscriber('/joy', Joy, self.handle_joy_message, queue_size=3, buff_size=2**16)

    def handle_joy_message(self, joy_msg):

        if joy_msg.buttons[self.capture_image_button]:
            # Capture and save image
            image = sl.Mat()
            runtime_parameters = sl.RuntimeParameters()

            if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # Get image and time stamp
                self.zed.retrieve_image(image, sl.VIEW.LEFT)
                timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
                image_cv = image.get_data()
                file_name = f"zed_image_left_{timestamp}.jpg"

                # Save image
                cv2.imwrite(os.path.join(self.save_directory, file_name), image_cv)

        elif joy_msg.buttons[self.initialize_camera_button]:
            # Toggle camera
            if not self.zed:
                self.zed = self.initialize_camera()
                rospy.loginfo("Camera initialized!")
            else:
                self.zed.close()
                rospy.loginfo("Camera closed!")


    def initialize_camera(self):

        # Create a Camera object
        zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
        init_params.camera_fps = 30  # Set fps at 30

        # Open the camera
        if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            rospy.loginfo("Zed cam failed to initialize")
        else:
            return zed;


    def start(self):
        # Keeps python from exiting until node is stopped
        rospy.loginfo("Starting Zed Camera Node...")
        rospy.spin()

if __name__ == '__main__':
    zed = ZedCamera()
    zed.start()