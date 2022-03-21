#!/usr/bin/env python3

import rospy
import os
import cv2
import pyzed.sl as sl
import numpy as np

from sensor_msgs.msg import Joy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray
from rospy_tutorials.msg import Floats

class ZedCamera(object):
    """docstring for ZedCamera"""

    def __init__(self):
        super(ZedCamera, self).__init__()

        # Initialize button mapping
        self.capture_depth_map_button = 6 # LT
        self.capture_depth_video_button = 7 # RT
        self.stream_video_button = 8 # Back
        self.toggle_camera_button = 9 # Start

        # Initialize zed object
        self.zed = None
        self.init_params = None

        # Initialize variables
        self.capture_depth_map = False
        self.continuous_capture = False
        self.stream_video = False

        # Initialize save directory
        self.save_directory = os.path.join(os.getcwd(), "data")

        if not os.path.isdir(self.save_directory):
            rospy.loginfo("Creating save Directory")
            os.makedirs(self.save_directory)

        # Initialize zed_ros node
        rospy.init_node('zed_ros', anonymous=False)
        
        # Initialize joy message subscribers
        rospy.Subscriber('/joy_teleop/joy', Joy, self.handle_joy_message, queue_size=3, buff_size=2**16)

        # Initialize stream publisher
        self.image_publisher = rospy.Publisher('/zed_left_camera', Float64MultiArray, queue_size=10)

        # Initialize params
        self.initialize_parameters()

        # Define sleep rate
        self.rate = rospy.Rate(10)

    def handle_joy_message(self, joy_msg):
        # Massive if statement to handle joy mesages
        if joy_msg.buttons[self.capture_depth_map_button]:
            self.capture_depth_map ^= True
            rospy.sleep(1.)
        elif joy_msg.buttons[self.capture_depth_video_button]:
            if self.continuous_capture:
                self.continuous_capture ^= True
                rospy.sleep(1.)
            else:
                for i in range(3, -1, -1):
                    rospy.loginfo(f"Toggling continuous_capture in: {i}")
                    rospy.sleep(1.)
                self.continuous_capture ^= True
                
            rospy.loginfo(f"Continuous capture: {self.continuous_capture}")
        elif joy_msg.buttons[self.stream_video_button]:
            self.stream_video ^= True
            rospy.loginfo(f"Video streaming: {self.stream_video}")
            rospy.sleep(1.)
        elif joy_msg.buttons[self.toggle_camera_button]:
            self.toggle_camera()
    
    def grab_frame(self, image_mat, depth_mat, point_cloud_mat, mirror_mat):
        # Grab frame
            err = self.zed.grab(self.runtime_parameters)

            if err == sl.ERROR_CODE.SUCCESS:
                rospy.loginfo("Succesfully grabbed frame!")
                
                # Take image and matching depth map
                rospy.loginfo("Got image!")
                self.zed.retrieve_image(image_mat, sl.VIEW.LEFT)
                
                # Retrieve depth map. Depth is aligned on the left image
                rospy.loginfo("Got depth map!")
                self.zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
                
                # Retrieve Point Cloud
                rospy.loginfo("Got point cloud!")
                self.zed.retrieve_measure(point_cloud_mat, sl.MEASURE.XYZRGBA)

                # Get time stamp for file name
                timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
                
                # Get Depth data and save to file
                image_data = image_mat.get_data()
                depth_data = depth_mat.get_data()
                point_cloud_data = point_cloud_mat.get_data()
                point_cloud_data.dot(mirror_mat)

                if self.stream_video:
                    self.publish_image_message(image_data)

                # Save data
                rospy.loginfo("Saving data...")
                self.save_data(image_data, depth_data, point_cloud_data, timestamp)

            else:
                rospy.loginfo(f"Error grabbing frame: {err}")    
                self.toggle_camera()

    def publish_image_message(self, image_data):

        # Publish image
        # image = image_data.flatten().astype(np.uint8)

        img_msg = Float64MultiArray()
        image_data = tuple(map(tuple, image_data))
        img_msg.data = image_data

        self.image_publisher.publish(img_msg)

    def publish_video(self):
        # Capture and save image
        image = sl.Mat()
        
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:   
            # Get image and time stamp
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            
            # Get Image data
            image_data = image.get_data()
            
            # Publish image data
            self.publish_image_message(image_data)

    def save_data(self, image_data, depth_data, point_cloud_data, timestamp):
        # Create folder for depth map/image pair
        directory = os.path.join(self.save_directory, f"{timestamp.get_milliseconds()}")
        os.makedirs(directory)
        
        # Save Image
        file_name = f"zed_image_left_{timestamp.get_milliseconds()}.jpg"
        cv2.imwrite(os.path.join(directory, file_name), image_data)
        
        # Save depth map
        file_name = f"depth_map_{timestamp.get_milliseconds()}.npy"
        np.save(os.path.join(directory, file_name), depth_data)

        # Save depth map
        file_name = f"point_cloud_{timestamp.get_milliseconds()}.npy"
        np.save(os.path.join(directory, file_name), point_cloud_data)
        
        rospy.loginfo("Data saved!")
        
    def toggle_camera(self):
        
        # Initialize zed object if it doesn't exist
        # Otherwise, close the camera
        if not self.zed:
            rospy.loginfo("Initializing camera!")
            
            # Create a Camera object
            zed = sl.Camera()
            err = zed.open(self.init_params)

            if err != sl.ERROR_CODE.SUCCESS:
                rospy.loginfo(f"Zed cam failed to initialize: {err}")
            else:
                self.zed = zed            
                rospy.loginfo("Camera initialized!")
            
        else:
            rospy.loginfo("Closing camera!")
            
            self.zed.close()
            self.zed = None

            rospy.loginfo("Camera closed!")

    def initialize_parameters(self):

        rospy.loginfo("Setting InitParams and RuntimeParams...")

        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.QUALITY  # Use PERFORMANCE depth mode
        self.init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        self.init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
        self.init_params.camera_fps = 30  # Set fps at 30

        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
        
        # Setting the depth confidence parameters
        self.runtime_parameters.confidence_threshold = 100
        self.runtime_parameters.textureness_confidence_threshold = 100

        rospy.loginfo("InitParams and RuntimeParams set!")

    def toggle_continuous_capture(self):
        self.continuous_capture ^= True
        rospy.loginfo(f"Continuous capture: {self.continuous_capture}")
        rospy.sleep(1.)

    def start(self):
        # Keeps python from exiting until node is stopped
        rospy.loginfo("Starting Zed Camera Node...")

        # Initialize depth map, image, and point cloud objects
        rospy.loginfo("Initializing depth/image objects")
        image_mat = sl.Mat()
        depth_mat = sl.Mat()
        point_cloud_mat = sl.Mat()

        # Prepare for distance calculation
        mirror_ref = sl.Transform()
        mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
        tr_np = mirror_ref.m

        while not rospy.is_shutdown():

            if self.capture_depth_map:
                if not self.zed:
                    rospy.loginfo("Capture depth map error: Initialize Camera First!")
                else:
                    rospy.loginfo("Capturing depth map/image pair...")
                    self.grab_frame(image_mat, depth_mat, point_cloud_mat, tr_np)
                    self.capture_depth_map = False

            elif self.continuous_capture:
                rospy.loginfo("Capturing depth map/image pair...")
                self.grab_frame(image_mat, depth_mat, point_cloud_mat, tr_np)

            if self.stream_video and not self.capture_depth_map and not self.continuous_capture:
                if not self.zed:
                    rospy.loginfo("Video stream error: Initialize Camera First!")
                else:
                    self.publish_video()
                

            self.rate.sleep()

if __name__ == '__main__':
    zed = ZedCamera()
    zed.start()