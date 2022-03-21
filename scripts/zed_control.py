#!/usr/bin/env python3

import rospy
import os
import cv2
import pyzed.sl as sl
import numpy as np

from sensor_msgs.msg import Joy

class ZedCamera(object):
    """docstring for ZedCamera"""

    def __init__(self):
        super(ZedCamera, self).__init__()

        # Initialize button mapping
        self.capture_depth_map_button = 6
        self.capture_depth_video_button = 7
        self.stream_video_button = 8
        self.toggle_camera_button = 9

        # Initialize zed object
        self.zed = None
        self.init_params = None
        
        # Initialize save directory
        self.save_directory = os.path.join(os.getcwd(), "data")

        if not os.path.isdir(self.save_directory):
            rospy.loginfo("Creating save Directory")
            os.makedirs(self.save_directory)

        # Initialize zed_ros node
        rospy.init_node('zed_ros', anonymous=True)
        
        # Initialize joy message subscribers
        rospy.Subscriber('/joy_teleop/joy', Joy, self.handle_joy_message, queue_size=3, buff_size=2**16)

        # Initialize params
        self.initialize_parameters()
        
    def handle_joy_message(self, joy_msg):
        # Massive if statement to handle joy mesages
        if joy_msg.buttons[self.capture_depth_map_button]:
            self.capture_depth_map()

        elif joy_msg.buttons[self.toggle_camera_button]:
            self.toggle_camera()
            
    def capture_depth_map(self):
        
        if not self.zed:
            rospy.loginfo("Initialize Camera First!")
        else:
            rospy.loginfo("Capturing depth map/image pair...")
            
            # Initialize depth map, image, and point cloud objects
            rospy.loginfo("Initializing depth/image objects")
            depth = sl.Mat()
            image = sl.Mat()
            point_cloud = sl.Mat()

            # Prepare for distance calculation
            mirror_ref = sl.Transform()
            mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
            tr_np = mirror_ref.m
            
            if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                rospy.loginfo("Grabbed runtime params...")
                
                # Take image and matching depth map
                rospy.loginfo("Got image!")
                self.zed.retrieve_image(image, sl.VIEW.LEFT)
                
                # Retrieve depth map. Depth is aligned on the left image
                rospy.loginfo("Got depth map!")
                self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                
                # Retrieve Point Cloud
                rospy.loginfo("Got point cloud!")
                self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

                # Get time stamp for file name
                timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
                
                # Get Depth data and save to file
                depth_data = depth.get_data()
                image_data = image.get_data()
                point_cloud_data = point_cloud.get_data()
                point_cloud_data.dot(tr_np)

                # Save data
                rospy.loginfo("Saving data...")
                self.save_data(self, image_data, depth_data, point_cloud_data, timestamp)
                
            self.toggle_camera()
            
    def stream_video(self):
        # Capture and save image
        image = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        
        if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:   
            # Get image and time stamp
            self.zed.retrieve_image(image, sl.VIEW.LEFT)
            timestamp = self.zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)
            
            # Get Image data and save to file
            image_data = image.get_data()
            
            # TODO: Publish image data

            # Save image
            cv2.imwrite(os.path.join(self.save_directory, file_name), image_data)

    def save_data(self, image_data, depth_data, point_cloud_data, timestamp):
        # Create folder for depth map/image pair
        directory = os.path.join(self.save_directory, timestamp)
        os.path.makedirs(directory)
        
        # Save Image
        file_name = f"zed_image_left_{timestamp}.jpg"
        cv2.imwrite(os.path.join(directory, file_name), image_data)
        
        # Save depth map
        file_name = f"depth_map_{timestamp}.npy"
        np.save(file_name, depth_data)

        # Save depth map
        file_name = f"point_cloud_{timestamp}.npy"
        np.save(file_name, point_cloud_data)
        
        rospy.loginfo("Data saved!")
        
    def toggle_camera(self):
        
        # Initialize zed object if it doesn't exist
        # Otherwise, close the camera
        if not self.zed and not self.init_params:
            rospy.loginfo("Initializing camera!")
            
            # Create a Camera object
            zed = sl.Camera()
            err = zed.open(self.init_params)

            if err != sl.ERROR_CODE.SUCCESS:
                rospy.loginfo(f"Zed cam failed to initialize: {err}")
            else:
                self.zed = zed
                self.init_params = init_params
            
                rospy.loginfo("Camera initialized!")
            
        else:
            rospy.loginfo("Closing camera!")
            
            self.zed.close()
            self.zed = None
            self.init_params = None
            
            rospy.loginfo("Camera closed!")

    def initialize_parameters(self):

        rospy.loginfo("Setting InitParams and RuntimeParams...")

        self.init_params = sl.InitParameters()
        self.init_params.depth_mode = sl.DEPTH_MODE.QUALITY  # Use PERFORMANCE depth mode
        self.init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
        self.init_params.camera_resolution = sl.RESOLUTION.HD2K  # Use HD1080 video mode
        self.init_params.camera_fps = 15  # Set fps at 30

        # Create and set RuntimeParameters after opening the camera
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
        
        # Setting the depth confidence parameters
        self.runtime_parameters.confidence_threshold = 100
        self.runtime_parameters.textureness_confidence_threshold = 100

        rospy.loginfo("InitParams and RuntimeParams set!")

    def start(self):
        # Keeps python from exiting until node is stopped
        rospy.loginfo("Starting Zed Camera Node...")
        rospy.spin()

if __name__ == '__main__':
    zed = ZedCamera()
    zed.start()