#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSubscriber:
    def __init__(self):

        # Initialize video_stream node
        rospy.init_node('video_stream_subscriber', anonymous=True)

        # Initialize Subscriber
        rospy.Subscriber("/zed_left_camera", Image, self.showStream, queue_size=10, buff_size=2**24)

        # Initialize CvBridge
        self.bridge = CvBridge()

    def showStream(self, ros_image):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgra8")

        except CvBridgeError as e:
            rospy.logerror("CvBridge error: {e}")

        else:
            cv2.imshow("image", cv_image)

            command = chr(cv2.waitKey(1) & 255)

            if command == 'q':
                cv2.destroyAllWindows()
                rospy.loginfo("Closing video window and shutting down node")
                rospy.signal_shutdown("Shutting down")
                
        finally:
            pass

    def start(self):
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
if __name__ == '__main__':
    image_subscriber = ImageSubscriber()
    image_subscriber.start()