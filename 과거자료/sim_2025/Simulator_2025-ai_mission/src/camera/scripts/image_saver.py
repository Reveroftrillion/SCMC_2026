#!/usr/bin/env python3
import rospy
import cv2
import os
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import message_filters

# --- Configuration ---
SAVE_DIR = "/home/ctrl/Simulator_2025/sign" 
SAVE_INTERVAL = 0.3  # seconds
# --- End Configuration ---

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver_node', anonymous=True)
        self.bridge = CvBridge()

        if not os.path.isdir(SAVE_DIR):
            rospy.logerr(f"Save directory '{SAVE_DIR}' does not exist. Shutting down.")
            rospy.signal_shutdown("Save directory not found")
            return

        # Subscribers
        left_sub = message_filters.Subscriber('/image_jpeg/compressed', CompressedImage)
        right_sub = message_filters.Subscriber('/image_jpeg2/compressed', CompressedImage)

        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback_sync)

        rospy.loginfo("Image saver node started. Subscribed to topics, saving to: " + SAVE_DIR)
        self.save_timer = rospy.Timer(rospy.Duration(SAVE_INTERVAL), self.timer_callback)
        self.latest_left = None
        self.latest_right = None


    def callback_sync(self, left_msg, right_msg):
        self.latest_left = left_msg
        self.latest_right = right_msg

    def timer_callback(self, event):
        if self.latest_left is None or self.latest_right is None:
            return
        
        try:
            # Process and save left image
            left_image = self.bridge.compressed_imgmsg_to_cv2(self.latest_left, "bgr8")
            self.save_image(left_image, "left", self.latest_left.header.stamp)

            # Process and save right image
            right_image = self.bridge.compressed_imgmsg_to_cv2(self.latest_right, "bgr8")
            self.save_image(right_image, "right", self.latest_right.header.stamp)
            
            # Reset after saving to ensure we only save new pairs
            self.latest_left = None
            self.latest_right = None

        except Exception as e:
            rospy.logerr(f"Failed to process and save images: {e}")

    def save_image(self, image, prefix, stamp):
        # Use ROS time for timestamping to be more consistent with the data
        timestamp = stamp.to_sec()
        filename = os.path.join(SAVE_DIR, f"{prefix}_{timestamp:.6f}.jpg")
        try:
            cv2.imwrite(filename, image)
            rospy.loginfo(f"Saved {filename}")
        except Exception as e:
            rospy.logerr(f"Failed to save image {filename}: {e}")

if __name__ == '__main__':
    try:
        ImageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Image saver node shut down.")
