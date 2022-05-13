import cv2
import numpy as np
import rospy as rp
from cv_bridge import CvBridge
from numba import njit
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2, PointField

import message_filters

class GapDetectingNode:
    def __init__(self):
        # publishers
        self.cloud_publisher = rp.Publisher("/gap_detector/points", PointCloud2, queue_size=1)

        # subscribers
        camera_subscriber = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_subscriber = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)

        ts = message_filters.ApproximateTimeSynchronizer([camera_subscriber, depth_subscriber], 2, 0.1)
        ts.registerCallback(self.joint_callback)

        self.bridge = CvBridge()

        self.last_camera_mask = None
        self.picker = 0
        self.process_every = 2  # 2 means every 2nd depth frame is processed, 3 means every 3rd etc.

        # Tuned for green pieces of paper:
        self.low_hue = 50  # 50
        self.high_hue = 100  # 100
        self.low_sat = 80  # 80
        self.high_sat = 180  # 180
        self.low_val = 100  # 100
        self.high_val = 255  # 255
        print('initialized')

        # UNCOMMENT FOR THRESHOLD DEBUGGING:
        # self.windowName = 'WINDOW'
        # cv2.namedWindow(self.windowName)
        # cv2.createTrackbar('low_hue', self.windowName, 0, 255, self.low_hue_update)
        # cv2.createTrackbar('high_hue', self.windowName, 0, 255, self.high_hue_update)
        # cv2.createTrackbar('low_sat', self.windowName, 0, 255, self.low_sat_update)
        # cv2.createTrackbar('high_sat', self.windowName, 0, 255, self.high_sat_update)
        # cv2.createTrackbar('low_val', self.windowName, 0, 255, self.low_val_update)
        # cv2.createTrackbar('high_val', self.windowName, 0, 255, self.high_val_update)
        # self.image = None
        # rate = rp.Rate(1)  # 1hz
        # while not rp.is_shutdown():
        #     self.check_hsv()
        #     rate.sleep()

    def joint_callback(self, rgb_message, depth_message):
        image = self.bridge.imgmsg_to_cv2(rgb_message)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        low = (self.low_hue, self.low_sat, self.low_val)
        high = (self.high_hue, self.high_sat, self.high_val)
        self.last_camera_mask = cv2.inRange(hsv, low, high)

        # UNCOMMENT FOR THRESHOLD DEBUGGING:
        # self.image = image

        if self.picker % self.process_every == 0:
            if self.last_camera_mask is not None:
                depth_image = self.bridge.imgmsg_to_cv2(depth_message, "16UC1")

                points = project(depth_image, self.last_camera_mask)

                fields = [PointField('x', 0, PointField.FLOAT32, 1),
                        PointField('y', 4, PointField.FLOAT32, 1),
                        PointField('z', 8, PointField.FLOAT32, 1),
                        ]

                header = rgb_message.header
                pc2_message = point_cloud2.create_cloud(header, fields, points)
                self.cloud_publisher.publish(pc2_message)
        self.picker += 1

    def low_hue_update(self, value):
        self.low_hue = value

    def high_hue_update(self, value):
        self.high_hue = value

    def low_sat_update(self, value):
        self.low_sat = value

    def high_sat_update(self, value):
        self.high_sat = value

    def low_val_update(self, value):
        self.low_val = value

    def high_val_update(self, value):
        self.high_val = value

    def check_hsv(self):
        if self.image is not None:
            key = None
            while key != ord('q'):
                image_copy = self.image.copy()
                hsv = cv2.cvtColor(image_copy, cv2.COLOR_BGR2HSV)
                low = (self.low_hue, self.low_sat, self.low_val)
                high = (self.high_hue, self.high_sat, self.high_val)
                mask = cv2.inRange(hsv, low, high)
                image_copy = cv2.bitwise_and(image_copy, image_copy, mask=mask)
                cv2.imshow(self.windowName, image_copy)
                key = cv2.waitKey(1)


@njit
def project(depth_image, last_camera_mask):
    points = []
    for u, v in np.argwhere(last_camera_mask):
        d = depth_image[u, v] / 1000
        x = (v - 653.6612548828125) * d / 909.1968383789062
        y = (u - 367.6266174316406) * d / 909.0488891601562
        z = d
        points.append((x, y, z))
    return points


if __name__ == '__main__':
    rp.init_node('gap_detector')

    detector = GapDetectingNode()

    while not rp.is_shutdown():
        rp.spin()
