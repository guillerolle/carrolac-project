#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

from matplotlib import pyplot as plt


class Features_Node(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.get_logger().info("Inicializando nodo...")

        self.create_subscription(
            Image, "/stereo_camera/left/image_raw", self.image_callback, 10
        )

        self.cvbridge = CvBridge()
        # self.qcd = cv2.QRCodeDetector()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(
            dictionary=self.aruco_dict,
            detectorParams=self.aruco_params
        )
        # self.get_logger().info(str(dir(self.aruco_detector.getDetectorParameters())))

        plt.ion()
        self.myfigure, self.myaxis = plt.subplots()

    def image_callback(self, msg):
        # self.get_logger().info("Recibiendo mensaje...")
        img = self.cvbridge.imgmsg_to_cv2(msg)

        (markerCorners, markerIds, rejected) = \
            self.aruco_detector.detectMarkers(img)

        cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)

        cv2.imwrite("../aruco_markers/textures/gz.png", img)

        # PLOT WITH PYPLOT ###
        if hasattr(self, 'plotimage'):
            self.plotimage.set_data(img)
        else:
            self.plotimage = self.myaxis.imshow(img)
        self.myfigure.canvas.draw()
        self.myfigure.canvas.flush_events()
        #########################
        # input()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Features_Node())


if __name__ == "__main__":
    main()
