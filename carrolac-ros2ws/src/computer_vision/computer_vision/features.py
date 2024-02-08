#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

from matplotlib import pyplot as plt


class Features_Node(Node):
    def __init__(self):
        super().__init__('features_match')
        self.get_logger().info("Inicializando nodo...")

        self.create_subscription(
            Image, "/stereo_camera/left/image_raw", self.image_callback, 10
        )

        self.cvbridge = CvBridge()
        self.qcd = cv2.QRCodeDetector()

        plt.ion()
        self.myfigure, self.myaxis = plt.subplots()

    def image_callback(self, msg):
        self.get_logger().info("Recibiendo mensaje...")
        image = self.cvbridge.imgmsg_to_cv2(msg)

        retval, decoded_info, points, straight_qrcode = self.qcd.detectAndDecodeMulti(image)
        self.get_logger().info(str(retval) + ": " + str(decoded_info))

        if retval:
            image = cv2.polylines(image, points.astype(int), True, (0, 255, 0), 3)
            for s, p in zip(decoded_info, points):
                image = cv2.putText(image, s, p[0].astype(int),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)


        # PLOT WITH PYPLOT ###
        if hasattr(self, 'plotimage'):
            self.plotimage.set_data(image)
        else:
            self.plotimage = self.myaxis.imshow(image)
        self.myfigure.canvas.draw()
        self.myfigure.canvas.flush_events()
        #########################


def main(args=None):
    rclpy.init(args=args)

    rclpy.spin(Features_Node())


if __name__ == "__main__":
    main()
