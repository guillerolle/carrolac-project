#!/usr/bin/env python3

import cv2
from matplotlib import pyplot as plt


class SimpleUse():
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(
            dictionary=self.aruco_dict,
            detectorParams=self.aruco_params
        )
        # self.get_logger().info(str(dir(self.aruco_detector.getDetectorParameters())))

        plt.ion()
        self.myfigure, self.myaxis = plt.subplots()

        self.detect_simple_image()

    def detect_simple_image(self):
        img = cv2.imread("../aruco_markers/textures/gz.png")

        self.myaxis.imshow(img)

        (markerCorners, markerIds, rejected) = \
            self.aruco_detector.detectMarkers(img)

        print("markerCorners: " + str(markerCorners))
        print("markerIds: " + str(markerIds))
        print("Rejected: " + str(rejected))
        img = cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)

        plotimage = self.myaxis.imshow(img)
        input()

if __name__=="__main__":
    SimpleUse()