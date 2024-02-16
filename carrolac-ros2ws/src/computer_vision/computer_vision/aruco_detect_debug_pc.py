#!/usr/bin/env python3

import cv2
from matplotlib import pyplot as plt
import numpy as np

markerSize = 0.25
markerPoints = 0.5*markerSize*np.array(
    (((-1, 1, 0),
      (1, 1, 0),
      (1, -1, 0),
      (-1, -1, 0)),)
)


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
        img = cv2.imread("../aruco_markers/gz.png")

        self.myaxis.imshow(img)

        (markerCorners, markerIds, rejected) = \
            self.aruco_detector.detectMarkers(img)

        cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds)

        p = np.array([[277.,   0., 160.,   0.],   [0., 277., 120.,   0.],   [0.,   0.,   1.,   0.]])
        k = np.eye(3)
        d = np.array([[0.0, 0.0, 0.0, 0.0, 0.0],])

        if markerIds is not None:
            nmarkers = len(markerCorners)
            # self.get_logger().info(str(nmarkers))
            for i in range(nmarkers):
                (retval, rvec, tvec) = cv2.solvePnP(objectPoints=markerPoints,
                                                    imagePoints=markerCorners[i][0],
                                                    cameraMatrix=k, distCoeffs=d)
                cv2.drawFrameAxes(img, k, d, rvec, tvec, 0.1)

        plotimage = self.myaxis.imshow(img)
        input()

if __name__=="__main__":
    SimpleUse()