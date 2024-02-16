#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseArray

from cv_bridge import CvBridge
import cv2

import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

markerSize = 0.25
markerVertices = 0.5 * markerSize * np.array(
    (
     (0, -1, 1),
     (0, 1, 1),
     (0, 1, -1),
     (0, -1, -1),
     )
)

markersCenter = (
    (3.25, 0.0, 0.75, 0.0, 0.0, 3.14),
)

markerPoints = []
for m in markersCenter:
    tf = np.eye(4)
    # print(m)
    r = R.from_euler(angles=m[3:6], seq='xyz')
    tf[0:3, 0:3] = r.as_matrix()
    tf[0:3, 3] = m[0:3]
    # print(tf)
    # print(tf.shape)
    vs = []
    for v in markerVertices:
        o = np.ones(4)
        o[0:3] = v
        # print(o.shape)
        # print(tf @ o)
        vs.append(tf @ o)
    markerPoints.append(np.array(vs))
# print(markerPoints)
# print(markerPoints[0])


class Features_Node(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.get_logger().info("Inicializando nodo...")

        self.create_subscription(
            Image, "/stereo_camera/left/image_raw", self.image_callback, 10
        )

        self.aruco_pub = self.create_publisher(Image, "computer_vision/aruco/image_raw", 10)

        self.create_subscription(
            CameraInfo, "/stereo_camera/left/camera_info", self.camerainfo_callback, 10
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

        self.cameraInfo = None

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_static_broadcaster()

    def marker_static_broadcaster(self):
        transforms = []
        for i in range(2):
            self.get_logger().info("Creating Static Transform for AU{:0>4}".format(i))
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = "AU{:0>4}".format(i)
            t.transform.translation.x = 3.25
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.75
            transforms.append(t)

        self.tf_static_broadcaster.sendTransform(transforms)

    def camerainfo_callback(self, msg):
        self.cameraInfo = msg

    def image_callback(self, msg):
        # self.get_logger().info("Recibiendo mensaje...")
        if self.cameraInfo is None:
            print("Received image message but camera info is none.. Skipping message")
            return

        # self.get_logger().info(str(self.cameraInfo.r))
        img = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # cv2.imshow()
        # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        (markerCorners, markerIds, rejected) = \
            self.aruco_detector.detectMarkers(img)

        cv2.aruco.drawDetectedMarkers(img, markerCorners, markerIds, borderColor=(0, 0, 255))

        if markerIds is not None:
            nmarkers = len(markerCorners)
            k = np.array(self.cameraInfo.k).reshape([3, 3])
            d = np.array(self.cameraInfo.d)

            # self.get_logger().info(str(k))
            # self.get_logger().info(str(d))

            self.get_logger().debug("Markers ID: " + str(markerIds))

            for i in range(nmarkers):
                mp = np.array(markerPoints[i][0:4, 0:3])
                # print("Marker Vertices: " + str(markerVertices))
                # print("Marker Points: " + str(mp))
                # print(mp)
                mc = markerCorners[i][0]
                # print("Marker Corners: " + str(markerCorners[i]))
                (retval, rvec, tvec) = cv2.solvePnP(objectPoints=markerVertices,
                                                    imagePoints=mc,
                                                    cameraMatrix=k, distCoeffs=d)
                cv2.drawFrameAxes(img, k, d, rvec, tvec, markerSize)
                self.get_logger().debug("RetVal: " + str(retval))
                self.get_logger().debug("Rotation Vector: " + str(np.transpose(rvec)))
                rv = R.from_rotvec(rvec.T)
                rq = rv.as_quat()
                self.get_logger().debug("Rotation Euler: " + str(rq[0]))
                self.get_logger().debug("Translation Vector: " + str(np.transpose(tvec)))
                # self.get_logger().debug("Translation Vector Original: " + str(tvec))

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = self.cameraInfo.header.frame_id
                t.child_frame_id = "AU{:0>4}@gopro".format(markerIds[i][0])
                t.transform.translation.x = tvec[0][0]
                t.transform.translation.y = tvec[1][0]
                t.transform.translation.z = tvec[2][0]
                t.transform.rotation.x = rq[0][0]
                t.transform.rotation.y = rq[0][1]
                t.transform.rotation.z = rq[0][2]
                t.transform.rotation.w = rq[0][3]
                self.tf_broadcaster.sendTransform(t)

        self.aruco_pub.publish(self.cvbridge.cv2_to_imgmsg(img))



        # cv2.imwrite("../aruco_markers/gz.png", img)

        # PLOT WITH PYPLOT ###
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
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
