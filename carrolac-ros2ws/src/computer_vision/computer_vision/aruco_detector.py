#!/usr/bin/env python3
import rclpy
import tf2_ros

from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
import tf2_py
from builtin_interfaces.msg import Duration, Time

from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseArray, Transform, Vector3, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
import cv2

import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

markerSize = 0.5 * 8 / 10
markerVertices = 0.5 * markerSize * np.array(
    (
        (0, -1, 1),
        (0, 1, 1),
        (0, 1, -1),
        (0, -1, -1),
    )
)

markersCenter = (
    (3.25, -0.15, 0.75, 0.0, 0.0, 3.14),
    (1.50, -4.25, 0.75, 0, 0, 1.57),
    (-1.25, -13.00, 0.75, 0, 0, 1.57),
    (-4.6, -22.30, 0.75, 0, 0, 0.8),
    (4.5, -22.30, 0.75, 0, 0, 2.4),
    (-3.25, -0.15, 0.75, 0.0, 0.0, 0),
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

def homogeneous_2_tf2(hom):
    t = Transform()
    qq = R.from_matrix(hom[0:3, 0:3]).as_quat()
    t.rotation.x = qq[0]
    t.rotation.y = qq[1]
    t.rotation.z = qq[2]
    t.rotation.w = qq[3]
    t.translation.x = hom[0, 3]
    t.translation.y = hom[1, 3]
    t.translation.z = hom[2, 3]
    return t


def tf2_2_homogeneous(transf):
    homogeneous = np.eye(4)
    rr = R.from_quat([transf.rotation.x, transf.rotation.y,
                      transf.rotation.z, transf.rotation.w])
    homogeneous[0:3, 0:3] = rr.as_matrix()
    homogeneous[0, 3] = transf.translation.x
    homogeneous[1, 3] = transf.translation.y
    homogeneous[2, 3] = transf.translation.z
    return homogeneous


class Features_Node(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.get_logger().info("Inicializando nodo...")

        self.create_subscription(
            Image, "/gopro_camera/image_raw", self.image_callback, 10
        )

        self.aruco_pub = self.create_publisher(Image, "computer_vision/aruco/image_raw", 10)
        self.error_pub = self.create_publisher(MarkerArray, "computer_vision/aruco/error_vector_array", 10)

        self.create_subscription(
            CameraInfo, "/gopro_camera/camera_info", self.camerainfo_callback, 10
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

        # plt.ion()
        # self.myfigure, self.myaxis = plt.subplots()

        self.cameraInfo = None

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_static_broadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.tf_timer = self.create_timer(0.01, self.on_tf_timer)
        self.tf_map2odom = TransformStamped()
        self.tf_map2odom.header.frame_id = "map"
        self.tf_map2odom.child_frame_id = "odom"



    def on_tf_timer(self):
        ## UPDATE ODOM IF POSSIBLE
        ## MAP TRANSFORMS

        AUtransforms = []
        pubMarkerArray = MarkerArray()
        for i in range(len(markersCenter)*2):
            AU_Frame = "AU{:0>4}".format(i)
            AU_Frame_GoPro = "AU{:0>4}@gopro".format(i)
            try:
                tf_aumap2augp = self.tf_buffer.lookup_transform(AU_Frame, AU_Frame_GoPro,
                                                            Time(sec=0))
                if self.get_clock().now().to_msg().sec - tf_aumap2augp.header.stamp.sec >= 1:
                    raise tf2_ros.TransformException("Too old")
                _aumap2augp = tf2_2_homogeneous(tf_aumap2augp.transform)

                tf_map2aumap = self.tf_buffer.lookup_transform('map', AU_Frame, Time(sec=0))
                _map2aumap = tf2_2_homogeneous(tf_map2aumap.transform)

                AUtransforms.append((_map2aumap, _aumap2augp))

                ### PUBLISH ERROR
                pubMark = Marker()
                pubMark.header.stamp = self.get_clock().now().to_msg()
                pubMark.header.frame_id = AU_Frame
                pubMark.id = i
                pubMark.action = Marker.ADD
                pubMark.type = Marker.LINE_LIST
                pubMark.points = [Point(x=0.0, y=0.0, z=0.0),
                                  Point(x=tf_aumap2augp.transform.translation.x,
                                        y=tf_aumap2augp.transform.translation.y,
                                        z=tf_aumap2augp.transform.translation.z)]
                pubMark.scale.x = 0.1
                pubMark.scale.y = 0.1
                pubMark.scale.z = 0.1
                pubMark.color.g = 1.0
                pubMark.color.a = 1.0
                pubMarkerArray.markers.append(pubMark)

                ### PUBLISH ERROR IN ROTATION
                rvec = R.from_quat([tf_aumap2augp.transform.rotation.x, tf_aumap2augp.transform.rotation.y,
                                    tf_aumap2augp.transform.rotation.z, tf_aumap2augp.transform.rotation.w]).as_rotvec()
                pubMark = Marker()
                pubMark.header.stamp = self.get_clock().now().to_msg()
                pubMark.header.frame_id = AU_Frame
                pubMark.id = 100+i
                pubMark.action = Marker.ADD
                pubMark.type = Marker.LINE_LIST
                pubMark.points = [Point(x=0.0, y=0.0, z=0.0),
                                  Point(x=rvec[0],
                                        y=rvec[1],
                                        z=rvec[2])]
                pubMark.scale.x = 0.1
                pubMark.scale.y = 0.1
                pubMark.scale.z = 0.1
                pubMark.color.b = 1.0
                pubMark.color.a = 1.0
                pubMarkerArray.markers.append(pubMark)


                ### PUBLISH ERROR MAP FIXED
                pubMark = Marker()
                pubMark.header.stamp = self.get_clock().now().to_msg()
                pubMark.header.frame_id = "map"
                pubMark.id = 1000+i
                pubMark.action = Marker.ADD
                pubMark.type = Marker.LINE_LIST
                pubMark.points = [Point(x=0.0, y=0.0, z=0.0),
                                  Point(x=tf_map2aumap.transform.translation.x,
                                        y=tf_map2aumap.transform.translation.y,
                                        z=tf_map2aumap.transform.translation.z)]
                pubMark.scale.x = 0.01
                pubMark.scale.y = 0.01
                pubMark.scale.z = 0.01
                pubMark.color.a = 1.0
                pubMark.color.g = 1.0
                pubMark.color.r = 1.0
                pubMarkerArray.markers.append(pubMark)
                #
                continue
                tf_map2aumap.transform.translation.x = 0.0
                tf_map2aumap.transform.translation.y = 0.0
                tf_map2aumap.transform.translation.z = 0.0
                hom2 = tf2_2_homogeneous(tf_map2aumap.transform)
                final_hom = np.eye(4)
                final_hom[0:3, 0:3] = _aumap2augp[0:3, 0:3]
                _aumap2augp[0:3, 0:3] = np.eye(3)
                final_hom[:, 3] = hom2@_aumap2augp[:, 3]
                # self.get_logger().info(str(_aumap2augp))
                # self.get_logger().info(str(hom2))
                # AUtransforms.append(homogeneous_2_tf2(final_hom))
            except tf2_ros.TransformException as e:
                self.get_logger().debug(f'Could not transform {AU_Frame} to {AU_Frame_GoPro}: {e}')
                pass

        error = Transform()
        error_rotvec = np.zeros(3)
        n = float(len(AUtransforms))
        for t in AUtransforms:
            _map2aumapnotras = t[0]
            _map2aumapnotras[0:3, 3] = np.zeros(3)
            _map2augpnotras = _map2aumapnotras@t[1]
            e = homogeneous_2_tf2(_map2augpnotras)
            error.translation.x += e.translation.x/n
            error.translation.y += e.translation.y/n
            error.translation.z += e.translation.z/n

            rt = t[1]
            rt[0:3, 3] = R.from_matrix(t[1][0:3, 0:3]).as_rotvec()
            rr = _map2aumapnotras@rt
            # re = R.from_matrix(rr[0:3, 0:3]).as_rotvec()/n
            re = rr[0:3, 3]
            error_rotvec += re/n
            # error_rotvec += R.from_matrix(np.linalg.inv(_map2augpnotras[0:3, 0:3])).as_rotvec()/n
            # error.rotation.x += e.rotation.x/n
            # error.rotation.y += e.rotation.y/n
            # error.rotation.z += e.rotation.z/n
            # error.rotation.w += e.rotation.w/n
        error_quat = R.from_rotvec(error_rotvec).as_quat()
        error.rotation.x = error_quat[0]
        error.rotation.y = error_quat[1]
        error.rotation.z = error_quat[2]
        error.rotation.w = error_quat[3]

        ### PUBLISH ERROR
        pubMark = Marker()
        pubMark.header.stamp = self.get_clock().now().to_msg()
        pubMark.header.frame_id = "map"
        pubMark.id = -1
        pubMark.action = Marker.ADD
        pubMark.type = Marker.LINE_LIST
        pubMark.points = [Point(x=0.0, y=0.0, z=0.0),
                          Point(x=error.translation.x, y=error.translation.y, z=error.translation.z)]
        pubMark.scale.x = 0.1
        pubMark.scale.y = 0.1
        pubMark.scale.z = 0.1
        pubMark.color.a = 1.0
        pubMark.color.r = 1.0
        pubMarkerArray.markers.append(pubMark)

        ### PUBLISH ERROR IN ROTATION
        pubMark = Marker()
        pubMark.header.stamp = self.get_clock().now().to_msg()
        pubMark.header.frame_id = "map"
        pubMark.id = -2
        pubMark.action = Marker.ADD
        pubMark.type = Marker.LINE_LIST
        pubMark.points = [Point(x=0.0, y=0.0, z=0.0),
                          Point(x=error_rotvec[0], y=error_rotvec[1], z=error_rotvec[2])]
        pubMark.scale.x = 0.1
        pubMark.scale.y = 0.1
        pubMark.scale.z = 0.1
        pubMark.color.a = 1.0
        pubMark.color.g = 1.0
        pubMark.color.b = 1.0
        pubMarkerArray.markers.append(pubMark)

        self.error_pub.publish(pubMarkerArray)

        KpR = -0.5
        KpT = -1.0
        if len(AUtransforms) > 0:
            # transforms = []
            # t = TransformStamped()
            # self.get_logger().info("Error rotvec: " + str(error_rotvec))
            self.tf_map2odom.transform.translation.x += KpT * error.translation.x * self.tf_timer.timer_period_ns * 1e-9
            self.tf_map2odom.transform.translation.y += KpT * error.translation.y * self.tf_timer.timer_period_ns * 1e-9
            self.tf_map2odom.transform.translation.z += KpT * error.translation.z * self.tf_timer.timer_period_ns * 1e-9
            map2odom_rotvec = R.from_matrix(tf2_2_homogeneous(self.tf_map2odom.transform)[0:3, 0:3]).as_rotvec()
            map2odom_rotvec += KpR * error_rotvec * self.tf_timer.timer_period_ns * 1e-9
            map2odom_quat = R.from_rotvec(map2odom_rotvec).as_quat()
            self.tf_map2odom.transform.rotation.x = map2odom_quat[0]
            self.tf_map2odom.transform.rotation.y = map2odom_quat[1]
            self.tf_map2odom.transform.rotation.z = map2odom_quat[2]
            self.tf_map2odom.transform.rotation.w = map2odom_quat[3]
            # self.get_logger().info(str(self.tf_map2odom))
            # self.get_logger().info("map2odom rotvec: " + str(map2odom_rotvec))
            # transforms.append(t)
        self.tf_map2odom.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.tf_map2odom)

    def marker_static_broadcaster(self):
        transforms = []
        for i in range(len(markersCenter)*2):
            self.get_logger().info("Creating Static Transform for AU{:0>4}".format(i))
            mc = markersCenter[int(np.floor(i/2))]
            isodd = True if i % 2 == 1 else False
            eul = R.from_euler(seq="XYZ", angles=[mc[3], mc[4], mc[5] + (np.pi if isodd else 0)])
            qua = eul.as_quat()
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = "AU{:0>4}".format(i)
            t.transform.translation.x = mc[0]
            t.transform.translation.y = mc[1]
            t.transform.translation.z = mc[2]
            t.transform.rotation.x = qua[0]
            t.transform.rotation.y = qua[1]
            t.transform.rotation.z = qua[2]
            t.transform.rotation.w = qua[3]
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

            ## HARDCODEANDO PARAMETROS PORQUE NO LOGRE HACER CALIBRACION CON GAZEBO
            # image_width = 910
            # image_height = 512
            # hfov = 2
            # focal_length = image_width / (2*np.tan(hfov / 2))
            # cx = image_width/2
            # cy = image_height/2
            # dk12 = np.sqrt(image_width**2 + image_height**2)/(2*np.tan(hfov/2)*image_width)
            # dk12 = 0.5*np.tan(hfov/2)**2
            fx = 292.5241
            fy = 292.4264
            cx = 454.60223
            cy = 255.70208
            dk = [-0.000305, -0.000239, 0.000055, 0.000027, 0.000000]
            if k[0, 0] != fx:
                self.get_logger().debug("Camera Intrinsic fx differs from expected. Got: " + str(k[0, 0]) +\
                                        "\tExpected: " + str(fx) + ". Hard coding...")
                k[0, 0] = fx

            if k[1, 1] != fy:
                self.get_logger().debug("Camera Intrinsic fy differs from expected. Got: " + str(k[1, 1]) +\
                                        "\tExpected: " + str(fy) + ". Hard coding...")
                k[1, 1] = fy

            if k[0, 2] != cx:
                self.get_logger().debug("Camera Intrinsic cx differs from expected. Got: " + str(k[0, 2]) +\
                                        "\tExpected: " + str(cx) + ". Hard coding...")
                k[0, 2] = cx

            if k[1, 2] != cy:
                self.get_logger().debug("Camera Intrinsic cy differs from expected. Got: " + str(k[1, 2]) +\
                                        "\tExpected: " + str(cy) + ". Hard coding...")
                k[1, 2] = cy
            for _d in range(5):
                if d[_d] != dk[_d]:
                    self.get_logger().debug("Camera Intrinsic Distortion k" + str(_d) + " differs from expected.\
                                            Got: " + str(d[_d]) +\
                                            "\tExpected: " + str(dk[_d]) + ". Hard coding...")
                    d[_d] = dk[_d]
            ###########################################################

            # self.get_logger().info(str(k))
            # self.get_logger().info(str(d))

            self.get_logger().debug("Markers ID: " + str(markerIds))
            transforms = []
            for i in range(nmarkers):
                # mp = np.array(markerPoints[0][0:4, 0:3])
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
                transforms.append(t)
            self.tf_broadcaster.sendTransform(transforms)

        self.aruco_pub.publish(self.cvbridge.cv2_to_imgmsg(img))

        # cv2.imwrite("../aruco_markers/gz.png", img)

        # PLOT WITH PYPLOT ###
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # if hasattr(self, 'plotimage'):
        #     self.plotimage.set_data(img)
        # else:
        #     self.plotimage = self.myaxis.imshow(img)
        # self.myfigure.canvas.draw()
        # self.myfigure.canvas.flush_events()
        #########################
        # input()


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Features_Node())


if __name__ == "__main__":
    main()
