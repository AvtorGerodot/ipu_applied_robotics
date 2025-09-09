#!/usr/bin/python3

#
#   Developer : Alexey Zakharov (alexey.zakharov@vectioneer.com)
#   All rights reserved. Copyright (c) 2017-2020 VECTIONEER.
#

import motorcortex
import time
import mcx_tracking_cam_pb2 as tracking_cam_msg
from aruco_msgs.msg import Marker, MarkerArray
from blobs_msgs.msg import PointArray
from circle_msgs.msg import CircleArray
from line_msgs.msg import LineArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import rospy
import tf
from math import cos, sin, sqrt
import numpy as np
Markers = tracking_cam_msg.Markers
Blobs = tracking_cam_msg.Blobs
aruco_tf = tf.TransformBroadcaster()
pub_aruco = rospy.Publisher('markers', MarkerArray, queue_size=1)
pub_blobs = rospy.Publisher('blobs', PointArray, queue_size=1)
pub_new_blobs = rospy.Publisher('new_blobs', PointArray, queue_size=1)
pub_lines = rospy.Publisher('lines', LineArray, queue_size=1)
pub_circles = rospy.Publisher('circles', CircleArray, queue_size=1)
pub_image = rospy.Publisher('image', Image, queue_size=1)
bridge = CvBridge()
ip = "192.168.42.1"
frame = "tracking_cam3"


def onLog(val):
    print(val[0].value)


def onError(val):
    try:
        errors = motorcortex.ErrorList()
        if errors.ParseFromString(val[0].value):
            print(errors)
    except Exception as e:
        print(e)


def onProjection(val):
    try:
        projection = tracking_cam_msg.Projection()
        if projection.ParseFromString(val[0].value):
            print(projection)
    except Exception as e:
        print(e)


def onBlob(val):
    try:
        blobs = tracking_cam_msg.Blobs()
        if blobs.ParseFromString(val[0].value):
            send_blobs_to_ros(blobs.value)
    except Exception as e:
        print(e)

def onBlobNew(val):
    try:
        blobs = tracking_cam_msg.Blobs()
        if blobs.ParseFromString(val[0].value):
            send_new_blobs_to_ros(blobs.value)
    except Exception as e:
        print(e)


def onMarker(val):
    try:
        markers = tracking_cam_msg.Markers()
        if markers.ParseFromString(val[0].value):
            send_markers_to_ros(markers.value)
    except Exception as e:
        print(e)


def onCircle(val):
    
    try:
        circles = tracking_cam_msg.Circles()
        if circles.ParseFromString(val[0].value):
            print(circles.value)
            send_circles_to_ros(circles.value)
    except Exception as e:
        print(e)


def onLines(val):
    try:
        lines = tracking_cam_msg.Lines()
        if lines.ParseFromString(val[0].value):
            send_lines_to_ros(lines.value)
    except Exception as e:
        print(e)

def send_blobs_to_ros(blobs):
    msg_array = PointArray()
    msg_array.header.stamp = rospy.Time.now()
    msg_array.header.frame_id = frame
    for blob in blobs:
        msg = Point()
        msg.x = float(blob.cx)
        msg.y = float(blob.cy)
        msg_array.points.append(msg)
    pub_blobs.publish(msg_array)

def send_new_blobs_to_ros(blobs):
    msg_array = PointArray()
    msg_array.header.stamp = rospy.Time.now()
    msg_array.header.frame_id = frame
    for blob in blobs:
        msg = Point()
        msg.x = float(blob.cx)
        msg.y = float(blob.cy)
        msg_array.points.append(msg)
    pub_new_blobs.publish(msg_array)

def send_circles_to_ros(circles):
    msg_array = CircleArray()
    msg_array.header.stamp = rospy.Time.now()
    msg_array.header.frame_id = frame
    for circle in circles:
        msg = Circle()
        msg.center.x = float(circle.x)
        msg.center.y = float(circle.y)
        msg.radius = float(circle.r)
        msg_array.circles.append(msg)
    pub_circles.publish(msg_array)


def send_lines_to_ros(lines):
    msg_array = LineArray()
    msg_array.header.stamp = rospy.Time.now()
    msg_array.header.frame_id = frame
    for line in lines:
        msg = Line()
        msg.first.x = float(line.x0)
        msg.first.y = float(line.y0)
        msg.second.x = float(line.x1)
        msg.second.y = float(line.y1)
        msg_array.lines.append(msg)
    pub_lines.publish(msg_array)


def send_markers_to_ros(markers):
    msg_array = MarkerArray()
    msg_array.header.stamp = rospy.Time.now()
    msg_array.header.frame_id = frame
    for marker in markers:
        msg = Marker()
        msg.id = marker.id
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame
        msg.pose.pose.position.x = marker.transVec3[0]
        msg.pose.pose.position.y = marker.transVec3[1]
        msg.pose.pose.position.z = marker.transVec3[2]
        ax = marker.rotVec3[0]
        ay = marker.rotVec3[1]
        az = marker.rotVec3[2]
        angle = sqrt(ax*ax + ay*ay + az*az)
        cosa = cos(angle*0.5)
        sina = sin(angle*0.5)
        msg.pose.pose.orientation.x = ax*sina/angle
        msg.pose.pose.orientation.y = ay*sina/angle
        msg.pose.pose.orientation.z = az*sina/angle
        msg.pose.pose.orientation.w = cosa
        msg_array.markers.append(msg)
        aruco_tf.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                               (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                               msg.header.stamp, "marker_"+str(msg.id), frame)
    pub_aruco.publish(msg_array)
def onImage(val):
    image = cv2.imdecode(np.frombuffer(val[1].value, np.uint8), -1)
    pub_image.publish(bridge.cv2_to_imgmsg(image, "bgr8"))

def main():
    global ip, frame
    rospy.init_node('motorcortex_proxy')
    if rospy.has_param("~camera_ip"):
        ip = rospy.get_param("~camera_ip")
        print("get ip param:"+str(ip))
    else:
        print("no ip param -> use default ip")
    if rospy.has_param("~camera_frame"):
        frame = rospy.get_param("~camera_frame")
        print("get frame param:"+frame)
    else:
        print("no frame param -> use default frame")
    # Creating empty object for parameter tree
    parameter_tree = motorcortex.ParameterTree()

    # Loading protobuf types and hashes
    motorcortex_types = motorcortex.MessageTypes()

    # Open request connection
    dir_path = os.path.dirname(os.path.realpath(__file__))
    req, sub = motorcortex.connect("ws://"+ip+":5558:5557", motorcortex_types, parameter_tree,
                                   certificate=dir_path+"/motorcortex.crt", timeout_ms=1000,
                                   login="root", password="vectioneer")

    # subscribe example
    subscription2 = sub.subscribe(
        ["root/Processing/BlobDetector/blobBuffer"], "blob", 1)
    subscription2.get()
    subscription2.notify(onBlob)

    subscription1 = sub.subscribe(
        ["root/Processing/ArucoDetector/markerBuffer"], "marker", 1)
    subscription1.get()
    subscription1.notify(onMarker)

    subscription3 = sub.subscribe(
        ["root/Processing/CircleDetector/markerCircle"], "circle", 1)
    subscription3.get()
    subscription3.notify(onCircle)
    
    subscription4 = sub.subscribe(
        ["root/Processing/BlobDetectorNew/blobBuffer"], "blob", 1)
    subscription4.get()
    subscription4.notify(onBlobNew)

    subscription5 = sub.subscribe(["root/Processing/ProjectionModule/projectionBuffer"], "projection", 1)
    subscription5.get()
    subscription5.notify(onProjection)
    subscription6 = sub.subscribe(["root/Comm_task/utilization_max","root/Processing/image"], "camera", 1)
    subscription6.notify(onImage)

    #
    # subscription2 = sub.subscribe(["root/Processing/BlobDetector/blobBuffer"], "blob", 1)
    # subscription2.get()
    # subscription2.notify(onBlob)

    # subscription3 = sub.subscribe(["root/Logic/activeErrors"], "error", 1000)
    # subscription3.get()
    # subscription3.notify(onError)

    # subscription4 = sub.subscribe(["root/logger/logOut"], "log", 1)
    # subscription4.get()
    # subscription4.notify(onLog)

    # request/reply example
    # for x in range(100):
    #     marker_reply = req.getParameter("root/Processing/ArucoDetector/markerBuffer")
    #     markers = tracking_cam_msg.Markers()
    #     markers.ParseFromString(marker_reply.get().value)
    #     print(markers)
    #     time.sleep(1)

    # time.sleep(100)
    # req.close()
    # sub.close()


if __name__ == '__main__':
    main()
    while not rospy.is_shutdown():
        try:
            rospy.sleep(0.1)
        except Exception as e:
            print(e)
            req.close()
            sub.close()
