#!/usr/bin/python3
# coding:utf-8

import sys
import math
import cv2
import os
from sys import platform
import rospy
# from hsrb_interface     import geometry
import tf2_ros
import tf2_py as tf2

from std_msgs.msg import Header
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2,PointField
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image, CameraInfo
import image_geometry

import copy
import tf
import datetime
import ros_numpy
import numpy as np

sys.path.append("/openpose/build/python")
framenum=0
try:
    from openpose import pyopenpose as op
except ImportError as e:
        print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
        raise e
    
# 点の座標を定義するフレームの名前

# PointCloud2のフィールドの一覧
FIELDS = [
    # 点の座標(x, y, z)
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    # 点の色(RGB)
    # 赤: 0xff0000, 緑:0x00ff00, 青: 0x0000ff
    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
    
    # 独自に定義したフィールド
    ##　何番目の人か 
    PointField(name='person_num', offset=16, datatype=PointField.FLOAT32, count=1),
    ##　関節番号
    PointField(name='joint_num', offset=20, datatype=PointField.FLOAT32, count=1),
    ##　信頼度
    PointField(name='confidence', offset=24, datatype=PointField.FLOAT32, count=1),
    ##　画素の位置(x)
    PointField(name='image_x', offset=28, datatype=PointField.FLOAT32, count=1),
    ##　画素の位置(y)
    PointField(name='image_y', offset=32, datatype=PointField.FLOAT32, count=1),
]

class OpenposeRos:
    def __init__(self) -> None:
        image_topic = rospy.get_param('~image_topic', 'color/image_raw')
        depth_topic = rospy.get_param('~depth_topic', 'depth/image_raw')
        info_topic = rospy.get_param('~camera_info_topic', 'color/camera_info')
        print(image_topic, depth_topic, info_topic)
        self.bridge      = CvBridge()
        color_sub = message_filters.Subscriber(image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        info_sub = message_filters.Subscriber(info_topic, CameraInfo)

        ts = message_filters.TimeSynchronizer([color_sub, depth_sub, info_sub], 10)
        ts.registerCallback(self.msgFilterCallback)
        self.pcd_pub     = rospy.Publisher('/openpose_joints', PointCloud2, queue_size=3)
        self.output_pub = rospy.Publisher("/openpose_image",Image, queue_size=1)
        self.camera_model = image_geometry.PinholeCameraModel()

        params = dict()
        params["model_folder"] ="/openpose/models/"
        params["net_resolution"] ="-1x256"
        # Starting OpenPose
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(params)
        self.opWrapper.start()
        rospy.loginfo("OpenposeRos setup completed")
    
    def msgFilterCallback(self, color_msg, depth_msg, info_msg):
        poseKeypoints = self.getPose(color_msg)
        
        if poseKeypoints is not None:
            # print("get3DPose")
            kernel = np.ones((10, 10), np.uint8)
            self.camera_model.fromCameraInfo(info_msg)
            depth_msg.encoding='mono16'
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "mono16")
            # depth_img = cv2.GaussianBlur(depth_img,(5,5),0)
            # depth_img = cv2.blur(depth_img,(50,50))
            # depth_img = cv2.medianBlur(depth_img, 5)
            depth_img = cv2.dilate(depth_img,kernel)
            self.get3DPose(poseKeypoints, depth_img, self.camera_model, depth_msg.header)

    def getPose(self, colorMsg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(colorMsg, "bgr8")
        except CvBridgeError as e:
            print('Cv_Bridge_Error')
            return None
        datum = op.Datum()
        datum.cvInputData = cv_image
        self.opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        poseKeypoints = copy.deepcopy(datum.poseKeypoints)
        # print(type(poseKeypoints))
        # print(poseKeypoints)
        try:
            self.output_pub.publish(self.bridge.cv2_to_imgmsg(datum.cvOutputData, "bgr8"))
        except CvBridgeError as e:
            print(e)
        return poseKeypoints

    def get3DPose(self,keypoints, depth_img, camera_model, header):
        genPoints=[]
        minimumConfidence=0.01

        # points=ros_numpy.numpify(pcd)
        #points = pc2.read_points(pcd,field_names = ("x", "y", "z"))

        center_x = camera_model.cx()
        center_y = camera_model.cy()
        unit_scaling = 0.001
        constant_x = unit_scaling / camera_model.fx()
        constant_y = unit_scaling / camera_model.fy()

    
        for personNum,personData in enumerate(keypoints):
            for jointNum,jointData in enumerate(personData):
               
                x_index = round(jointData[0])
                y_index = round(jointData[1])
                # print(depth_img.shape)
                if depth_img.shape[1] > x_index and depth_img.shape[0] > y_index:
                    print("人番号："+str(personNum)+", 関節番号："+str(jointNum)+", x："+str(jointData[0])+", y："+str(jointData[1])+", depth："+str(depth_img[y_index,x_index])+", confidence："+str(jointData[2])) 

                    depth = depth_img[y_index,x_index]
                    x = (x_index - center_x) * depth * constant_x
                    y = (y_index - center_y) * depth * constant_y
                    z = depth/1000.0


                    #print(str(points[int(index)][0])+","+str(points[int(index)][1])+","+str(points[int(index)][2]))
                    if minimumConfidence<jointData[2] and z>0 : #and not math.isnan(points[int(index)][0]) and not math.isnan(points[int(index)][1]) and not math.isnan(points[int(index)][2]): 
                        # [x, y, z, rgb, humanNum, jointNum]
                        #genPoint=[points[int(index)][0],points[int(index)][1],points[int(index)][2],0xff0000,personNum,jointNum,jointData[2]]
                        genPoint=[x,y,z,0xff0000,personNum,jointNum,jointData[2],jointData[1] ,jointData[0] ]
                        genPoints.append(genPoint)
                else:
                    print(x_index, y_index)
        if len(genPoints)>0:
            HEADER = Header(frame_id=header.frame_id,stamp=header.stamp)
            genPCD=pc2.create_cloud(HEADER,FIELDS, genPoints)
            self.pcd_pub.publish(genPCD)
            return genPCD
    

if __name__ == '__main__':
    rospy.init_node("openpose_ros")
    openpose_ros = OpenposeRos()
    rospy.spin()
    