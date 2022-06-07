#!/usr/bin/python3
# coding:utf-8

import sys
import math
import cv2
import os
from sys import platform
import time
import copy
import datetime
import ros_numpy
import numpy as np

import rospy
# from hsrb_interface     import geometry
import tf2_ros
import tf2_py as tf2
import tf

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


sys.path.append("/openpose/build/python")
try:
    from openpose import pyopenpose as op
except ImportError as e:
        print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
        raise e

# PointCloud2のフィールドの一覧
FIELDS = [
    # 点の座標(x, y, z)
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    
    # 独自に定義したフィールド
    ##　何番目の人か 
    PointField(name='person_num', offset=16, datatype=PointField.FLOAT32, count=1),
    ##　関節番号
    PointField(name='joint_num', offset=20, datatype=PointField.FLOAT32, count=1),
    ##　信頼度
    PointField(name='confidence', offset=24, datatype=PointField.FLOAT32, count=1),
]

class OpenposeRos:
    def __init__(self) -> None:
        image_topic = rospy.get_param('~image_topic', 'color/image_raw')
        depth_topic = rospy.get_param('~depth_topic', 'depth/image_raw')
        info_topic = rospy.get_param('~camera_info_topic', 'color/camera_info')
        self.proc_image_width = rospy.get_param('~process_image_width', 1120)
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

        self.average_process_time = 0

        params = dict()
        params["model_folder"] ="/openpose/models/"
        params["net_resolution"] ="-1x112"
        # Starting OpenPose
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(params)
        self.opWrapper.start()
        rospy.loginfo("OpenposeRos setup completed")
    
    def msgFilterCallback(self, color_msg, depth_msg, info_msg):

        start = time.time()
        cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        [cv_color, scale_rate] = self.scale_image(cv_color, self.proc_image_width)
        poseKeypoints = self.getPose(cv_color)
        
        if poseKeypoints is not None:
            # print("get3DPose")
            
            kernel = np.ones((10, 10), np.uint8)
            self.camera_model.fromCameraInfo(info_msg)
            depth_msg.encoding='mono16'
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "mono16")
            # [cv_depth, scale_rate] = self.scale_image(cv_depth, self.proc_image_width)
            # depth_img = cv2.GaussianBlur(depth_img,(5,5),0)
            # depth_img = cv2.blur(depth_img,(50,50))
            # depth_img = cv2.medianBlur(depth_img, 5)
            # depth_img = cv2.dilate(depth_img,kernel)
            # print(depth_msg.header)
            # print(color_msg.header)
            pose_pcd = self.get3DPose(poseKeypoints, cv_depth, self.camera_model, depth_msg.header, scale_rate)
            if pose_pcd is not None:
                self.publish3DPose(pose_pcd)
            end = time.time()

            self.average_process_time = 0.9*self.average_process_time + 0.1*(end-start)
            rospy.loginfo("FPS: "+str(1.0/self.average_process_time) + " Process Time: "+str(self.average_process_time)) 
                
    def getPose(self, cv_image):
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

    def scale_image(self, input_image, output_width):
        h, w = input_image.shape[:2]
        rate = output_width / w
        output_height = round(h * rate)
        output_image = cv2.resize(input_image, dsize=(output_width, output_height))
        return [output_image, rate]

    def get3DPose(self,keypoints, depth_img, camera_model, header, scale_rate):
        genPoints=[]
        minimumConfidence=0.01

        center_x = camera_model.cx()
        center_y = camera_model.cy()
        unit_scaling = 0.001
        constant_x = unit_scaling / camera_model.fx()
        constant_y = unit_scaling / camera_model.fy()
    
        for personNum,personData in enumerate(keypoints):
            for jointNum,jointData in enumerate(personData):
                if minimumConfidence>jointData[2]:
                    continue
                
                x_index = round(jointData[0]/scale_rate)
                y_index = round(jointData[1]/scale_rate)
                # print(depth_img.shape)
                if depth_img.shape[1] <= x_index:
                    x_index = depth_img.shape[1] - 1
                if depth_img.shape[0] <= y_index:
                    y_index = depth_img.shape[0] - 1
                depth = depth_img[y_index,x_index]
                # print("人番号："+str(personNum)+", 関節番号："+str(jointNum)+", x："+str(jointData[0])+", y："+str(jointData[1])+", depth："+str(depth_img[y_index,x_index])+", confidence："+str(jointData[2])) 

                x = (x_index - center_x) * depth * constant_x
                y = (y_index - center_y) * depth * constant_y
                z = depth/1000.0

                if z<=0:
                    continue 
                  
                genPoint=[x,y,z,personNum,jointNum,jointData[2] ]
                genPoints.append(genPoint)

        if len(genPoints)>0:
            HEADER = Header(frame_id=header.frame_id,stamp=header.stamp)
            pose_pcd=pc2.create_cloud(HEADER,FIELDS, genPoints)
            return pose_pcd
        else:
            return None

    def publish3DPose(self, pose_pcd_):
        self.pcd_pub.publish(pose_pcd_)
        
            
    

if __name__ == '__main__':
    rospy.init_node("openpose_ros")
    openpose_ros = OpenposeRos()
    rospy.spin()
    