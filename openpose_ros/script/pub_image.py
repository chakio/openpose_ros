#!/usr/bin/python
# coding:utf-8
# From Python
# It requires OpenCV installed for Python
# import hsrb_interface
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
import copy
import ros_numpy
import tf
import datetime
# Remember to add your installation path here
# Option a
dir_path = os.path.dirname("/openpose/build/examples")
#if pldir_pathatform == "win32": sys.path.append(dir_path + '/../../python/openpose/');
#sys.path.append('../../python')
# Option b
# If you run `make install` (default path is `/usr/local/python` for Ubuntu), you can also access the OpenPose/python module from there. This will install OpenPose and the python library at your desired installation path. Ensure that this is in your python path in order to use it.
sys.path.append('/usr/local/python')
framenum=0
# Parameters for OpenPose. Take a look at C++ OpenPose example for meaning of components. Ensure all below are filled
try:
    from openpose import *
except:
    raise Exception('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
    
# 点の座標を定義するフレームの名前

################################################################################
#                             Grobal Definition                                #
################################################################################

subscribedTime  = 0
getImage        = False
getPcd          = False
first           = True


def generatePcd(keypoints,pcd):
    global subscribedTime
    genPoints=[]
    minimumConfidence=0.01
    points=ros_numpy.numpify(pcd)
    #points = pc2.read_points(pcd,field_names = ("x", "y", "z"))
   
    for personNum,personData in enumerate(keypoints):
        for jointNum,jointData in enumerate(personData):
            #print("人番号："+str(personNum)+",関節番号："+str(jointNum)+",x："+str(round(jointData[0]))+",y："+str(jointData[1])+",z："+str(jointData[2])) 
            #index=round(jointData[0])+round(jointData[1])*640
            #print(str(points[int(index)][0])+","+str(points[int(index)][1])+","+str(points[int(index)][2]))
            if minimumConfidence<jointData[2]: #and not math.isnan(points[int(index)][0]) and not math.isnan(points[int(index)][1]) and not math.isnan(points[int(index)][2]): 
                # [x, y, z, rgb, humanNum, jointNum]
                #genPoint=[points[int(index)][0],points[int(index)][1],points[int(index)][2],0xff0000,personNum,jointNum,jointData[2]]
                if int(round(jointData[1]))<480 and int(round(jointData[0]))<640:
                    genPoint=[points[int(round(jointData[1])),int(round(jointData[0]))][0],points[int(round(jointData[1])),int(round(jointData[0]))][1],points[int(round(jointData[1])),int(round(jointData[0]))][2],0xff0000,personNum,jointNum,jointData[2],jointData[1] ,jointData[0] ]
                    genPoints.append(genPoint)
    if len(genPoints)>0:
        HEADER = Header(frame_id='/camera_rgb_optical_frame',stamp=subscribedTime)
        genPCD=pc2.create_cloud(HEADER,FIELDS, genPoints)
        print "--------detectPeople---------"
        return genPCD
    return 0

def imageCB(data):
    #print('imageCB')
    global original_image
    global getImage 
    getImage=True
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        original_image = copy.copy(cv_image)

    except CvBridgeError as e:
        print('Cv_Bridge_Error')


if __name__ == '__main__':
    global original_image
    global picture
    rospy.init_node("hsr_ros_openpose")
    bridge      = CvBridge()

    image_sub   = rospy.Subscriber("/camera/rgb/image_raw", Image, imageCB)
    image_pub = rospy.Publisher("/openpose_image",Image)

    params      = dict()
    params["logging_level"] = 3
    params["output_resolution"] = "-1x-1"
    params["net_resolution"] = "-1x368"
    params["model_pose"] = "BODY_25"
    params["alpha_pose"] = 0.6
    params["scale_gap"] = 0.3
    params["scale_number"] = 1
    params["render_threshold"] = 0.05
    # If GPU version is built, and multiple GPUs are available, set the ID here
    params["num_gpu_start"] = 0
    params["disable_blending"] = False
    # Ensure you point to the correct path where models are located
    params["default_model_folder"] ="/openpose/models/"
    # Construct OpenPose object allocates GPU memory
    openpose = OpenPose(params)
    print "--------------Openpose wakeup--------------"
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if first:
            first = False
        if getImage:#process updated image only
            keypoints, output_image = openpose.forward(original_image, True)
            #print(keypoints)
            #opPcd=generatePcd(keypoints,pcdDataNow)
            cv2.imshow("output", output_image)
            cv2.waitKey(1)
            try:
                image_pub.publish(bridge.cv2_to_imgmsg(output_image, "bgr8"))
                print "--------------pub image--------------"
            except CvBridgeError as e:
                print(e)
            getImage=False
        r.sleep()



   

