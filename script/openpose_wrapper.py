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
################################################################################
#                             Grobal Definition                                #
################################################################################
# ロボット機能の初期化 ----------------------------------------------------------
"""
robot       = hsrb_interface.Robot()
omni_base   = robot.get('omni_base')
whole_body  = robot.get('whole_body')
gripper     = robot.get('gripper')
tts         = robot.get('default_tts')
"""

subscribedTime  = 0
getImage        = False
getPcd          = False
first           = True
openpose_start  = False
picture         = False
interval        = 0
starttime       = 0

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
    
def pcdCB(data):
    #print('pcdCB')
    global pcdData
    global getPcd
    global subscribedTime
    global interval
    pcdData         = data
    getPcd          = True
    subscribedTime  = rospy.Time.now() 
    interval = rospy.get_time()
    #print (subscribedTime)

def startCB(data):
    global openpose_start
    global picture
    if data.data=="start":
        print "--------------Openpose Start--------------"
        framenum=0
        openpose_start=True
        status_pub.publish(True)
    if data.data=="picture_start":
        print "--------------Openpose Picture Start--------------"
        framenum=0
        openpose_start=True
        picture = True
        status_pub.publish(True)
    elif data.data=="end":
        print "--------------Openpose End--------------"
        openpose_start=False
        picture = False


if __name__ == '__main__':
    global original_image
    global picture
    #rospy.init_node("hsr_ros_openpose")
    bridge      = CvBridge()

    image_sub   = rospy.Subscriber("/camera/rgb/image_raw", Image, imageCB)
    pcd_sub     = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, pcdCB)
    start_Sub   = rospy.Subscriber("/hsr_openpose_start", String, startCB)
    pcd_pub     = rospy.Publisher('/hsr_openpose_joints', PointCloud2, queue_size=3)
    status_pub  = rospy.Publisher('/hsr_openpose_status', Bool, queue_size=10)
    image_pub = rospy.Publisher("/hsr_openpose_image",Image)
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
    r = rospy.Rate(5) # 10hz
    tflistener=tf.TransformListener()
    interval = rospy.get_time()
    starttime = rospy.get_time()
    while not rospy.is_shutdown():
        if first:
            #tts.say("open pose start")
            first = False
        if rospy.get_time()-interval>5:
            #tts.say("カメラ画像が取得できません")
            rospy.sleep(5)
        if rospy.get_time()-starttime>5:
            try:
                (trans,rot)=tflistener.lookupTransform('/camera_link','/base_link',rospy.Time(0))
            except:
                #tts.say("TFが取得できません")
                rospy.sleep(5)

        if getImage and getPcd and openpose_start:
            pcdDataNow=pcdData
            keypoints, output_image = openpose.forward(original_image, True)
            #print(keypoints)
            opPcd=generatePcd(keypoints,pcdDataNow)
            if opPcd:
                #framenum +=1
                pcd_pub.publish(opPcd)
                now = datetime.datetime.now()
                #filename = '
                #filename = '/home/ytnpc2018b/openposeresult/'+str(framenum)+".jpg"
                #print filename
                #3cv2.imwrite(filename,output_image )
                #cv2.imshow("output", output_image)
                #cv2.waitKey(1)
                if picture:
                    try:
                        image_pub.publish(self.bridge.cv2_to_imgmsg(output_image, "bgr8"))
                    except CvBridgeError as e:
                        print(e)
            r.sleep()
            
            getImage=False



   

