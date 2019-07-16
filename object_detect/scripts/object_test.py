#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

from object_detect.msg import Center_msg
from std_msgs.msg import Int64

class Findposition():
    def __init__(self):
        rospy.on_shutdown(self.cleanup);
        self.flag_voice = 0
        self.flag_image = 0
        # 创建cv_bridge话题
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.pub = rospy.Publisher('chatter', Center_msg, queue_size=10)   #定义发布的主题名称chatter， 消息类型String,实质是std_msgs.msg.String， 设置队列条目个数
        rospy.Subscriber("voice/object_color", Int64, self.voice_object_callback)
        image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
	
	while not rospy.is_shutdown():
	    if self.flag_image == 1 and self.flag_voice == 1:
		self.pub.publish(self.center_ob)      #发布信息到主题
            	self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
            	rate.sleep()        #睡眠一定持续时间，如果参数为负数，睡眠会立即返回
		self.flag_image = 0 
		self.flag_voice = 0
	    else:
		pass


    def image_callback(self, data):
        if self.flag_image == 0 and self.flag_voice == 1: 
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
                frame = np.array(self.cv_image, dtype=np.uint8)
            except CvBridgeError, e:
                print e

            # 将图像从RGB转成灰度图
            self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  
            #　将图像从RGB转成HSV
            self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 

            #   识别物体并获取其像素中心坐标
            morph = self.Get_contour_Color()
            Color = self.Get_Color(self.a)

            points = self.Find_contour_Color(morph)
            mask = self.Draw_contour(points)
            center_x, center_y = self.Get_center(points)

            self.center_ob = [center_x,center_y]

            
            rospy.loginfo("center is %s", self.center_ob)  #在屏幕输出识别的物体中心位置坐标
            
            self.flag_image = 1
        else:
            pass

    def Get_Color(self,a):
    #get black area

        if self.a == 0:
            low_Color = np.array([156, 43, 46])  #粉红
            high_Color = np.array([180, 255, 255])
        elif self.a == 1:
            low_Color = np.array([100,50,50])     #蓝色
            high_Color = np.array([140,255,255])
        elif self.a == 2:
            low_Color = np.array([35, 43, 46])    #绿色
            high_Color = np.array([77, 255, 255])
        elif self.a == 3:
            low_Color = np.array([11, 43, 46])  #黄色
            high_Color = np.array([25, 255, 255])
        elif self.a == 4:
            low_Color = np.array([125, 43, 46]) #紫色
            high_Color = np.array([155, 255, 255])
        elif self.a == 5:
            low_Color = np.array([0, 43, 46])   #橙色
            high_Color = np.array([10, 255, 255])

        mask = cv2.inRange(self.hsv, low_Color, high_Color)
        Color = cv2.bitwise_and(self.hsv, self.hsv,mask=mask)
        return Color

    #将区域进行二值化处理 
    def Get_contour_Color(self):
        #change to gray
        Color = self.Get_Color(self.a)
        Color_gray = cv2.cvtColor(Color, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(Color_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_Color(self,frame):
        img_cp = self.Get_contour_Color()
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]	#当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
            # print img_boxpoints
        return img_boxpoints

    #绘制轮廓
    def Draw_contour(self,points):
        mask = np.zeros(self.gray.shape,np.uint8)
        if len(points) == 0:
            pass
        else:
            cv2.drawContours(mask,[points],-1,255,2)
        return mask

    #获取中心位置
    def Get_center(self,points):
        # global center
        if len(points) == 0:
            center = 0
        else:
            p1x,p1y=points[0,0],points[0,1]
            p3x,p3y=points[2,0],points[2,1]
            center_x,center_y=(p1x+p3x)/2,(p1y+p3y)/2
            center=(center_x,center_y)
        return center

    #绘制中心点
    def Draw_center(self,center,mask):
        # global mask1        
        if center == 0:
            pass
        else:
            cv2.circle( mask,center,1,(255,255,255),2)

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()


################################################################################     
#######################      通过语音交互识别要抓的颜色　   #########################
################################################################################ 
    def voice_object_callback(self,data):
        if self.flag_voice == 0:
        	self.a = data.data
        	self.flag_voice = 1
	else:
	    pass
        return self.a

if __name__== '__main__' : #作用是当节点停止时让
    # global center_x, center_y
    try:
        rospy.init_node("object_detect_test", anonymous=True)
        rate = rospy.Rate(10) # 发布频率为10hz

        rospy.loginfo("object_detect is started.. \n Please subscribe the ROS image.")
        Findposition()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down object_detect node."
        cv2.destroyAllWindows()
