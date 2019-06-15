#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

from object_detect.msg import Center_msg


class Findposition:
    def __init__(self):

        rospy.on_shutdown(self.cleanup);

        # 创建cv_bridge话题
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)

        self.pub = rospy.Publisher('chatter', Center_msg, queue_size=10)   #定义发布的主题名称chatter， 消息类型String,实质是std_msgs.msg.String， 设置队列条目个数


        # 初始化订阅rgb格式图像数据的订阅者
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)

    def image_callback(self, data):
    	global center_x, center_y
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError, e:
            print e

        # 将图像从RGB转成灰度图
        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  
        #　将图像从RGB转成HSV
        self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 


        #   识别物体并获取其像素中心坐标
        morph = self.Get_contour_red1()
        red1 = self.Get_red1()

        points = self.Find_contour_red1(morph)
        mask = self.Draw_contour(points)
        center_x, center_y = self.Get_center(points)
        # print(center_x, center_y)


        # # cv2.imshow('cv_image',cv_image)

        # if center == 0:
        #     pass
        # else:
        #     center_x,center_y = self.Get_center(points)
        #     print(center_x,center_y)        # cv_image = self.Get_contour_red1()

        # 将识别后的图像转换成ROS消息并发布
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        center_ob = [center_x,center_y]
        while not rospy.is_shutdown():      #用于检测程序是否退出，是否按Ctrl-C 或其他
            rospy.loginfo("center is")  #在屏幕输出日志信息，写入到rosout节点，也可以通过rqt_console来查看
            self.pub.publish(center_ob)      #发布信息到主题
            rate.sleep()        #睡眠一定持续时间，如果参数为负数，睡眠会立即返回


        # return center_x, center_y


    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()


################################################################################
#######################      获取红色1区域　　   #############################
################################################################################
    #提取红色1的区域 
    def Get_red1(self):
        #get black area
        low_red1 = np.array([156, 43, 46])
        high_red1 = np.array([180, 255, 255])
        mask = cv2.inRange(self.hsv, low_red1, high_red1)
        red1 = cv2.bitwise_and(self.hsv, self.hsv,mask=mask)
        return red1

    #将红色1区域进行二值化处理 
    def Get_contour_red1(self):
        #change to gray
        red1 = self.Get_red1()
        # red1_gray = cv2.cvtColor(red1, cv2.COLOR_HSV2BGR)
        red1_gray = cv2.cvtColor(red1, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(red1_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_red1(self,frame):
        img_cp = self.Get_contour_red1()
        _, cnts, _ = cv2.findContours(img_cp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print (len(cnts))
        if len(cnts) == 0:
            img_boxpoints = cnts
        else:
            cnt_second = sorted(cnts, key=cv2.contourArea, reverse=True)[0]	#当没有检测到图像的时候报错，要修改
            box =cv2.minAreaRect(cnt_second)    #生成最小外接矩形
            img_boxpoints = np.int0(cv2.boxPoints(box))  #返回最小外接矩形4 个顶点
            # print img_boxpoints
        return img_boxpoints


################################################################################ 
#######################      获取轮廓的中心点坐标　　   #############################
################################################################################ 
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
        # return mask


################################################################################ 
#######################     定义ROS下的一个服务端　　   ######################
################################################################################     #主函数



################################################################################     
       
if __name__== '__main__' :
    # global center_x, center_y
    try:
        # 初始化ros节点
        rospy.init_node("object_detect", anonymous=True)
        rate = rospy.Rate(0.5) # 发布频率为10hz

        Findposition()

        # rospy.Service('object_position', Center, Findposition) #定义服务节点名称，服务的类型，处理函数

        # print(len(res))

		# d.main_process_black()	#黑色
		# d.main_process_blue()	#蓝色
		# d.main_process_Cyan()	#青色
		# d.main_process_orange()
		# d.main_process_yellow()	#	黄色（用不上）
		# d.main_process_Violet()	#紫色
		# d.main_process_red()
		# d.main_process_red1()
		# d.main_process_green()

        #rospy.loginfo("Eyes detector is started..")
        rospy.loginfo("object_detect is started.. \n Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down object_detect node."
        cv2.destroyAllWindows()


    # #cv2.namedWindow("Image") #创建窗口
    # #抓取摄像头视频图像
    # # cap = cv2.VideoCapture(0)  #创建内置摄像头变量
    # cap = cv2.VideoCapture(1)  #创建外置摄像头变量
 
    # while(cap.isOpened()):  #isOpened()  检测摄像头是否处于打开状态
    #     ret, img = cap.read()  #把摄像头获取的图像信息保存之img变量
    #     if ret == True:       #如果摄像头读取图像成功
    #         # cv2.imshow('Image',img)
    #         d = Findposition(img)
    #         # d.main_process_black()	#黑色
    #         # d.main_process_blue()	#蓝色
    #         # d.main_process_Cyan()	#青色
    #         # d.main_process_orange()
    #         # d.main_process_yellow()	#	黄色（用不上）
    #         # d.main_process_Violet()	#紫色
    #         # d.main_process_red()
    #         d.main_process_red1()
    #         # d.main_process_green()

    #         k = cv2.waitKey(100)

    #         if k == ord('a') or k == ord('A'):
    #             cv2.imwrite('test.jpg',img)
    #             break
    # cap.release()  #关闭摄像头
    # cv2.waitKey(0)
    # cv2.destroyAllWindow()
