#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

from object_detect.msg import Center_msg


class Findposition:
    def __init__(self, a):
        rospy.on_shutdown(self.cleanup);

        # 创建cv_bridge话题
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)

        self.pub = rospy.Publisher('chatter', Center_msg, queue_size=10)   #定义发布的主题名称chatter， 消息类型String,实质是std_msgs.msg.String， 设置队列条目个数


        # 初始化订阅rgb格式图像数据的订阅者
        if a == 1:
        	self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback_Pink, queue_size=1)
        elif a == 2:
        	self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback_blue, queue_size=1)        	
        elif a == 3:
        	self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback_Cyan, queue_size=1) 
        elif a == 4:
        	self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback_yellow, queue_size=1) 
        elif a == 5:
        	self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback_Violet, queue_size=1) 
        elif a == 6:
        	self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback_orange, queue_size=1) 
        else:
        	pass

################################################################################
#######################      获取粉红色物体的中心坐标　　   ###########################
################################################################################
#####################      获取粉红色物体的中心坐标回调函数　　   ######################
    def image_callback_Pink(self, data):
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
        morph = self.Get_contour_Pink()
        Pink = self.Get_Pink()

        points = self.Find_contour_Pink(morph)
        mask = self.Draw_contour(points)
        center_x, center_y = self.Get_center(points)

        center_ob = [center_x,center_y]

        while not rospy.is_shutdown():      #用于检测程序是否退出，是否按Ctrl-C 或其他
            rospy.loginfo("center is %s", center_ob)  #在屏幕输出识别的物体中心位置坐标
            self.pub.publish(center_ob)      #发布信息到主题
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            rate.sleep()        #睡眠一定持续时间，如果参数为负数，睡眠会立即返回

#######################      获取粉红色区域　　   #############################
    #提取红色1的区域 
    def Get_Pink(self):
        #get black area
        low_Pink = np.array([156, 43, 46])
        high_Pink = np.array([180, 255, 255])
        mask = cv2.inRange(self.hsv, low_Pink, high_Pink)
        Pink = cv2.bitwise_and(self.hsv, self.hsv,mask=mask)
        return Pink

    #将红色1区域进行二值化处理 
    def Get_contour_Pink(self):
        #change to gray
        Pink = self.Get_Pink()
        # Pink_gray = cv2.cvtColor(Pink, cv2.COLOR_HSV2BGR)
        Pink_gray = cv2.cvtColor(Pink, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(Pink_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_Pink(self,frame):
        img_cp = self.Get_contour_Pink()
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
#######################      获取蓝色物体的中心坐标　　   ###########################
################################################################################
#####################      获取蓝色物体的中心坐标回调函数　　   ######################
    def image_callback_blue(self, data):
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
        morph = self.Get_contour_blue()
        Pink = self.Get_blue()

        points = self.Find_contour_blue(morph)
        mask = self.Draw_contour(points)
        center_x, center_y = self.Get_center(points)

        center_ob = [center_x,center_y]

        while not rospy.is_shutdown():      #用于检测程序是否退出，是否按Ctrl-C 或其他
            rospy.loginfo("center is %s", center_ob)  #在屏幕输出识别的物体中心位置坐标
            self.pub.publish(center_ob)      #发布信息到主题
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            rate.sleep()        #睡眠一定持续时间，如果参数为负数，睡眠会立即返回

#######################      获取蓝色区域　　   #############################
    #提取蓝色的区域 
    def Get_blue(self):
        #get black area
        low_blue = np.array([100,50,50])    #　创建黑色最低的hsv的范围数组
        high_blue = np.array([140,255,255]) #　创建黑色最高的hsv的范围数组
        mask = cv2.inRange(self.hsv, low_blue, high_blue)   # 利用cv2.inRange函数设阈值，去除背景部分
        blue = cv2.bitwise_and(self.hsv, self.hsv, mask=mask) #对图形的二进制数据进行“与”操作
        # blue = cv2.bitwise_and(self.hsv, self.hsv, mask=mask)

        return blue

    #将蓝色区域进行二值化处理 
    def Get_contour_blue(self):
        #change to gray
        blue = self.Get_blue()  #获取带有颜色区域的图片
        # blue_gray = cv2.cvtColor(blue, cv2.COLOR_HSV2BGR)
        # blue_gray = cv2.cvtColor(blue_gray, cv2.COLOR_BGR2GRAY)
        blue_gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)  #将图形转成灰色图
        
        #binaryzation
        _, thresh = cv2.threshold(blue_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)    #对灰度图进行固定阈值二值化（使用Otsu’s 二值化）
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3)) #利用形态学滤波开运算(open)来进行图形处理，先腐蚀后膨胀的
        # open = cv2.erode(img_morph, (3,3), img_morph, iterations=2)    #腐蚀图像
        # img_morph = cv2.dilate(open, (3,3), open, iterations=2)   #膨胀图像
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_blue(self,img):
        img_cp = self.Get_contour_blue()    #img_cp不是灰度图而是二值图
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
#######################      获取青色物体的中心坐标　　   ############################
################################################################################
#####################      获取青色物体的中心坐标回调函数　　   #######################
    def image_callback_Cyan(self, data):
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
        morph = self.Get_contour_Cyan()
        Pink = self.Get_Cyan()

        points = self.Find_contour_Cyan(morph)
        mask = self.Draw_contour(points)
        center_x, center_y = self.Get_center(points)

        center_ob = [center_x,center_y]

        while not rospy.is_shutdown():      #用于检测程序是否退出，是否按Ctrl-C 或其他
            rospy.loginfo("center is %s", center_ob)  #在屏幕输出识别的物体中心位置坐标
            self.pub.publish(center_ob)      #发布信息到主题
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            rate.sleep()        #睡眠一定持续时间，如果参数为负数，睡眠会立即返回
#######################      获取青色区域　　   #############################
    #提取绿色的区域 
    def Get_Cyan(self):
        #get black area
        low_Cyan = np.array([35, 43, 46])
        high_Cyan = np.array([77, 255, 255])
        mask = cv2.inRange(self.hsv, low_Cyan, high_Cyan)
        Cyan = cv2.bitwise_and(self.hsv, self.hsv,mask=mask)
        return Cyan

    #将绿色区域进行二值化处理 
    def Get_contour_Cyan(self):
        #change to gray
        Cyan = self.Get_Cyan()
        # Cyan_gray = cv2.cvtColor(Cyan, cv2.COLOR_HSV2BGR)
        Cyan_gray = cv2.cvtColor(Cyan, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(Cyan_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_Cyan(self,img):
        img_cp = self.Get_contour_Cyan()
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
#######################      获取黄色物体的中心坐标　　   #############################
################################################################################
#####################      获取黄色物体的中心坐标回调函数　　   ########################
    def image_callback_yellow(self, data):
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
        morph = self.Get_contour_yellow()
        Pink = self.Get_yellow()

        points = self.Find_contour_yellow(morph)
        mask = self.Draw_contour(points)
        center_x, center_y = self.Get_center(points)

        center_ob = [center_x,center_y]

        while not rospy.is_shutdown():      #用于检测程序是否退出，是否按Ctrl-C 或其他
            rospy.loginfo("center is %s", center_ob)  #在屏幕输出识别的物体中心位置坐标
            self.pub.publish(center_ob)      #发布信息到主题
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            rate.sleep()        #睡眠一定持续时间，如果参数为负数，睡眠会立即返回


#######################      获取黄色区域　　   #############################
    #提取橙色的区域 
    def Get_yellow(self):
        #get black area
        low_yellow = np.array([11, 43, 46])
        high_yellow = np.array([25, 255, 255])
        mask = cv2.inRange(self.hsv, low_yellow, high_yellow)
        yellow = cv2.bitwise_and(self.hsv, self.hsv,mask=mask)
        return yellow

    #将橙色区域进行二值化处理 
    def Get_contour_yellow(self):
        #change to gray
        yellow = self.Get_yellow()
        # yellow_gray = cv2.cvtColor(yellow, cv2.COLOR_HSV2BGR)
        yellow_gray = cv2.cvtColor(yellow, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(yellow_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_yellow(self,img):
        img_cp = self.Get_contour_yellow()
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
#######################      获取紫色物体的中心坐标　　   ############################
################################################################################
#####################      获取紫色物体的中心坐标回调函数　　   ########################
    def image_callback_Violet(self, data):
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
        morph = self.Get_contour_Violet()
        Pink = self.Get_Violet()

        points = self.Find_contour_Violet(morph)
        mask = self.Draw_contour(points)
        center_x, center_y = self.Get_center(points)

        center_ob = [center_x,center_y]

        while not rospy.is_shutdown():      #用于检测程序是否退出，是否按Ctrl-C 或其他
            rospy.loginfo("center is %s", center_ob)  #在屏幕输出识别的物体中心位置坐标
            self.pub.publish(center_ob)      #发布信息到主题
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            rate.sleep()        #睡眠一定持续时间，如果参数为负数，睡眠会立即返回

#######################      获取紫色区域　　   #############################
    #提取紫色的区域 
    def Get_Violet(self):
        #get black area
        low_Violet = np.array([125, 43, 46])
        high_Violet = np.array([155, 255, 255])
        mask = cv2.inRange(self.hsv, low_Violet, high_Violet)
        Violet = cv2.bitwise_and(self.hsv, self.hsv,mask=mask)
        return Violet

    #将紫色区域进行二值化处理 
    def Get_contour_Violet(self):
        #change to gray
        Violet = self.Get_Violet()
        # Violet_gray = cv2.cvtColor(Violet, cv2.COLOR_HSV2BGR)
        Violet_gray = cv2.cvtColor(Violet, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(Violet_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_Violet(self,img):
        img_cp = self.Get_contour_Violet()
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
#######################      获取橙色物体的中心坐标　　   #############################
################################################################################
#####################      获取橙色物体的中心坐标回调函数　　   ########################
    def image_callback_orange(self, data):
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
        morph = self.Get_contour_orange()
        Pink = self.Get_orange()

        points = self.Find_contour_orange(morph)
        mask = self.Draw_contour(points)
        center_x, center_y = self.Get_center(points)

        center_ob = [center_x,center_y]

        while not rospy.is_shutdown():      #用于检测程序是否退出，是否按Ctrl-C 或其他
            rospy.loginfo("center is %s", center_ob)  #在屏幕输出识别的物体中心位置坐标
            self.pub.publish(center_ob)      #发布信息到主题
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            rate.sleep()        #睡眠一定持续时间，如果参数为负数，睡眠会立即返回

#######################      获取橙色区域　　   #############################
    #提取红色的区域 
    def Get_orange(self):
        #get black area
        low_orange = np.array([0, 43, 46])
        high_orange = np.array([10, 255, 255])
        mask = cv2.inRange(self.hsv, low_orange, high_orange)
        orange = cv2.bitwise_and(self.hsv, self.hsv,mask=mask)
        return orange

    #将红色区域进行二值化处理 
    def Get_contour_orange(self):
        #change to gray
        orange = self.Get_orange()
        # orange_gray = cv2.cvtColor(orange, cv2.COLOR_HSV2BGR)
        orange_gray = cv2.cvtColor(orange, cv2.COLOR_BGR2GRAY)
        
        #binaryzation
        _, thresh = cv2.threshold(orange_gray, 10, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (3,3))
        # cv2.erode(img_morph, (3,3), img_morph, iterations=2)
        # cv2.dilate(img_morph, (3,3), img_morph, iterations=2)
        return img_morph

    #获取中心区域轮廓及坐标 
    def Find_contour_orange(self,img):
        img_cp = self.Get_contour_orange()
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

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

################################################################################            
if __name__== '__main__' :
    # global center_x, center_y
    try:
        rospy.init_node("object_detect", anonymous=True)
        rate = rospy.Rate(10) # 发布频率为10hz

        Findposition(1)

        rospy.loginfo("object_detect is started.. \n Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down object_detect node."
        cv2.destroyAllWindows()
