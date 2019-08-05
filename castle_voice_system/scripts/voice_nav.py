#!/usr/bin/python
# -*- coding: UTF-8 -*-

from xml.dom.minidom import parse
import xml.dom.minidom

import rospy

from std_msgs.msg import Int64, Int32

class XML_Analysis():
    def __init__(self):
        rospy.init_node('XML_Analysis', log_level=rospy.INFO)
        #在launch文件中获取参数
        self.file_path = rospy.get_param("~file_path", "/params/voiceNav.xml")
        self.sub = rospy.Subscriber('/voice/castle_xf_cmd_topic', Int32 , self.cmd_callback)
        self.pub = rospy.Publisher('/nav_position', Int64, queue_size = 1)
        self.DOMTree = None
        self.nlp = None
        self.version = None
        self.rawtext = None
        self.rawtext_data = None
        self.confidence = None
        self.confidence_data = None
        self.engine = None
        self.focus = None
        self.result_confidence = None
        self.result_confidence_data = None
        self.position = None
        self.position_id = None

        self.cmd_flag = False

        self.r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.cmd_flag is True:
                self.analysis_file()
                self.Process_Speech_cmd_to_Speed()
                self.cmd_flag = False
            self.r.sleep()

    # 使用minidom解析器打开 XML 文档
    def open_xml_file(self):
        self.DOMTree = xml.dom.minidom.parse(self.file_path)
        self.nlp = self.DOMTree.documentElement #获取nlp节点

    def analysis_file(self):
        self.open_xml_file()
        if self.nlp.hasChildNodes():
            self.version = self.nlp.getElementsByTagName('version')[0]
            #print "version: %s" % version.childNodes[0].data
            self.rawtext = self.nlp.getElementsByTagName('rawtext')[0]
            self.rawtext_data = self.rawtext.childNodes[0].data
            #print "rawtext: %s" % self.rawtext.childNodes[0].data
            self.confidence = self.nlp.getElementsByTagName('confidence')[0]
            self.confidence_data = self.confidence.childNodes[0].data
            #print "confidence: %s" % self.confidence.childNodes[0].data
            self.engine = self.nlp.getElementsByTagName('engine')[0]
            #print "engine: %s" % self.engine.childNodes[0].data

            self.result = self.nlp.getElementsByTagName("result")[0] #获取result节点

            self.focus = self.result.getElementsByTagName('focus')[0]
            self.focus_data = map(str,self.focus.childNodes[0].data.split('|'))
            '''
            try:
                print "focus: ", map(str,focus_data).index('start')
            except ValueError:
                print 'None'
            '''

            self.result_confidence = self.result.getElementsByTagName('confidence')[0]
            self.result_confidence_data = map(int,self.result_confidence.childNodes[0].data.split('|'))
            print "confidence:", self.result_confidence_data

            self.object = self.nlp.getElementsByTagName("object")[0] #获取object节点
            try:
                self.position = self.object.getElementsByTagName('position')[0]
                print "position: %s" % self.position.childNodes[0].data
                if self.position.hasAttribute("id"):
                    self.position_id = int(self.position.getAttribute("id"))
                    print "position id: %s" %  self.position.getAttribute("id")
            except IndexError:
                self.position = None
                self.position_id = None
                print 'No position'

            print "解析完成.....\n"

        else:
            print "No XML"

    def cmd_callback(self, req):
        print "命令词识别成功，正在解析命令词结果..."
        self.cmd_flag = True

    def Process_Speech_cmd_to_Speed(self):
        print "文本： ", self.rawtext_data #打印会话结果
        print "可信度： ", self.confidence_data #打印可信度
        if self.confidence_data>=15:
            print("pass!")
            self.goal_point_msg = Int64()
            self.goal_point_msg.data = self.position_id
            #rospy.loginfo(self.goal_point_msg)
            self.pub.publish(self.goal_point_msg)

if __name__ == '__main__':
    XML_Analysis()
        
