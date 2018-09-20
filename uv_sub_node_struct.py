#!/usr/bin/env python
import rospy
from ur5_planning.msg import uv
from ur5_planning.msg import structlight
import time
class UVStructRead():
    def __init__(self):
       # self.nodename=nodename
        self.left_uv0_buf = []
        self.left_uv1_buf = []
        self.right_uv0_buf = []
        self.right_uv1_buf = []
    def Init_node(self):
        rospy.init_node("desireuv_node")
        sub = rospy.Subscriber("/structlight_uv", structlight, self.callback)
        return sub
    # def Count_100(self,temp):

    def callback(self,msg):
        #left_uv0_buf
        if len(self.left_uv0_buf)==100:
            self.left_uv0_buf=self.left_uv0_buf[1:]
            self.left_uv0_buf.append(list(msg.left_uv0.uvinfo))
            #print "---------self.uvlist_buf",self.uvlist_buf

        else:
            self.left_uv0_buf.append(list(msg.left_uv0.uvinfo))
        #left_uv1_buf
        if len(self.left_uv1_buf)==100:
            self.left_uv1_buf=self.left_uv1_buf[1:]
            self.left_uv1_buf.append(list(msg.left_uv1.uvinfo))
            #print "---------self.uvlist_buf",self.uvlist_buf

        else:
            self.left_uv1_buf.append(list(msg.left_uv1.uvinfo))
        if len(self.right_uv0_buf)==100:
            self.right_uv0_buf=self.right_uv0_buf[1:]
            self.right_uv0_buf.append(list(msg.right_uv0.uvinfo))
            #print "---------self.uvlist_buf",self.uvlist_buf

        else:
            self.right_uv0_buf.append(list(msg.right_uv0.uvinfo))
        if len(self.right_uv1_buf)==100:
            self.right_uv1_buf=self.right_uv1_buf[1:]
            self.right_uv1_buf.append(list(msg.right_uv1.uvinfo))
            #print "---------self.uvlist_buf",self.uvlist_buf

        else:
            self.right_uv1_buf.append(list(msg.right_uv1.uvinfo))
def main():
    uv0=UVStructRead()
    uv0.Init_node()
    while not rospy.is_shutdown():
        if len(uv0.left_uv0_buf)==0 and len(uv0.left_uv1_buf)==0 and len(uv0.right_uv0_buf) and len(uv0.right_uv1_buf):
            print "wait data----\n"
            continue
        else:
            time.sleep(1)
            print "uv0.left_uv0_buf[-1]\n",uv0.left_uv0_buf[-1]
            print "uv0.left_uv1_buf[-1]\n", uv0.left_uv1_buf[-1]
            print "uv0.right_uv0_buf[-1]\n", uv0.right_uv0_buf[-1]
            print "uv0.right_uv1_buf[-1]\n", uv0.right_uv1_buf[-1]

        #uv0.uvlist=[]
    #rospy.spin()
if __name__=="__main__":
    main()