#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy,math
import Quaternion as Q
import time
from numpy import linalg
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

import yaml,os
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ur5_pose_get import *
from std_msgs.msg import Float64

from ur5_planning.msg import uv
from ur5_planning.msg import structlight
from uv_sub_node_struct import *
class StruLight:
    def __init__(self,u0,v0,pu,pv,f,alpha,x4_laser_ref,y4_laser_ref,z4_laser_ref,T1,urdfname,calibinfo,lamda=1.5):
        self.u0=u0
        self.v0=v0
        self.f=f
        self.pu=pu
        self.pv=pv
        self.alpha=alpha
        self.x4_laser_ref=x4_laser_ref
        self.y4_laser_ref=y4_laser_ref
        self.z4_laser_ref=z4_laser_ref
        self.urdfname=urdfname
        self.lamda=lamda

        self.T1=T1
        self.calibinfo=calibinfo
        rospy.init_node("structure_light_vision_control")
        self.delta_theta_pub_plane1 = rospy.Publisher("/delta_theta_pub_plane1_uv", Float64, queue_size=10)
        self.delta_puv_pub_plane1 = rospy.Publisher("/delta_puv_pub_plane1_uv", Float64, queue_size=10)
        self.delta_theta_pub_plane2 = rospy.Publisher("/delta_theta_pub_plane2_uv", Float64, queue_size=10)
    def GET_T_A(self,T):
        A11 = T[0]
        A12 = T[1]
        A14 = T[3]
        A21 = T[4]
        A22 = T[5]
        A24 = T[7]
        A31 = T[8]
        A32 = T[9]
        A34 = T[11]
        return A11,A12,A14,A21,A22,A24,A31,A32,A34
    """
    convert 10T to T01
    """
    def Get_Inv_10T_T01(self,T):
        Ttemp = numpy.matrix(T).reshape(4,4)
        # print "Ttemp",Ttemp
        R_initial=self.tr2r(Ttemp)
        # print "R_initial",R_initial
        Trans_initial=self.transl(Ttemp)
        # print "Trans_initial", Trans_initial
        New_trans=numpy.dot(-1*(R_initial.I),Trans_initial)
        Zero1=[0,0,0,1]
        R_new_Trans=numpy.column_stack((R_initial.I,New_trans))
        T_new=numpy.row_stack((R_new_Trans,numpy.matrix(Zero1)))
        # print "T_new ",T_new
        return T_new
    def GET_Inv_T(self,T):
        Ttemp=numpy.matrix(T)
        Ttemp=self.Get_Inv_10T_T01(T).tolist()
        # print "Ttemp",Ttemp
        invA11 =  Ttemp[0][0]
        invA12 =  Ttemp[0][1]
        invA13 =  Ttemp[0][2]
        invA14 =  Ttemp[0][3]
        invA21 =  Ttemp[1][0]
        invA22 =  Ttemp[1][1]
        invA23 =  Ttemp[1][2]
        invA24 =  Ttemp[1][3]
        invA31 =  Ttemp[2][0]
        invA32 =  Ttemp[2][1]
        invA33 =  Ttemp[2][2]
        invA34 =  Ttemp[2][3]
        return invA11,invA12,invA13,invA14,invA21,invA22,invA23,invA24,invA31,invA32,invA33,invA34

    def Get_X_from_cali_quaternion(self,calibinfo):
        transition_L = numpy.array(calibinfo[:3]).T
        # print transition_L
        rot = calibinfo[3:6]
        s = calibinfo[6]
        q0 = Q.quaternion(s, numpy.mat(rot))
        # print "q02R--------\n", q0.r()
        # print q0.r()
        T_part1 = numpy.column_stack((q0.r(), transition_L))
        # print T_part1
        T_part2 = numpy.array([0, 0, 0, 1])
        # print T_part2
        T = numpy.row_stack((T_part1, T_part2))
        # print T
        T = T.tolist()
        T = T[0] + T[1] + T[2] + T[3]
        # print("T:" , T)
        return T

    def Get_ur_X(self,info):
        # ar_info = [
        #     0.0213643977774,
        #     0.0657990946654,
        #     -0.0296300940415,
        #     0.667109190919,
        #     -0.334514375439,
        #     -0.0881824882439,
        #     0.659764585882
        #
        # ]
        aa = self.Get_X_from_cali_quaternion(info)
        aa = numpy.mat(aa)
        # print "X", aa
        return aa.reshape((4, 4))
    def seq(self,start, stop, step=1):
        n = int(round((stop - start) / float(step)))
        if n > 1:
            return ([start + step * i for i in range(n + 1)])
        elif n == 1:
            return ([start])
        else:
            return ([])

    def get_jacabian_from_joint(self,urdfname,jointq):
        #robot = URDF.from_xml_file("/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
        robot = URDF.from_xml_file(urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        # print tree.getNrOfSegments()
        chain = tree.getChain("base_link", "ee_link")
        # print chain.getNrOfJoints()
        # forwawrd kinematics
        kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        q=jointq
        #q = [0, 0, 1, 0, 1, 0]
        pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        # # print pose
        # #print list(pose)
        # q0=Kinematic(q)
        # if flag==1:
        #     q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward())
        # else:
        #     q_ik = kdl_kin.inverse(pose)  # inverse kinematics
        # # print "----------iverse-------------------\n", q_ik
        #
        # if q_ik is not None:
        #     pose_sol = kdl_kin.forward(q_ik)  # should equal pose
        #     print "------------------forward ------------------\n",pose_sol

        J = kdl_kin.jacobian(q)
        #print 'J:', J
        return J,pose
    def Get_p_theta(self,n_uv,u1_desire,v1_desire,u2_desire,v2_desire,u1,v1,u2,v2):
        a_uv_desire = (n_uv * (u1_desire * v1_desire + u2_desire * v2_desire) - (u1_desire + u2_desire) * (v1_desire + v2_desire))/ (n_uv * (u1_desire ** 2 + u2_desire ** 2) - (u1_desire + u2_desire) ** 2)
        # print "a_uv_desire",a_uv_desire
        b_uv_desire = (v1_desire + v2_desire) / n_uv - a_uv_desire * (u1_desire + u2_desire) / n_uv
        # print "b_uv_desire",b_uv_desire
        a_uv = (n_uv * (u1 * v1 + u2 * v2) - (u1 + u2) * (v1 + v2)) / (n_uv * (u1 ** 2 + u2 ** 2) - (u1 + u2) ** 2)
        # print "a_uv",a_uv
        b_uv = (v1 + v2) / n_uv - a_uv * (u1 + u2) / n_uv
        # print "b_uv",b_uv
        puv = b_uv / math.sqrt(a_uv ** 2 + 1)
        theta_uv = math.asin(-a_uv / math.sqrt(a_uv ** 2 + 1))

        puv_desire = b_uv_desire / math.sqrt(a_uv_desire ** 2 + 1)
        theta_uv_desire = math.asin(-a_uv_desire / math.sqrt(a_uv_desire ** 2 + 1))
        # print "a_uv_desire",a_uv_desire
        return puv,theta_uv,puv_desire,theta_uv_desire,a_uv_desire,b_uv_desire

    def Get_p3_theta3(self,a_uv_desire,b_uv_desire,A11,A12,A14,A21,A22,A24,A31,A32,A34):
        puv_desire = b_uv_desire / math.sqrt(a_uv_desire ** 2 + 1)
        theta_uv_desire = math.asin(-a_uv_desire / (a_uv_desire ** 2 + 1))

        s1_desire = (puv_desire - self.u0 * math.sin(theta_uv_desire) - self.v0 * math.cos(theta_uv_desire)) * self.pu * self.pv / (
                    self.f * ((self.pv * math.sin(theta_uv_desire)) ** 2 + (self.pu * math.cos(theta_uv_desire)) ** 2) ** 0.5)
        s2_desire = math.sin(theta_uv_desire) / (math.sin(theta_uv_desire) ** 2 + (self.pu / self.pv * math.cos(theta_uv_desire)) ** 2) ** 0.5
        s3_desire = math.cos(theta_uv_desire) / (math.cos(theta_uv_desire) ** 2 + (self.pv / self.pu * math.sin(theta_uv_desire)) ** 2) ** 0.5

        s3_desire_new = abs(s3_desire)
        abs_judge = s3_desire_new / s3_desire
        s1_desire = s1_desire / abs_judge
        s2_desire = s2_desire / abs_judge
        p2_desire = s1_desire
        theta2_desire = math.asin(s2_desire)

        m1_desire = p2_desire * A34 - math.sin(theta2_desire) * A14 - math.cos(theta2_desire) * A24
        m2_desire = A11 * math.sin(theta2_desire) + A21 * math.cos(theta2_desire) - A31 * p2_desire
        m3_desire = A12 * math.sin(theta2_desire) + A22 * math.cos(theta2_desire) - A32 * p2_desire

        m2_desire_new = abs(m2_desire)
        m2_desire = abs(m2_desire)
        abs_judge = m2_desire_new / m2_desire
        m1_desire = m1_desire / abs_judge
        m3_desire = m3_desire / abs_judge
        ##
        p3_desire = m1_desire / math.sqrt(m2_desire * m2_desire + m3_desire * m3_desire)
        theta3_desire = math.asin(m3_desire / math.sqrt(m2_desire * m2_desire + m3_desire * m3_desire))
        return p3_desire,theta3_desire

    def delta_puv(self,theta_uv,theta_uv_desire,puv,puv_desire):
        delta_theta = theta_uv - theta_uv_desire
        delta_puv = puv - puv_desire
        return [delta_theta,delta_puv]
    """
    T,plane to camera frame
    """

    def tr2jac(self,T, samebody):
        # T = np.array(T)
        R = self.tr2r(T)
        # jac = np.zeros((6, 6))
        """
        jac = [ jac_part1,  jac_part2;
                jac_part3,  jac_part4;
                    ]
        """
        if samebody == 1:
            jac_part1 = R
            New_trans = numpy.dot(-1 * (R.I), self.transl(T))
            jac_part2 = -numpy.dot(R, self.skew(New_trans))
            # print "self.transl(T))",self.transl(T)
            # T1=[1,2,3]
            # print "self.skew(self.transl(T))\n",self.skew(New_trans)
            jac_part3 = numpy.zeros((3, 3))
            jac_part4 = R

        else:
            jac_part1 = R
            jac_part2 = numpy.zeros((3, 3))
            jac_part3 = numpy.zeros((3, 3))
            jac_part4 = R
        jac_row1 = numpy.column_stack((jac_part1, jac_part2))
        jac_row2 = numpy.column_stack((jac_part3, jac_part4))
        jac = numpy.row_stack((jac_row1, jac_row2))
        return jac

    """
    if l is 3*1 , then get 
    skew(l) = [ 0, -l(2), l(1)
                l(2), 0 , -l(0)
                -l(1), l(0), 0]
    if l is 1*1, then get
    skew(l) = [ 0 , -l[0]
                l[0], 0 ]

    """

    def skew(self,l):
        a, b = numpy.shape(l)
        try:
            if a == 3:
                s = numpy.array([0, -l[2], l[1], l[2], 0, -l[0], -l[1], l[0], 0])
                s = s.reshape((3, 3))
                # print "s:", s
                return s
            elif a == 1:
                s = numpy.array([0, -l[0], l[0], 0])
                s = s.reshape((2, 2))
                return s
        except:
            print("erro l size!!!  3*1 or 1*1 required!")

    def tr2r(self,T):
        r = [0, 1, 2]
        c = [0, 1, 2]
        R1 = T[r]
        R = R1[:, c]
        return R

    def transl(self,T):
        r = [3]
        c = [0, 1, 2]
        l1 = T[:, r]
        l = l1[c]
        return l
    def Get_vxvyvzwxwywz(self,puv_plane1, theta_uv_plane1, puv_desire_plane1, theta_uv_desire_plane1,a_uv_desire_plane1,b_uv_desire_plane1,puv_plane2, theta_uv_plane2, puv_desire_plane2, theta_uv_desire_plane2, a_uv_desire_plane2, b_uv_desire_plane2,T_reflection_to_camera,T_camera_to_endeffector):
        T1=T_reflection_to_camera
        T_1_to_2=[0,  1 , 0  ,0, 0  ,0 , 1  ,0 ,1  ,0  ,0  ,0 ,0  ,0  ,0 , 1]
        T2=numpy.dot(numpy.matrix(T_1_to_2).reshape(4,4),numpy.matrix(T1).reshape(4,4))

        A11, A12, A14, A21, A22, A24, A31, A32, A34=self.GET_T_A(T1)
        invA11, invA12, invA13, invA14, invA21, invA22, invA23, invA24, invA31, invA32, invA33, invA34=self.GET_Inv_T(T1)
        T2=self.From_matrix_to_list(T2)
        print "T2",T2
        B11, B12, B14, B21, B22, B24, B31, B32, B34=self.GET_T_A(T2)
        invB11, invB12, invB13, invB14, invB21, invB22, invB23, invB24, invB31, invB32, invB33, invB34=self.GET_Inv_T(T2)


        delta_theta_plane1, delta_puv_plane1=self.delta_puv(theta_uv_plane1,theta_uv_desire_plane1,puv_plane1,puv_desire_plane1)
        delta_theta_plane2, delta_puv_plane2 = self.delta_puv(theta_uv_plane2, theta_uv_desire_plane2, puv_plane2, puv_desire_plane2)

        # p3_desire, theta3_desire=self.Get_p3_theta3(a_uv_desire,b_uv_desire,A11, A12, A14, A21, A22, A24, A31, A32, A34)

        line_p3=[]
        line_theta3=[]
        vx1 = 0.0
        vy1 = 0.0
        vz1 = 0.0
        wx1 = 0.0
        wy1 = 0.0
        wz1 = 0.0
        lenda=0.0


        s1_plane1 = (puv_plane1 - self.u0 * math.sin(theta_uv_plane1) - self.v0 * math.cos(theta_uv_plane1)) * self.pu * self.pv / (self.f * ((self.pv * math.sin(theta_uv_plane1)) ** 2 + (self.pu * math.cos(theta_uv_plane1)) **2) ** 0.5)
        #define line_jacob1
        s2_plane1 = math.sin(theta_uv_plane1) / (math.sin(theta_uv_plane1) ** 2 + (self.pu / self.pv * math.cos(theta_uv_plane1)) ** 2) ** 0.5
        s3_plane1 = math.cos(theta_uv_plane1) / (math.cos(theta_uv_plane1) ** 2 + (self.pv / self.pu * math.sin(theta_uv_plane1)) ** 2) ** 0.5
        t11_plane1 = s2_plane1 / math.sin(theta_uv_plane1) - s2_plane1 * math.sin(theta_uv_plane1) * (1 - (self.pu / self.pv) ** 2)
        t12_plane1 = 0
        t21_plane1 = self.pu * self.pv * (puv_plane1 - self.u0 * math.cos(theta_uv_plane1) + self.v0 * math.sin(theta_uv_plane1)) / (self.f * ((self.pv * math.sin(theta_uv_plane1)) ** 2 + (self.pu * math.cos(theta_uv_plane1)) ** 2) ** 0.5) \
              + self.pu * self.pv * (puv_plane1 - self.u0 * math.sin(theta_uv_plane1) - self.v0 * math.cos(theta_uv_plane1)) * (self.pu - self.pv) * math.sin(theta_uv_plane1) * math.cos(theta_uv_plane1) \
              / (self.f * ((self.pv * math.sin(theta_uv_plane1)) ** 2 + (self.pu * math.cos(theta_uv_plane1)) ** 2) **1.5)
        t22_plane1 = self.pu * self.pv / (self.f * ((self.pv * math.sin(theta_uv_plane1)) **2 + (self.pu * math.cos(theta_uv_plane1)) ** 2) ** 0.5)


        s3_new_plane1 = abs(s3_plane1)
        abs_judge = s3_new_plane1 / s3_plane1
        s1_plane1 = s1_plane1 / abs_judge
        s2_plane1 = s2_plane1 / abs_judge
        p2_plane1 = s1_plane1
        theta2_plane1 = math.asin(s2_plane1)

        # p2 = s1; % sin(theta2) = s2; % cos(theta2) = s3;

        # define line_jacob2
        m1_plane1 = p2_plane1 * A34 - math.sin(theta2_plane1) * A14 - math.cos(theta2_plane1) * A24
        m2_plane1 = A11 * math.sin(theta2_plane1) + A21 * math.cos(theta2_plane1) - A31 * p2_plane1
        m3_plane1 = A12 * math.sin(theta2_plane1) + A22 * math.cos(theta2_plane1) - A32 * p2_plane1


        n11_plane1 = (m2_plane1 / (m3_plane1 * (m2_plane1 ** 2 + m3_plane1 ** 2)) - 1 / m3_plane1) * (A11 * math.cos(theta2_plane1) - A21 * math.sin(theta2_plane1)) + m2_plane1 / (m3_plane1 * (m2_plane1 ** 2 + m3_plane1 ** 2)) * (A12 * math.cos(theta2_plane1) - A22 * math.sin(theta2_plane1))
        n12_plane1 = (m2_plane1 / (m3_plane1 * (m2_plane1 ** 2 + m3_plane1 ** 2)) - 1 / m3_plane1) * (-A31) + m2_plane1 / (m3_plane1 * (m2_plane1 ** 2 + m3_plane1 ** 2)) * (-A32)
        n21_plane1 = (m2_plane1 **2 + m3_plane1 ** 2) ** (-0.5) * (-(A14 * math.cos(theta2_plane1) - A24 * math.sin(theta2_plane1))) - m1_plane1 * ((m2_plane1 ** 2 + m3_plane1 ** 2) ** (-1.5)) * (A11 * math.cos(theta2_plane1) - A21 * math.sin(theta2_plane1)) - m1_plane1 * ((m2_plane1 **2 + m3_plane1 ** 2) **(-1.5)) * (A11 * math.cos(theta2_plane1) - A21 * math.sin(theta2_plane1))
        n22_plane1 = (m2_plane1 ** 2 + m3_plane1 ** 2) ** (-0.5) * A34 - m1_plane1 * ((m2_plane1 ** 2 + m3_plane1 ** 2) ** (-1.5)) * (-A31) + m2_plane1 / (m3_plane1 * (m2_plane1 ** 2 + m3_plane1 ** 2)) * (-A32)




        m2_new_plane1 = m2_plane1
        m2_plane1 = abs(m2_plane1)
        abs_judge = m2_new_plane1 / m2_plane1
        m1_plane1 = m1_plane1 / abs_judge
        m3_plane1 = m3_plane1 / abs_judge
        p3 = m1_plane1 / math.sqrt(m2_plane1 * m2_plane1 + m3_plane1 * m3_plane1)
        theta3_plane1 = math.asin(m3_plane1 / math.sqrt(m2_plane1 * m2_plane1 + m3_plane1 * m3_plane1))




        #define line_jacob1_plane2
        s1_plane2 = (puv_plane2 - self.u0 * math.sin(theta_uv_plane2) - self.v0 * math.cos(theta_uv_plane2)) * self.pu * self.pv / (self.f * ((self.pv * math.sin(theta_uv_plane2)) ** 2 + (self.pu * math.cos(theta_uv_plane2)) **2) ** 0.5)
        #define line_jacob1
        s2_plane2 = math.sin(theta_uv_plane2) / (math.sin(theta_uv_plane2) ** 2 + (self.pu / self.pv * math.cos(theta_uv_plane2)) ** 2) ** 0.5
        s3_plane2 = math.cos(theta_uv_plane2) / (math.cos(theta_uv_plane2) ** 2 + (self.pv / self.pu * math.sin(theta_uv_plane2)) ** 2) ** 0.5
        t11_plane2 = s2_plane2 / math.sin(theta_uv_plane2) - s2_plane2 * math.sin(theta_uv_plane2) * (1 - (self.pu / self.pv) ** 2)
        t12_plane2 = 0
        t21_plane2 = self.pu * self.pv * (puv_plane2 - self.u0 * math.cos(theta_uv_plane2) + self.v0 * math.sin(theta_uv_plane2)) / (self.f * ((self.pv * math.sin(theta_uv_plane2)) ** 2 + (self.pu * math.cos(theta_uv_plane2)) ** 2) ** 0.5) \
              + self.pu * self.pv * (puv_plane2 - self.u0 * math.sin(theta_uv_plane2) - self.v0 * math.cos(theta_uv_plane2)) * (self.pu - self.pv) * math.sin(theta_uv_plane2) * math.cos(theta_uv_plane2) \
              / (self.f * ((self.pv * math.sin(theta_uv_plane2)) ** 2 + (self.pu * math.cos(theta_uv_plane2)) ** 2) **1.5)
        t22_plane2 = self.pu * self.pv / (self.f * ((self.pv * math.sin(theta_uv_plane2)) **2 + (self.pu * math.cos(theta_uv_plane2)) ** 2) ** 0.5)


        s3_new_plane2 = abs(s3_plane2)
        abs_judge = s3_new_plane2 / s3_plane2
        s1_plane2 = s1_plane2 / abs_judge
        s2_plane2 = s2_plane2 / abs_judge
        p2_plane2 = s1_plane2
        theta2_plane2 = math.asin(s2_plane2)

        # p2 = s1; % sin(theta2) = s2; % cos(theta2) = s3;

        # define line_jacob2
        m1_plane2 = p2_plane2 * A34 - math.sin(theta2_plane2) * A14 - math.cos(theta2_plane2) * A24
        m2_plane2 = A11 * math.sin(theta2_plane2) + A21 * math.cos(theta2_plane2) - A31 * p2_plane2
        m3_plane2 = A12 * math.sin(theta2_plane2) + A22 * math.cos(theta2_plane2) - A32 * p2_plane2


        n11_plane2 = (m2_plane2 / (m3_plane2 * (m2_plane2 ** 2 + m3_plane2 ** 2)) - 1 / m3_plane2) * (A11 * math.cos(theta2_plane2) - A21 * math.sin(theta2_plane2)) + m2_plane2 / (m3_plane2 * (m2_plane2 ** 2 + m3_plane2 ** 2)) * (A12 * math.cos(theta2_plane2) - A22 * math.sin(theta2_plane2))
        n12_plane2 = (m2_plane2 / (m3_plane2 * (m2_plane2 ** 2 + m3_plane2 ** 2)) - 1 / m3_plane2) * (-A31) + m2_plane2 / (m3_plane2 * (m2_plane2 ** 2 + m3_plane2 ** 2)) * (-A32)
        n21_plane2 = (m2_plane2 **2 + m3_plane2 ** 2) ** (-0.5) * (-(A14 * math.cos(theta2_plane2) - A24 * math.sin(theta2_plane2))) - m1_plane2 * ((m2_plane2 ** 2 + m3_plane2 ** 2) ** (-1.5)) * (A11 * math.cos(theta2_plane2) - A21 * math.sin(theta2_plane2)) - m1_plane2 * ((m2_plane2 **2 + m3_plane2 ** 2) **(-1.5)) * (A11 * math.cos(theta2_plane2) - A21 * math.sin(theta2_plane2))
        n22_plane2 = (m2_plane2 ** 2 + m3_plane2 ** 2) ** (-0.5) * A34 - m1_plane2 * ((m2_plane2 ** 2 + m3_plane2 ** 2) ** (-1.5)) * (-A31) + m2_plane2 / (m3_plane2 * (m2_plane2 ** 2 + m3_plane2 ** 2)) * (-A32)




        m2_new_plane2 = m2_plane2
        m2_plane2 = abs(m2_plane2)
        abs_judge = m2_new_plane2 / m2_plane2
        m1_plane2 = m1_plane2 / abs_judge
        m3_plane2 = m3_plane2 / abs_judge
        p3 = m1_plane2 / math.sqrt(m2_plane2 * m2_plane2 + m3_plane2 * m3_plane2)
        theta3_plane2 = math.asin(m3_plane2 / math.sqrt(m2_plane2 * m2_plane2 + m3_plane2 * m3_plane2))

        #% compute all line jacob matrix
        line_jacob1_plane1 = [t11_plane1, t12_plane1, t21_plane1, t22_plane1]
        line_jacob2_plane1 = [n11_plane1,n12_plane1,n21_plane1,n22_plane1]
        line_jacob1_plane2 = [t11_plane2, t12_plane2, t21_plane2, t22_plane2]
        line_jacob2_plane2 = [n11_plane2, n12_plane2, n21_plane2, n22_plane2]
        line_jacob_plane1 = numpy.dot(numpy.matrix(line_jacob2_plane1).reshape(2,2) , numpy.matrix(line_jacob1_plane1).reshape(2,2))
        line_jacob_plane2 = numpy.dot(numpy.matrix(line_jacob2_plane2).reshape(2,2) , numpy.matrix(line_jacob1_plane2).reshape(2,2))

        line_jacob_plane1=self.From_matrix_to_list(line_jacob_plane1)
        print "line_jacob_plane1",line_jacob_plane1
        print "line_jacob_plane2[2][2]",line_jacob_plane2.tolist()[0][0]
        line_jacob21=[line_jacob_plane1[0],line_jacob_plane1[1],0,line_jacob_plane1[2],line_jacob_plane1[3],0,0,0,line_jacob_plane2.tolist()[0][0]]
        print "line_jacob21",line_jacob21
        line_jacob3=[ 0,0,0,0,0,-1,-1,0,0,0,0,0, 0 ,   0  , 0  , -1   ,0  ,    0]#3X6


        # line_jacob3 = [0,0,0,- lenda * (math.cos(theta3)), - lenda * math.sin(theta3),- 1,-math.cos(theta3) ,- math.sin(theta3),lenda,lenda * math.sin(theta3) * p3,- lenda * math.cos(theta3) * p3,0]

        # compute overall jacobian matrix and its inverse jacobian matrix
        # total_jacob = inv(line_jacob2 * line_jacob1) * line_jacob3;

        AA = numpy.matrix(line_jacob3).reshape(3,6)
        inv_line_jacob3 = numpy.linalg.pinv(AA)

        inv_total_jacob = numpy.dot(inv_line_jacob3,numpy.matrix(line_jacob21).reshape(3,3))
        # inv_total_jacob = numpy.dot(numpy.dot(inv_line_jacob3, numpy.matrix(line_jacob2).reshape(2,2)),numpy.matrix(line_jacob1).reshape(2,2))

        # start computing velocity of reflection frame 1 2
        delta_theta_plane1 = theta_uv_plane1 - theta_uv_desire_plane1
        delta_puv_plane1 = puv_plane1 - puv_desire_plane1
        delta_theta_plane2 = theta_uv_plane2 - theta_uv_desire_plane2


        # delta_theta3 = (theta3 - theta3_desire)#*180/math.pi
        # delta_p3 = (p3 - p3_desire)#*180/math.pi
        self.delta_theta_pub_plane1.publish(delta_theta_plane1)
        self.delta_puv_pub_plane1.publish(delta_puv_plane1)
        self.delta_theta_pub_plane2.publish(delta_theta_plane2)
        print "delta_theta\n",delta_theta_plane1
        print "delta_puv\n",delta_puv_plane1
        print "delta_theta_plane2",delta_theta_plane2

        # print "self.lamda * inv_total_jacob",self.lamda * inv_total_jacob
        # print "numpy.matrix([delta_theta,delta_puv])",numpy.matrix([delta_theta,delta_puv])
        vel_refelection = numpy.dot(-1*self.lamda * inv_total_jacob,numpy.matrix([delta_theta_plane1,delta_puv_plane1 ,delta_theta_plane2]).T)
        #vel_refelection=-lamda*inv_total_jacob*[delta_theta_plane1 delta_puv_plane1  delta_theta_plane2]';
        # print "vel_refelection",vel_refelection
        vx1 = vel_refelection[0]*math.cos(theta3_plane1)
        vy1 = vel_refelection[0]*math.sin(theta3_plane1)
        vz1 = vel_refelection[2]
        wx1 = vel_refelection[3]
        wy1 = vel_refelection[4]
        wz1 = vel_refelection[5]

        vel_refelection_new=[vx1.tolist()[0][0],vy1.tolist()[0][0],vz1.tolist()[0][0],wx1.tolist()[0][0],wy1.tolist()[0][0],wz1.tolist()[0][0]]
        # print "vel_refelection end---", numpy.matrix(vel_refelection_new)
        # print "vx1,vy1,vz1,wx1,wy1,wz1",vx1,vy1,vz1,wx1,wy1,wz1
        #publish velocity of reflection plate [vx vy vz wx wy wz]

        print "vel_refelection_new\n",vel_refelection_new
        T_reflection_to_endeffector = numpy.dot(numpy.matrix(T_camera_to_endeffector).reshape(4,4),numpy.matrix(T_reflection_to_camera).reshape(4,4) )
        print "T_reflection_to_endeffector\n",T_reflection_to_endeffector

        # print "numpy.matrix(T_camera_to_endeffector).reshape(4,4)",numpy.matrix(T_camera_to_endeffector).reshape(4,4)
        # print "numpy.matrix(T_reflection_to_camera).reshape(4,4)",numpy.matrix(T_reflection_to_camera).reshape(4,4)
        # print "T_reflection_to_endeffector\n",T_reflection_to_endeffector

        Jacob_reflection_to_endeffector=self.tr2jac(T_reflection_to_endeffector,1)
        print "Jacob_reflection_to_endeffector\n",Jacob_reflection_to_endeffector

        # print "Jacob_reflection_to_endeffector\n",Jacob_reflection_to_endeffector
        v_reflection_to_ee=numpy.dot(Jacob_reflection_to_endeffector,numpy.matrix(vel_refelection_new).T)
        # time.sleep(1)
        print "v_reflection_to_ee\n",v_reflection_to_ee
        # return vx1,vy1,vz1,wx1,wy1,wz1
        return v_reflection_to_ee

    def get_joint_speed(self,q,ee_speed):
        #1,get base to ee jacabian
        Jacabian_joint=self.get_jacabian_from_joint(self.urdfname,q)
        #2,get ee(AX=XB) to camera frame jacabian
        X=self.Get_ur_X(self.calibinfo)#numpu array
        # print "X",X
        # print "Jacabian_joint",Jacabian_joint[0]
        #tr2jac
        jac = self.tr2jac(X,1)
        #print "------X",X
        inv_X_jac = jac.I
        j_speed=numpy.dot(Jacabian_joint[0].I,ee_speed)

        return j_speed
    #
    def get_deta_joint_angular(self,detat,j_speed):
        #print j_speed
        joint_angular=float(detat)*numpy.array(j_speed)
        #print '-------joint_angular-----\n',joint_angular
        return joint_angular

    def get_joint_angular(self,qnow,detajoint):
        #result=[]
        listangular=[]
        for i in range(len(detajoint.tolist())):
            listangular.append(detajoint.tolist()[i][0]+qnow[i])
        # print "list",detajoint.tolist()
        return listangular
    def From_matrix_to_list(self,data):
        temp=data.tolist()
        result=[]
        for i in xrange(len(temp)):
            for ii in xrange(len(temp)):
                result.append(temp[i][ii])
        return result

def main():
    u0 = 316.404078
    v0 = 251.341039
    kx = 627.260603
    ky = 622.895967
    alpha = 0.0
    f = 624.0429 * 1e-03
    pu = 9.9487e-04
    pv = 0.0010
    x4_laser_ref=1.0
    y4_laser_ref=2.0
    z4_laser_ref=3.0

    n_uv = 2.0
    ##left line----line0
    kk = (273, 259)
    kkk = (438, 168)
    u1_desire = kk[0]
    v1_desire = kk[1]
    u2_desire = kkk[0]
    v2_desire = kkk[1]
    #real time uv(u,v)
    #start uv
    u1 = 420.0
    v1 = 331.0
    u2 = 519.0
    v2 = 321.0
    ###right line -----lin1
    #(455, 170), (638, 216)
    line2_point0 = (455, 170)
    line2_point1 = (638, 216)
    u3_desire = line2_point0[0]
    v3_desire = line2_point0[1]
    u4_desire = line2_point1[0]
    v4_desire = line2_point1[1]
    u3 = 200
    v3 = 200
    u4 = 300
    v4 = 100
    ######
    calibinfo=[
        0.0213643977774,
        0.0657990946654,
        -0.0296300940415,
        0.667109190919,
        -0.334514375439,
        -0.0881824882439,
        0.659764585882
    ]
    reflect_camera_info=[
    # 0.134482874649,
    # 0.00201681390159,
    # 0.571575563438,
    # 0.0216075446025,
    # 0.947178734604,
    # 0.022100161443,
    # 0.319213316757
    0.0433484481415,
    0.00439504237274,
    0.232268496875,
    0.0200024965134,
    0.931985559523,
    0.0344022163001,
    0.360304460821
    ]
    ace=50
    vel=0.1
    urt=0
    detat=0.05
    ratet=3
    urdfname = "/data/ros/ur_ws/src/universal_robot/ur5_planning/urdf/ur5.urdf"
    # T1 = [0.5685,- 0.7033,- 0.4269,0.0974,-0.5357,- 0.7103,0.4567,0.1034,-0.6244,- 0.0310,- 0.7805,0.5306,0,0,0,1.0000]
    T1 = [-0.7952719448436829, 0.02682308183882049, 0.6056591913174864, 0.134482874649, 0.05504174517916128, 0.9980893937608886, 0.028070773941172033, 0.00201681390159, -0.6037490704210781, 0.05566043785932738, -0.7952288825395675, 0.571575563438, 0.0, 0.0, 0.0, 1.0]
    s0 = StruLight(u0, v0, pu, pv, f, alpha, x4_laser_ref, y4_laser_ref, z4_laser_ref, T1, urdfname,calibinfo)
    T1=s0.Get_ur_X(reflect_camera_info)

    # print "T_reflection_to_camera\n",T1
    T1=s0.From_matrix_to_list(T1)
    print "T1\n",T1

    # print "T_camera_to_endeffector",s0.Get_ur_X(calibinfo)
    # #marker to camera frame
    T_reflection_to_camera=T1
    #[-0.57119722,0.69567303,-0.43562352,0.09475704,0.5922862,0.71676598,0.3680254,0.08832676,0.56826568,-0.04779821,-0.82145222,0.48303625,0. ,0.,0. ,1.  ]
    # #camera to ee frame

    T_camera_to_endeffector=s0.From_matrix_to_list(s0.Get_ur_X(calibinfo))
    print "T_camera_to_endeffector\n",T_camera_to_endeffector
    # T_camera_to_endeffector=[0.594968000000000,-0.587314000000000,0.548703000000000,-0.122910000000000,-0.00030400000000000,-0.682843000000000,-0.730565000000000,0.348829000000000,0.803749000000000,	0.434496000000000,	-0.406449000000000,	0.552213000000000,0,	0,	0,	1]
    # s0=StruLight(u0,v0,pu,pv,f,alpha,x4_laser_ref,y4_laser_ref,z4_laser_ref,T1,urdfname)


    #get joint q
    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)

    struct_uv_sub = UVStructRead()
    sub = rospy.Subscriber("/structlight_uv", structlight, struct_uv_sub.callback)

    #give q to ur3
    ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)



    rate = rospy.Rate(ratet)

    while not rospy.is_shutdown():
        try:
            if len(struct_uv_sub.left_uv0_buf)!=0 and len(struct_uv_sub.left_uv1_buf)!=0 and len(struct_uv_sub.right_uv0_buf)!=0 and len(struct_uv_sub.right_uv1_buf)!=0:
                #get real uv,left line
                u1 = struct_uv_sub.left_uv0_buf[-1][0]
                v1 = struct_uv_sub.left_uv0_buf[-1][1]
                u2 = struct_uv_sub.left_uv1_buf[-1][0]
                v2 = struct_uv_sub.left_uv1_buf[-1][1]
                u3 = struct_uv_sub.right_uv0_buf[-1][0]
                v3 = struct_uv_sub.right_uv0_buf[-1][1]
                u4 = struct_uv_sub.right_uv1_buf[-1][0]
                v4 = struct_uv_sub.right_uv1_buf[-1][1]
                print "real time uv u1,v1,u2,v2",u1,v1,u2,v2
                print "real time u3, v3, u4,v4", u3, v3, u4,v4
                puv_plane1, theta_uv_plane1, puv_desire_plane1, theta_uv_desire_plane1,a_uv_desire_plane1,b_uv_desire_plane1=s0.Get_p_theta(n_uv,u1_desire,v1_desire,u2_desire,v2_desire,u1,v1,u2,v2)
                puv_plane2, theta_uv_plane2, puv_desire_plane2, theta_uv_desire_plane2, a_uv_desire_plane2, b_uv_desire_plane2 = s0.Get_p_theta(n_uv, u3_desire,
                                                                                                      v3_desire, u4_desire,
                                                                                                      v4_desire, u3, v3, u4,v4)

                # print "puv,theta_uv,puv_desire,theta_uv_desire",puv,theta_uv,puv_desire,theta_uv_desire
                #1,get ur pose
                q_now = ur_reader.ave_ur_pose
                if len(q_now)!=0:
                    # q_now = ur_reader.ave_ur_pose
                    # print "q_now\n", q_now
                    #2,get ee_speed
                    ee_speed=s0.Get_vxvyvzwxwywz(puv_plane1, theta_uv_plane1, puv_desire_plane1, theta_uv_desire_plane1,a_uv_desire_plane1,b_uv_desire_plane1,puv_plane2, theta_uv_plane2, puv_desire_plane2, theta_uv_desire_plane2, a_uv_desire_plane2, b_uv_desire_plane2,T_reflection_to_camera,T_camera_to_endeffector)
                    #3,get  joint speed
                    j_speed=s0.get_joint_speed(q_now,ee_speed)
                    #4,get deta joint speed
                    deta_ee_speed=s0.get_deta_joint_angular(detat,j_speed)
                    #5,get pub_q_now
                    q_pub_now=s0.get_joint_angular(q_now,deta_ee_speed)
                    print "##############################################################"
                    print "move ur base the servo system----"
                    print "q_now\n", q_now
                    print "q_pub_now\n",q_pub_now
                    ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(
                        q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
                        vel) + "," + "t=" + str(urt) + ")"
                    print ss
                    print "###############################################################"
                    ur_pub.publish(ss)
                else:
                    print "wait subscribe joint states"
                    time.sleep(1)
        except:
            pass
        rate.sleep()
if __name__=="__main__":
    main()
