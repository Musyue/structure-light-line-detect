#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy,math
import time
from numpy import linalg
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from trans_methods import *
class StruLight:
    def __init__(self,u0,v0,pu,pv,f,alpha,x4_laser_ref,y4_laser_ref,z4_laser_ref,T1,urdfname,lamda=5,delta_t=0.01,n=5000,i=1):
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
        self.delta_t=delta_t
        self.n=n
        self.i=i
        self.T1=T1
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

    def GET_Inv_T(self,T):
        Ttemp=numpy.matrix(T)
        Ttemp=Ttemp.I
        invA11 =  Ttemp[0]
        invA12 =  Ttemp[1]
        invA13 =  Ttemp[2]
        invA14 =  Ttemp[3]
        invA21 =  Ttemp[4]
        invA22 =  Ttemp[5]
        invA23 =  Ttemp[6]
        invA24 =  Ttemp[7]
        invA31 =  Ttemp[8]
        invA32 =  Ttemp[9]
        invA33 =  Ttemp[10]
        invA34 =  Ttemp[11]
        return invA11,invA12,invA13,invA14,invA21,invA22,invA23,invA24,invA31,invA32,invA33,invA34

    def seq(self,start, stop, step=1):
        n = int(round((stop - start) / float(step)))
        if n > 1:
            return ([start + step * i for i in range(n + 1)])
        elif n == 1:
            return ([start])
        else:
            return ([])

    def get_jacabian_from_joint(self,urdfname,jointq,flag):
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
        print "a_uv_desire",a_uv_desire
        b_uv_desire = (v1_desire + v2_desire) / n_uv - a_uv_desire * (u1_desire + u2_desire) / n_uv
        print "b_uv_desire",b_uv_desire
        a_uv = (n_uv * (u1 * v1 + u2 * v2) - (u1 + u2) * (v1 + v2)) / (n_uv * (u1 ** 2 + u2 ** 2) - (u1 + u2) ** 2)
        print "a_uv",a_uv
        b_uv = (v1 + v2) / n_uv - a_uv * (u1 + u2) / n_uv
        print "b_uv",b_uv
        puv = b_uv / math.sqrt(a_uv ** 2 + 1)
        theta_uv = math.asin(-a_uv / math.sqrt(a_uv ** 2 + 1))
        puv_desire = b_uv_desire / math.sqrt(a_uv_desire ** 2 + 1)
        theta_uv_desire = math.asin(-a_uv_desire / (a_uv_desire ** 2 + 1))
        print "a_uv_desire",a_uv_desire
        return puv,theta_uv,puv_desire,theta_uv_desire



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
            jac_part1 = R.T
            jac_part2 = -numpy.dot(R.T, self.skew(self.transl(T)))
            # print "self.transl(T))",self.transl(T)
            # print "self.skew(self.transl(T))",self.skew(self.transl(T))
            jac_part3 = numpy.zeros((3, 3))
            jac_part4 = R.T

        else:
            jac_part1 = R.T
            jac_part2 = numpy.zeros((3, 3))
            jac_part3 = numpy.zeros((3, 3))
            jac_part4 = R.T
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
    def get_joint_speed(self,q,vee):
        #1,get base to ee jacabian
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q,0)
        #2,get ee(AX=XB) to camera frame jacabian
        X=get_ur_X()#numpu array
        ebT=T_06
        #tr2jac
        jac = tr2jac(X,1)
        jac_b2e=tr2jac(T_06,0)
        #print "------X",X
        inv_X_jac = jac.I
        #get ee speed
        #print "tr2jac-----\n",jac
        cam_speed = self.get_cam_vdot(uvm, z, desireuv, nowuv)
        print "cam_speed--------",cam_speed
        ee_speed_in_eeframe = np.dot(inv_X_jac, cam_speed)
        v_list = ee_speed_in_eeframe.reshape((1, 6)).tolist()[0]
        #[z,y,]
        flag_list = [0, 1, 1, 0, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        print "ee_speed-----before changing--------",ee_speed_in_base

        print("ee_speed_after--------------\n",vdot_z)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        return j_speed

    def get_deta_joint_angular(self,detat,uvm,z,desireuv,nowuv,q):
        j_speed=self.get_joint_speed(uvm,z,desireuv,nowuv,q)
        #print j_speed
        joint_angular=float(detat)*numpy.array(j_speed)
        #print '-------joint_angular-----\n',joint_angular
        return joint_angular
    def get_joint_angular(self,qnow,detajoint):
        #result=[]
        listangular=[]
        for i in range(len(detajoint.tolist())):
            listangular.append(detajoint.tolist()[i][0]+qnow[i])
        print "list",detajoint.tolist()
        return listangular
    #get
    def Get_vxvyvzwxwywz(self,theta_uv,theta_uv_desire,puv,puv_desire,T,T_reflection_to_camera,T_camera_to_endeffector):
        A11, A12, A14, A21, A22, A24, A31, A32, A34=self.GET_T_A(T)
        invA11, invA12, invA13, invA14, invA21, invA22, invA23, invA24, invA31, invA32, invA33, invA34=self.GET_Inv_T(T)
        delta_theta, delta_puv=self.delta_puv(theta_uv,theta_uv_desire,puv,puv_desire)
        line_p3=[]
        line_theta3=[]
        ii=1
        vx1 = 0.0
        vy1 = 0.0
        vz1 = 0.0
        wx1 = 0.0
        wy1 = 0.0
        wz1 = 0.0
        lenda=0.0
        for t in self.seq(0,self.delta_t*self.n,self.delta_t):
            if (abs(delta_theta) and abs(delta_puv) >= 0.01):
                s1 = (puv - self.u0 * math.sin(theta_uv) - self.v0 * math.cos(theta_uv)) * self.pu * self.pv / (self.f * ((self.pv * math.sin(theta_uv)) ** 2 + (self.pu * math.cos(theta_uv)) **2) ** 0.5)
                #define line_jacob1
                s2 = math.sin(theta_uv) / (math.sin(theta_uv) ** 2 + (self.pu / self.pv * math.cos(theta_uv)) ** 2) ** 0.5
                s3 = math.cos(theta_uv) / (math.cos(theta_uv) ** 2 + (self.pv / self.pu * math.sin(theta_uv)) ** 2) ** 0.5
                t11 = s2 / math.sin(theta_uv) - s2 * math.sin(theta_uv) * (1 - (self.pu / self.pv) ** 2)
                t12 = 0
                t21 = self.pu * self.pv * (puv - self.u0 * math.cos(theta_uv) + self.v0 * math.sin(theta_uv)) / (self.f * ((self.pv * math.sin(theta_uv)) ** 2 + (self.pu * math.cos(theta_uv)) ** 2) ** 0.5) \
                      + self.pu * self.pv * (puv - self.u0 * math.sin(theta_uv) - self.v0 * math.cos(theta_uv)) * (self.pu - self.pv) * math.sin(theta_uv) * math.cos(theta_uv) \
                      / (self.f * ((self.pv * math.sin(theta_uv)) ** 2 + (self.pu * math.cos(theta_uv)) ** 2) **1.5)
                t22 = self.pu * self.pv / (self.f * ((self.pv * math.sin(theta_uv)) **2 + (self.pu * math.cos(theta_uv)) ** 2) ** 0.5)
                line_jacob1 = [t11,t12,t21,t22]
                s3_new = abs(s3)
                abs_judge = s3_new / s3
                s1 = s1 / abs_judge
                s2 = s2 / abs_judge
                p2 = s1
                theta2 = math.asin(s2)
                # p2 = s1; % sin(theta2) = s2; % cos(theta2) = s3;
                # define line_jacob2
                m1 = p2 * A34 - math.sin(theta2) * A14 - math.cos(theta2) * A24
                m2 = A11 * math.sin(theta2) + A21 * math.cos(theta2) - A31 * p2
                m3 = A12 * math.sin(theta2) + A22 * math.cos(theta2) - A32 * p2
                n11 = (m2 / (m3 * (m2 ** 2 + m3 ** 2)) - 1 / m3) * (A11 * math.cos(theta2) - A21 * math.sin(theta2)) + m2 / (m3 * (m2 ** 2 + m3 ** 2)) * (A12 * math.cos(theta2) - A22 * math.sin(theta2))
                n12 = (m2 / (m3 * (m2 ** 2 + m3 ** 2)) - 1 / m3) * (-A31) + m2 / (m3 * (m2 ** 2 + m3 ** 2)) * (-A32)
                n21 = (m2 **2 + m3 ** 2) ** (-0.5) * (-(A14 * math.cos(theta2) - A24 * math.sin(theta2))) - m1 * ((m2 ** 2 + m3 ** 2) ** (-1.5)) * (A11 * math.cos(theta2) - A21 * math.sin(theta2)) - m1 * ((m2 **2 + m3 ** 2) **(-1.5)) * (A11 * math.cos(theta2) - A21 * math.sin(theta2))
                n22 = (m2 ** 2 + m3 ** 2) ** (-0.5) * A34 - m1 * ((m2 ** 2 + m3 ** 2) ** (-1.5)) * (-A31) + m2 / (m3 * (m2 ** 2 + m3 ** 2)) * (-A32)
                line_jacob2 = [n11,n12,n21,n22]
                m2_new = abs(m2)
                m2 = abs(m2)
                abs_judge = m2_new / m2
                m1 = m1 / abs_judge
                m3 = m3 / abs_judge
                p3 = m1 / math.sqrt(m2 * m2 + m3 * m3)
                theta3 = math.asin(m3 / math.sqrt(m2 * m2 + m3 * m3))
                # p3 = m1 / sqrt(m2 * m2 + m3 * m3); % cos(theta3) = m2 / (m2 * m2 + m3 * m3); % sin(theta3) = m3 / (m2 * m2 + m3 * m3);
                # define line_jacob3

                line_p3.append(p3)
                line_theta3.append(theta3)

                if (ii == 1):
                    x4_laser_ref_est = self.x4_laser_ref
                    y4_laser_ref_est = self.y4_laser_ref
                    z4_laser_ref_est = self.z4_laser_ref
                    lenda = (math.cos(theta3) * x4_laser_ref_est + math.sin(theta3) * y4_laser_ref_est - p3) / z4_laser_ref_est
                else:
                    est = [0,0,0,- wx1 * math.cos(theta3),- wy1 * math.sin(theta3),- wz1,-vx1 * math.cos(theta3),- vy1 * math.sin(theta3),vz1,wx1 * p3 * math.sin(theta3),- wy1 * p3 * math.cos(theta3),0]
                    # print "est",numpy.matrix(est)
                    A = numpy.matrix(est).reshape(2,6)
                    inv_est=numpy.linalg.pinv(A)

                    ####not finish
                    print "ii",ii
                    # print "line_theta3",line_theta3
                    lenda_tmp=[(line_theta3[ii-1] - line_theta3[ii - 2]) / self.delta_t,(line_p3[ii-1] - line_p3[ii - 2]) / self.delta_t]
                    lenda_mat = numpy.dot(inv_est,numpy.matrix(lenda_tmp).reshape(1,2).T)
                    lenda = (lenda_mat[3] + lenda_mat[4] + lenda_mat[5]) / 3.0

                # lenda = (math.cos(theta3) * self.x4_laser_ref + math.sin(theta3) * self.y4_laser_ref - p3) / self.z4_laser_ref
                line_jacob3 = [0,0,0,- lenda * (math.cos(theta3)), - lenda * math.sin(theta3),- 1,-math.cos(theta3) ,- math.sin(theta3),lenda,lenda * math.sin(theta3) * p3,- lenda * math.cos(theta3) * p3,0]
                # compute overall jacobian matrix and its inverse jacobian matrix
                # total_jacob = inv(line_jacob2 * line_jacob1) * line_jacob3;
                AA = numpy.matrix(line_jacob3).reshape(2,6)
                inv_line_jacob3 = numpy.linalg.pinv(AA)
                inv_total_jacob = numpy.dot(numpy.dot(inv_line_jacob3, numpy.matrix(line_jacob2).reshape(2,2)),numpy.matrix(line_jacob1).reshape(2,2))

                # start computing velocity of reflection frame 1 2
                delta_theta = theta_uv - theta_uv_desire
                delta_puv = puv - puv_desire
                # print "self.lamda * inv_total_jacob",self.lamda * inv_total_jacob
                # print "numpy.matrix([delta_theta,delta_puv])",numpy.matrix([delta_theta,delta_puv])
                vel_refelection = numpy.dot(-self.lamda * inv_total_jacob,numpy.matrix([delta_theta,delta_puv]).T)
                # print "vel_refelection",vel_refelection
                vx1 = vel_refelection[0]
                vy1 = vel_refelection[1]
                vz1 = vel_refelection[2]
                wx1 = vel_refelection[3]
                wy1 = vel_refelection[4]
                wz1 = vel_refelection[5]
                print "t",t
                ii+=1
                #publish velocity of reflection plate [vx vy vz wx wy wz]
                T_reflection_to_endeffector = numpy.dot(numpy.matrix(T_camera_to_endeffector).reshape(4,4),numpy.matrix(T_reflection_to_camera).reshape(4,4) )
                # print "numpy.matrix(T_camera_to_endeffector).reshape(4,4)",numpy.matrix(T_camera_to_endeffector).reshape(4,4)
                # print "numpy.matrix(T_reflection_to_camera).reshape(4,4)",numpy.matrix(T_reflection_to_camera).reshape(4,4)
                # print "T_reflection_to_endeffector",T_reflection_to_endeffector
                Jacob_reflection_to_endeffector=self.tr2jac(T_reflection_to_endeffector,1)
                # print "Jacob_reflection_to_endeffector",Jacob_reflection_to_endeffector
                v_reflection_to_ee=numpy.dot(Jacob_reflection_to_endeffector,vel_refelection)
                time.sleep(1)
                print "v_reflection_to_ee",v_reflection_to_ee
                #return vx1,vy1,vz1,wx1,wy1,wz1



def main():
    u0 = 321.237
    v0 = 228.900
    alpha = 0.0
    f = 799.2035 * 1e-03
    pu = 9.9377e-04
    pv = 9.9029e-04
    x4_laser_ref=1.0
    y4_laser_ref=2.0
    z4_laser_ref=3.0

    n_uv = 2.0
    u1_desire = 415.0
    v1_desire = 291.0
    u2_desire = 503.0
    v2_desire = 290.0
    u1 = 420.0
    v1 = 331.0
    u2 = 519.0
    v2 = 321.0
    urdfname = "/data/ros/ur_ws/src/universal_robot/ur5_planning/urdf/ur5.urdf"
    T1 = [0.5685,- 0.7033,- 0.4269,0.0974,-0.5357,- 0.7103,0.4567,0.1034,-0.6244,- 0.0310,- 0.7805,0.5306,0,0,0,1.0000]
    #marker to camera frame
    T_reflection_to_camera=[-0.57119722,0.69567303,-0.43562352,0.09475704,0.5922862,0.71676598,0.3680254,0.08832676,0.56826568,-0.04779821,-0.82145222,0.48303625,0. ,0.,0. ,1.  ]
    #camera to ee frame
    T_camera_to_endeffector=[0.594968000000000,-0.587314000000000,0.548703000000000,-0.122910000000000,-0.00030400000000000,-0.682843000000000,-0.730565000000000,0.348829000000000,0.803749000000000,	0.434496000000000,	-0.406449000000000,	0.552213000000000,0,	0,	0,	1]
    s0=StruLight(u0,v0,pu,pv,f,alpha,x4_laser_ref,y4_laser_ref,z4_laser_ref,T1,urdfname)
    puv, theta_uv, puv_desire, theta_uv_desire=s0.Get_p_theta(n_uv,u1_desire,v1_desire,u2_desire,v2_desire,u1,v1,u2,v2)
    print "puv,theta_uv,puv_desire,theta_uv_desire",puv,theta_uv,puv_desire,theta_uv_desire
    print "vx1,vy1,vz1,wx1,wy1,wz1",s0.Get_vxvyvzwxwywz(theta_uv,theta_uv_desire,puv,puv_desire,T1,T_reflection_to_camera,T_camera_to_endeffector)
if __name__=="__main__":
    main()
