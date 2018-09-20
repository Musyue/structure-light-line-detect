#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import numpy as np
from ur5_planning.msg import uv
from ur5_planning.msg import structlight
class Deteclight:

    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.rgb_image=None
        self.light_pub = rospy.Publisher("/structlight_uv", structlight, queue_size=10)
    def callback(self,data):
        try:
            video_capture=self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.rgb_image = video_capture.copy()
        except CvBridgeError as e:
            print e

    def convert_hsv(self,image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    def convert_hls(self,image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)

    # image is expected be in RGB color space
    def select_rgb_white_yellow(self,image):
        # white color mask
        lower = np.uint8([200, 200, 200])
        upper = np.uint8([255, 255, 255])
        white_mask = cv2.inRange(image, lower, upper)
        # yellow color mask
        lower = np.uint8([190, 190, 0])
        upper = np.uint8([255, 255, 255])
        yellow_mask = cv2.inRange(image, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        masked = cv2.bitwise_and(image, image, mask=mask)
        return masked

    def convert_gray_scale(self,image):
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    def apply_smoothing(self,image, kernel_size=15):
        """
        kernel_size must be postivie and odd
        """
        return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

    def detect_edges(self,image, low_threshold=50, high_threshold=150):
        return cv2.Canny(image, low_threshold, high_threshold)

    def select_white_yellow(self,image):
        converted = self.convert_hls(image)
        # converted = self.convert_hsv(image)

        # print "con-------",converted
        # white color mask
        #
        lower = np.uint8([0, 200, 0])
        upper = np.uint8([255, 255, 255])
        white_mask = cv2.inRange(converted, lower, upper)
        # black color mask
        lower = np.uint8([0, 0, 0])
        upper = np.uint8([70, 255, 255])
        yellow_mask = cv2.inRange(converted, lower, upper)
        # combine the mask
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        return cv2.bitwise_and(image, image, mask=mask)

    def filter_region(self,image, vertices):
        """
        Create the mask using the vertices and apply it to the input image
        """
        mask = np.zeros_like(image)
        if len(mask.shape) == 2:
            cv2.fillPoly(mask, vertices, 255)
        else:
            cv2.fillPoly(mask, vertices, (255,) * mask.shape[2])  # in case, the input image has a channel dimension
        return cv2.bitwise_and(image, mask)

    def select_region(self,image,bottom_left_cols1,bottom_left_rows1,top_left_cols1,top_left_rows1,bottom_right_cols1,bottom_right_rows1,top_right_cols1,top_right_rows1):
        """
        It keeps the region surrounded by the `vertices` (i.e. polygon).  Other area is set to 0 (black).
        bottom_left_cols1=0.53
        bottom_left_rows1=0.70
        top_left_cols1=0.53
        top_left_rows1=0.28
        bottom_right_cols1=0.95
        bottom_right_rows1=0.70
        top_right_cols1=0.99
        top_right_rows1=0.28
        """
        # first, define the polygon by vertices
        rows, cols = image.shape[:2]
        bottom_left = [cols * bottom_left_cols1, rows * bottom_left_rows1]
        top_left = [cols * top_left_cols1, rows * top_left_rows1]
        bottom_right = [cols * bottom_right_cols1, rows * bottom_right_rows1]
        top_right = [cols *top_right_cols1, rows * top_right_rows1]
        # the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        return self.filter_region(image, vertices)

    def hough_lines(self,image):
        """
        `image` should be the output of a Canny transform.

        Returns hough lines (not the image with lines)
        """
        return cv2.HoughLinesP(image, rho=1, theta=np.pi / 180, threshold=20, minLineLength=20, maxLineGap=300)

    """
    line=[(-0.2867132867132867, 375.1958041958042), (-0.013986013986013986, 204.23776223776224), (-0.2857142857142857, 382.85714285714283), (-0.03292181069958848, 216.3127572016461)]
    get the best line 
    """

    def Get_same_slope(self,line):
        # result_list = []
        if line is not None:
            result_list = []
            for ii in xrange(len(line)):
                start = line[ii][0]
                for i in line:
                    # print i[0]
                    if abs(abs(i[0]) - abs(start)) <= 0.01 and abs(abs(i[0]) - abs(start)) != 0:
                        result_list.append(i)
                    else:
                        pass
            if len(result_list)!=0:
                slop_sum = 0
                slop_sum_final = 0
                intercept_sum_final = 0
                for i in result_list:
                    slop_sum += i[0]
                avg_slope = slop_sum / len(result_list)
                result_list_final = []
                for i in result_list:
                    if abs(i[0]) > abs(avg_slope):
                        result_list_final.append(i)
                    else:
                        pass
                for i in result_list_final:
                    slop_sum_final += i[0]
                    intercept_sum_final += i[1]
                if len(result_list_final)!=0:
                    # print "re------", result_list
                    # print "result",(slop_sum/len(result_list),intercept_sum/len(result_list))
                    # print "result_list_final", result_list_final
                    # print "result", (slop_sum_final / len(result_list_final), intercept_sum_final / len(result_list_final))
                    return (slop_sum_final / len(result_list_final), intercept_sum_final / len(result_list_final))
                else:
                    pass
            else:
                pass
        else:
            pass

    def average_slope_intercept(self,lines):
        left_lines = []  # (slope, intercept)
        left_weights = []  # (length,)
        right_lines = []  # (slope, intercept)
        right_weights = []  # (length,)
        all_lines = []
        zero_lines = []
        zero_weights = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                # if x2 == x1:
                #     continue  # ignore a vertical line
                # print "x1, y1, x2, y2",x1, y1, x2, y2
                slope = (y2 - y1) * 0.1 / ((x2 - x1) * 0.1)

                intercept = y1 - slope * x1
                length = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
                # print "slope,intercept,length",slope,intercept,length
                all_lines.append((slope, intercept))
                # if slope < 0:  # y is reversed in image
                #     left_lines.append((slope, intercept))
                #     left_weights.append((length))
                # elif slope > 0:
                #     right_lines.append((slope, intercept))
                #     right_weights.append((length))
                # elif slope == 0:
                #     zero_lines.append((slope, intercept))
                #     zero_weights.append((length))
                # else:
                #     print "line error------"
            all_line = self.Get_same_slope(all_lines)
            # right_lane = self.Get_same_slope(right_lines)
            # print "left_lane, right_lane", left_lane, right_lane
        return all_line

    def make_line_points(self,y1, y2, line):
        """
        Convert a line represented in slope and intercept into pixel points
        """
        if line is None:
            return None

        slope, intercept = line
        # try:
        # make sure everything is integer as cv2.line requires it
        if slope != 0 and slope != (float("-inf")) and slope != float("inf"):
            x1 = int((y1 - intercept) * 0.1 / (slope * 0.1))
            x2 = int((y2 - intercept) * 0.1 / (slope * 0.1))
            y1 = int(y1)
            y2 = int(y2)

            return ((x1, y1), (x2, y2))
        elif slope == 0:
            print "slope is zero"
            return ((300, int(intercept)), (900, int(intercept)))
        else:
            pass
        # except:
        #     print "you need start structure light"

    # def get_same_point(self,left_, right_lane):
    #     try:
    #         if (left_lane[0] - right_lane[0]) != 0:
    #             # print "left_lane[0],right_lane[0]",left_lane[0],right_lane[0]
    #             x = (right_lane[1] - left_lane[1]) / (left_lane[0] - right_lane[0])
    #             y = left_lane[0] * x + left_lane[1]
    #             return (int(x), int(y))
    #         else:
    #             print "left and right line vertical"
    #     except:
    #         print "you need to start structure light"

    def sturcture_lines(self,image, lines):
        if image is not None:
            if self.average_slope_intercept(lines) != 0:
                all_line = self.average_slope_intercept(lines)
                # same_point = self.get_same_point(left_lane, right_lane)

                y1 = image.shape[0] #* 0.5  # bottom of the image
                y2 = y1 * 0.1  # slightly lower than the middle
                # color1 = [0, 0, 255]
                # color2 = [255, 0, 0]
                # color3 = [0, 255, 0]
                # thickness = 5
                line_info = self.make_line_points(y1, y2, all_line)
                # right_line = self.make_line_points(y1, y2, right_lane)
                # fixed_line = make_line_points(y1, y2, fixed_lane)
                # print "same_point-----------------", same_point
                # print "left_lane, right_lane,fixed_lane", left_lane, right_lane, fixed_lane
                # print "left_line,right_line", left_line, right_line
                # cv2.line(image, left_line[0], left_line[1], color1, thickness)
                # cv2.line(image, right_line[0], right_line[1], color2, thickness)
                # # cv2.line(image, right_line[0], right_line[1], [255,255,0], thickness)
                # cv2.line(image, (301, 195), (626, 204), color3, thickness)
                return line_info
            else:
                pass
    def process_rgb_image(self,image):
        #print "rgb_image\n",rgb
        color1 = [0, 0, 255]
        color2 = [255, 0, 0]
        color3 = [0, 255, 0]
        thickness = 5
        try:
            # print "image from process rbg image",image
            hsv = self.convert_hsv(image)
            sehsv = self.select_rgb_white_yellow(hsv)
            select = self.select_white_yellow(image)
            gray = self.convert_gray_scale(select)
            smooth = self.apply_smoothing(gray)
            edges = self.detect_edges(smooth)
            """
            bottom_left_cols1=0.53
            bottom_left_rows1=0.70
            top_left_cols1=0.53
            top_left_rows1=0.28
            bottom_right_cols1=0.45
            bottom_right_rows1=0.70
            top_right_cols1=0.45
            top_right_rows1=0.28
            """
            region_left = self.select_region(edges,0.44,0.80,0.44,0.15,0.70,0.80,0.70,0.15)
            region_right = self.select_region(edges, 0.70, 0.80, 0.70, 0.15, 0.99, 0.80, 0.99, 0.15)

            lines_left = self.hough_lines(region_left)
            lines_right = self.hough_lines(region_right)

            left_line_info=self.sturcture_lines(image,lines_left)
            right_line_info = self.sturcture_lines(image, lines_right)

            # print "left_line,right_line", left_line, right_line
            struct_light=structlight()
            uvuv=uv()
            cv2.line(image, (273, 259), (438, 168), color3, thickness)
            cv2.line(image, (455, 170), (638, 216), color3, thickness)

            if left_line_info!=None:
                cv2.line(image, left_line_info[0], left_line_info[1], color1, thickness)
                # cv2.line(image, right_line[0], right_line[1], color2, thickness)
                # # cv2.line(image, right_line[0], right_line[1], [255,255,0], thickness)

                # print "left_line,right_line",left_line,right_line
                struct_light.left_uv0.uvinfo = [left_line_info[0][0], left_line_info[0][1]]
                struct_light.left_uv1.uvinfo= [left_line_info[1][0], left_line_info[1][1]]
                # struct_light.right_uv0.uvinfo = [right_line[0][0], right_line[0][1]]
                # struct_light.right_uv1.uvinfo = [right_line[1][0], right_line[1][1]]
                self.light_pub.publish(struct_light)
            else:
                # print "no left_line or no right_line,----->left_line\n",left_line
                pass
                # print "no left_line or no right_line----->right_line\n", right_line
            if right_line_info!=None:
                # cv2.line(image, left_line[0], left_line[1], color1, thickness)
                cv2.line(image, right_line_info[0], right_line_info[1], color2, thickness)
                # # cv2.line(image, right_line[0], right_line[1], [255,255,0], thickness)

                # print "left_line,right_line",left_line,right_line
                # struct_light.left_uv0.uvinfo = [left_line[0][0], left_line[0][1]]
                # struct_light.left_uv1.uvinfo= [left_line[1][0], left_line[1][1]]
                struct_light.right_uv0.uvinfo = [right_line_info[0][0], right_line_info[0][1]]
                struct_light.right_uv1.uvinfo = [right_line_info[1][0], right_line_info[1][1]]
                self.light_pub.publish(struct_light)
            else:
                pass
                # print "no left_line or no right_line,----->left_line\n",left_line
                #print "no left_line or no right_line----->right_line\n", right_line
            print "left_line,right_line",left_line_info,right_line_info

            cv2.namedWindow('img4', cv2.WINDOW_NORMAL)
            cv2.imshow('img4', image)
            cv2.namedWindow('select', cv2.WINDOW_NORMAL)
            cv2.imshow('select', select)
            cv2.namedWindow('edges', cv2.WINDOW_NORMAL)
            cv2.imshow('edges', edges)
            cv2.namedWindow('hsv', cv2.WINDOW_NORMAL)
            cv2.imshow('hsv',hsv)
            cv2.namedWindow('hsl', cv2.WINDOW_NORMAL)
            cv2.imshow('hsl', self.convert_hls(image))
            cv2.namedWindow('region_left', cv2.WINDOW_NORMAL)
            cv2.imshow('region_left', region_left)
            cv2.namedWindow('region_right', cv2.WINDOW_NORMAL)
            cv2.imshow('region_right', region_right)
                # return left_line, right_line
        except:
            print "draw line error--------------------"


        cv2.waitKey(8)

        # # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        except CvBridgeError as e:
            print e



def main():
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_structure_light")
        rospy.loginfo("Starting cv_bridge_structure_light node")
        k=Deteclight()
        while not rospy.is_shutdown():
                img4 = k.rgb_image
                if img4 is not None:
                    # print "img4",img4
                    k.process_rgb_image(k.rgb_image)
                    time.sleep(0.1)
            # except:
            #     pass
       # rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv2.destroyAllWindows()
if __name__=="__main__":
    main()