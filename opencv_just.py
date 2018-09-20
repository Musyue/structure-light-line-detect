#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import time
import numpy as np
def convert_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
def convert_hls(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
# image is expected be in RGB color space
def select_rgb_white_yellow(image):
    # white color mask
    lower = np.uint8([200, 200, 200])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(image, lower, upper)
    # yellow color mask
    lower = np.uint8([190, 190,   0])
    upper = np.uint8([255, 255, 255])
    yellow_mask = cv2.inRange(image, lower, upper)
    # combine the mask
    mask = cv2.bitwise_or(white_mask, yellow_mask)
    masked = cv2.bitwise_and(image, image, mask = mask)
    return masked

def convert_gray_scale(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def apply_smoothing(image, kernel_size=15):
    """
    kernel_size must be postivie and odd
    """
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

def detect_edges(image, low_threshold=50, high_threshold=150):
    return cv2.Canny(image, low_threshold, high_threshold)

def select_white_yellow(image):
    converted = convert_hls(image)
    #print "con-------",converted
    # white color mask
    #
    lower = np.uint8([  0, 200,   0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted, lower, upper)
    # black color mask
    lower = np.uint8([ 0,   0, 0])
    upper = np.uint8([ 70, 255, 255])
    yellow_mask = cv2.inRange(converted, lower, upper)
    # combine the mask
    mask = cv2.bitwise_or(white_mask, yellow_mask)
    return cv2.bitwise_and(image, image, mask = mask)


def filter_region(image, vertices):
    """
    Create the mask using the vertices and apply it to the input image
    """
    mask = np.zeros_like(image)
    if len(mask.shape) == 2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,) * mask.shape[2])  # in case, the input image has a channel dimension
    return cv2.bitwise_and(image, mask)


def select_region(image):
    """
    It keeps the region surrounded by the `vertices` (i.e. polygon).  Other area is set to 0 (black).
    """
    # first, define the polygon by vertices
    rows, cols = image.shape[:2]
    bottom_left = [cols * 0.53, rows * 0.70]
    top_left = [cols * 0.53, rows * 0.28]
    bottom_right = [cols * 0.95, rows * 0.70]
    top_right = [cols * 0.99, rows * 0.28]
    # the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    return filter_region(image, vertices)


def hough_lines(image):
    """
    `image` should be the output of a Canny transform.

    Returns hough lines (not the image with lines)
    """
    return cv2.HoughLinesP(image, rho=1, theta=np.pi / 180, threshold=20, minLineLength=20, maxLineGap=300)
"""
line=[(-0.2867132867132867, 375.1958041958042), (-0.013986013986013986, 204.23776223776224), (-0.2857142857142857, 382.85714285714283), (-0.03292181069958848, 216.3127572016461)]
"""
def Get_same_slope(line):
    result_list = []
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
        print "re------",result_list
        # print "result",(slop_sum/len(result_list),intercept_sum/len(result_list))
        print "result_list_final",result_list_final
        print "result", (slop_sum_final / len(result_list_final), intercept_sum_final / len(result_list_final))
        return (slop_sum_final / len(result_list_final), intercept_sum_final / len(result_list_final))
    else:
        pass
def average_slope_intercept(lines):
    left_lines = []  # (slope, intercept)
    left_weights = []  # (length,)
    right_lines = []  # (slope, intercept)
    right_weights = []  # (length,)
    all_lines=[]
    zero_lines =[]
    zero_weights = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            # if x2 == x1:
            #     continue  # ignore a vertical line
            # print "x1, y1, x2, y2",x1, y1, x2, y2
            slope = (y2 - y1)*0.1 / ((x2 - x1)*0.1)

            intercept = y1 - slope * x1
            length = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
            # print "slope,intercept,length",slope,intercept,length
            all_lines.append((slope, intercept))
            if slope < 0:  # y is reversed in image
                left_lines.append((slope, intercept))
                left_weights.append((length))
            elif slope>0:
                right_lines.append((slope, intercept))
                right_weights.append((length))
            elif slope==0:
                zero_lines.append((slope, intercept))
                zero_weights.append((length))
            else :
                pass
        left_lane=Get_same_slope(left_lines)
        right_lane= Get_same_slope(right_lines)
        print "left_lane, right_lane",left_lane, right_lane
    return left_lane, right_lane
            # else:
            #     fixed_point=((x1, y1),(x2, y2))
    # add more weight to longer lines
    # temp=all_lines[0]
    # temppp=0.0-abs(temp[0])
    # for i in all_lines:
    #     tt=0-abs(i[0])
    #     if tt>temppp:
    #         temp=i
    #     temppp=tt
    # print "left_lines",left_lines
    # print "right_lines",right_lines
    # print "zero_lines",zero_lines
    # left_slop=
    # for i in left_lines:

    # left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
    # right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None
    # # fixed_lane = np.dot(zero_weights, zero_lines) / np.sum(zero_weights) if len(zero_weights) > 0 else None
    # print "np.dot(left_weights, left_lines)",np.dot(left_weights, left_lines)
    # print "left_lane, right_lane,fixed_lane",left_lane, right_lane
    # fixed_lane = (0,temp[1])
    # try:
    #     if (left_lane.tolist()[0]!=float("-inf") and left_lane.tolist()[0]!=float("inf") and right_lane.tolist()[0]!=float("-inf") and right_lane.tolist()[0]!=float("inf")) :
    #         # if (fixed_lane and left_lane and right_lane) is None:
    #         #     pass
    #         # else:
    #         return left_lane, right_lane  # (slope, intercept), (slope, intercept)
    #     else:
    #         return 0
    # except:
    #     pass


def make_line_points(y1, y2, line):
    """
    Convert a line represented in slope and intercept into pixel points
    """
    if line is None:
        return None

    slope, intercept = line
    # try:
        # make sure everything is integer as cv2.line requires it
    if slope!=0 and slope != (float("-inf")) and slope !=float("inf"):
        x1 = int((y1 - intercept)*0.1 / (slope*0.1))
        x2 = int((y2 - intercept)*0.1 / (slope*0.1))
        y1 = int(y1)
        y2 = int(y2)

        return ((x1, y1), (x2, y2))
    elif slope==0:
        print "slope is zero"
        return ((300, int(intercept)), (900, int(intercept)))
    else:
        pass
    # except:
    #     print "you need start structure light"

def get_same_point(left_lane, right_lane):
    try:
        if (left_lane[0]-right_lane[0])!=0:
            # print "left_lane[0],right_lane[0]",left_lane[0],right_lane[0]
            x=(right_lane[1]-left_lane[1])/(left_lane[0]-right_lane[0])
            y=left_lane[0]*x+left_lane[1]
            return (int(x),int(y))
        else:
            print "left and right line vertical"
    except:
        print "you need to start structure light"

def lane_lines(image, lines):
    if average_slope_intercept(lines)!=0:
        left_lane, right_lane= average_slope_intercept(lines)

        same_point=get_same_point(left_lane, right_lane)
        y1 = image.shape[0] * 0.5  # bottom of the image
        y2 = y1 * 0.1  # slightly lower than the middle
        color1 = [0, 0, 255]
        color2 = [255, 0, 0]
        color3 = [0, 255, 0]
        thickness = 5
        left_line = make_line_points(y1, y2, left_lane)
        right_line = make_line_points(y1, y2, right_lane)
        # fixed_line = make_line_points(y1, y2, fixed_lane)
        print "same_point-----------------",same_point
        # print "left_lane, right_lane,fixed_lane", left_lane, right_lane, fixed_lane
        # print "left_line,right_line", left_line, right_line ,fixed_line
        cv2.line(image, left_line[0], left_line[1], color1, thickness)
        cv2.line(image,right_line[0],right_line[1], color2, thickness)
        # cv2.line(image, right_line[0], right_line[1], [255,255,0], thickness)
        cv2.line(image, (301,195), (626,204), color3, thickness)
        return left_line, right_line
    else:
        pass



# def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=20):
#     # make a separate image to draw lines and combine with the orignal later
#     line_image = np.zeros_like(image)
#     for line in lines:
#         if line is not None:
#             cv2.line(line_image, *line, color, thickness)
#     # image1 * α + image2 * β + λ
#     # image1 and image2 must be the same shape.
#     return cv2.addWeighted(image, 1.0, line_image, 0.95, 0.0)


##################
DELAY = 0.02
USE_CAM = 1
IS_FOUND = 0

MORPH = 7
CANNY = 250
##################
_width  = 480.0
_height = 640.0
_margin = 0.0
##################
def main():
    if USE_CAM: video_capture = cv2.VideoCapture(0)
    ret, rgb = video_capture.read()
    print "ret",ret
    time.sleep(1)
    # ret=1
    if (ret):
        # img = cv2.imread('1324255489.jpg')
        # img4 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img4=rgb.copy()
        # print "img4",img4
        hsv=convert_hsv(img4)
        sehsv=select_rgb_white_yellow(hsv)
        select=select_white_yellow(img4)
        gray=convert_gray_scale(select)
        smooth=apply_smoothing(gray)
        edges=detect_edges(smooth)
        region=select_region(edges)

        lines=hough_lines(region)
        # lane_lines(img4, lines)
        try:
            # left_lane, right_lane=average_slope_intercept(lines)
            # print "left_lane, right_lane", left_lane, right_lane
            # left_line, right_line = lane_lines(rgb, lines)
            # print "left_line, right_line", left_line, right_line
            # lane_lines(rgb, lines)
            lane_lines(img4, lines)
            # for x1, y1, x2, y2 in lines[0]:
            #     cv2.line(img4, (x1, y1), (x2, y2), (255, 0, 255), 2)#(bgr)
            #     print "x1, y1, x2, y2",x1, y1, x2, y2
            # print len(lines)
            # print "img4",img4

            cv2.namedWindow('img4', cv2.WINDOW_NORMAL)
            cv2.imshow('img4',img4)
            cv2.namedWindow('select', cv2.WINDOW_NORMAL)
            cv2.imshow('select', select)
            cv2.namedWindow('edges', cv2.WINDOW_NORMAL)
            cv2.imshow('edges', edges)
            cv2.namedWindow('hsv', cv2.WINDOW_NORMAL)
            cv2.imshow('hsv', convert_hsv(img4))
            cv2.namedWindow('hsl', cv2.WINDOW_NORMAL)
            cv2.imshow('hsl', convert_hls(img4))
            cv2.namedWindow('region', cv2.WINDOW_NORMAL)
            cv2.imshow('region', region)
            #cv2.namedWindow('lines', cv2.WINDOW_NORMAL)
            # cv2.imshow('lines', average_slope_intercept(lines))
            # print "hhh",select_rgb_white_yellow(img4)
            cv2.waitKey(8)
        except:
            pass
    else:
        pass
if __name__=="__main__":
    while(1):
        main()