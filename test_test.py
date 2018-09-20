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

    # white color mask
    #
    lower = np.uint8([  0, 200,   0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted, lower, upper)
    # black color mask
    # lower = np.uint8([ 0,   50, 0])
    # upper = np.uint8([ 255, 255, 255])
    lower = np.uint8([ 10,   0, 100])
    upper = np.uint8([ 40, 255, 255])
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
    bottom_left = [cols * 0.1, rows * 0.95]
    top_left = [cols * 0.4, rows * 0.6]
    bottom_right = [cols * 0.9, rows * 0.95]
    top_right = [cols * 0.6, rows * 0.6]
    # the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    return filter_region(image, vertices)


def hough_lines(image):
    """
    `image` should be the output of a Canny transform.

    Returns hough lines (not the image with lines)
    """
    return cv2.HoughLinesP(image, rho=1, theta=np.pi / 180, threshold=20, minLineLength=20, maxLineGap=300)


def average_slope_intercept(lines):
    left_lines = []  # (slope, intercept)
    left_weights = []  # (length,)
    right_lines = []  # (slope, intercept)
    right_weights = []  # (length,)

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x2 == x1:
                continue  # ignore a vertical line
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1
            length = np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
            if slope < 0:  # y is reversed in image
                left_lines.append((slope, intercept))
                left_weights.append((length))
            else:
                right_lines.append((slope, intercept))
                right_weights.append((length))

    # add more weight to longer lines
    left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
    right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None

    return left_lane, right_lane  # (slope, intercept), (slope, intercept)


def make_line_points(y1, y2, line):
    """
    Convert a line represented in slope and intercept into pixel points
    """
    if line is None:
        return None

    slope, intercept = line

    # make sure everything is integer as cv2.line requires it
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    y1 = int(y1)
    y2 = int(y2)

    return ((x1, y1), (x2, y2))


def lane_lines(image, lines):
    left_lane, right_lane = average_slope_intercept(lines)

    y1 = image.shape[0]  # bottom of the image
    y2 = y1 * 0.6  # slightly lower than the middle

    left_line = make_line_points(y1, y2, left_lane)
    right_line = make_line_points(y1, y2, right_lane)

    return left_line, right_line


def draw_lane_lines(image, lines, color=[255, 0, 0], thickness=20):
    # make a separate image to draw lines and combine with the orignal later
    line_image = np.zeros_like(image)
    for line in lines:
        if line is not None:
            pass
            # cv2.line(line_image, *line, color, thickness)
    # image1 * α + image2 * β + λ
    # image1 and image2 must be the same shape.
    return cv2.addWeighted(image, 1.0, line_image, 0.95, 0.0)


# lane_images = []
# for image, lines in zip(test_images, list_of_lines):
#     lane_images.append(draw_lane_lines(image, lane_lines(image, lines)))
def main():
    img = cv2.imread('YellowWhite.jpg')
    #img = cv2.imread('test2.png')
    img4 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hsv=convert_hsv(img4)
    select=select_white_yellow(img4)
    gray=convert_gray_scale(select)
    smooth=apply_smoothing(gray)
    edges=detect_edges(smooth)
    region=select_region(edges)

    lines=hough_lines(region)
    print "average_slope_intercept",average_slope_intercept(lines)
    # left_line, right_line=lane_lines(img4,lines)
    # print "left_line, right_line",left_line, right_line
    for x1, y1, x2, y2 in lines[0]:
        cv2.line(img4, (x1, y1), (x2, y2), (0, 255, 0), 2)
    print "img4",img4
    cv2.namedWindow('img4', cv2.WINDOW_NORMAL)
    cv2.imshow('img4',img4)
    cv2.namedWindow('gray', cv2.WINDOW_NORMAL)
    cv2.imshow('gray', smooth)
    cv2.namedWindow('edges', cv2.WINDOW_NORMAL)
    cv2.imshow('edges', edges)
    cv2.namedWindow('hsv', cv2.WINDOW_NORMAL)
    cv2.imshow('hsv', convert_hsv(img4))
    cv2.namedWindow('hsl', cv2.WINDOW_NORMAL)
    cv2.imshow('hsl', convert_hls(img4))
    cv2.namedWindow('region', cv2.WINDOW_NORMAL)
    cv2.imshow('region', region)
    # cv2.namedWindow('lines', cv2.WINDOW_NORMAL)
    # cv2.imshow('lines', average_slope_intercept(lines))
    # print "hhh",select_rgb_white_yellow(img4)
    cv2.waitKey(8)
if __name__=="__main__":
    while(1):
        main()