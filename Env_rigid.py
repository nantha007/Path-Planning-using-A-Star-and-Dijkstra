#!/usr/bin/env python3

__author__ = "Nantha Kumar Sunder"
__version__ = "0.1.0"
__license__ = "MIT"

import matplotlib.pyplot as plt
import os, sys
# This try-catch is a workaround for Python3 when used with ROS; it is not needed for most platforms
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import cv2
import numpy as np

def circle(image, scale):
    x = 190*scale
    y = (150-130)*scale
    radius = 15*scale
    #cv2.circle(image, (x,y), radius, (0, 0, 0), -1)
    # using half plane to create obstacle
    for i in range(0, len(image[0])):
        for j in range(0, len(image)):
            if (i-x)**2 + (j-y)**2 <=(radius)**2:
                image[j,i] = [0,0,0]
    return image

def ellipse(image, scale):
    x = 140 * scale
    y = (150 - 120) * scale
    x_axis = 15*scale
    y_axis = 6*scale
    #cv2.ellipse(image, (x,y), (x_axis,y_axis), 0, 0,360, (0,0,0), thickness=-1, lineType=8, shift=0)
    # using half plane to create obstacle
    for i in range(0, len(image[0])):
        for j in range(0, len(image)):
            if ((i-x)**2/x_axis**2) + ((j-y)**2/y_axis**2) <=1:
                image[j,i] = [0,0,0]
    return image

def rect(image, scale):
    x1 = 50*scale
    y1 = round((60-22.5)*scale)
    x2 = 100*scale
    y2 = round((60+22.5)*scale)
    black = [0,0,0]
    # using half plane to create obstacle
    for i in range(0, len(image[0])):
        for j in range(0, len(image)):
            if i >= x1 and i<=x2  and j<=y2 and j>=y1:
                image[j,i] = 0
    return image

def polygon(image, scale):
    #poly_pts = np.array([[[125, 150-56], [163,150-52],[170,150-90],[193,150-52],[173,150-15],[150,150-15]]], dtype=np.int32)*scale
    #cv2.fillPoly(image, poly_pts, (0,255,0) )
    # using half plane to create obstacle
    for i in range(0, len(image[0])):
        for j in range(0, len(image)):
            if (len(image)-j>=15*scale and (-len(image)+j + 1.85*i - 1525 <=0 ) and (-len(image)+j - 1.6522*i + 1854.3 >=0 ) \
            and (len(image)-j-5.4286*i + 4163.4 <= 0)) or ((len(image)-j+0.1053*i - 345.7895 <= 0) and (-len(image)+j-1.64*i + 1305 <= 0) \
            and (len(image)-j-5.4286*i + 4163.4 >= 0) and len(image)-j>=15*scale):
                image[j,i] = [0,0,0]
    return image

def drawGrid(image, scale, resolution):
    x = np.linspace(0,250,round(250/resolution) +1)*scale
    y = np.linspace(0,150,round(150/resolution) +1)*scale
    x_min = 0
    x_max = 250* scale
    y_min = 0
    y_max = 150 * scale
    for i in range(0,len(x)):
        cv2.line(image, (int(x[i]), y_min),(int(x[i]),y_max),(0,0,0),2)
    for i in range(0,len(y)):
        cv2.line(image, (x_min, int(y[i])),(x_max,int(y[i])),(0,0,0),2)
    return image

def visualize(row,col,resolution, image, scale, color):
    x = np.linspace(0,250,round(250/resolution)+1)*scale
    x.astype(int)
    y = np.linspace(0,150,round(150/resolution)+1)*scale
    y.astype(int)
    if row != 0:
        x_min = int(x[row -1]+2)
        x_max = int(x[row] -2)
    else:
        x_min = 2
        x_max = int(x[1] -2)

    if col!=0:
        y_min = 150*scale - int(y[col-1]+2)
        y_max = 150*scale - int(y[col] -2)
    else:
        y_max = 150*scale - int(y[1] -2)
        y_min = 150*scale - 2
    cv2.rectangle(image, (x_min,y_min), (x_max,y_max),color, -1)
    return image

def getMap(image, resolution, scale):
    gray_im = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    (thresh, im_bw) = cv2.threshold(gray_im, 220, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    im_bw = cv2.resize(im_bw, (0,0), fx=1/(scale*resolution), fy=1/(scale*resolution))
    im_bw[im_bw<255] =0
    im_bw[im_bw==255] = 1
    im_bw = 1-im_bw
    return im_bw

def minkowski(image, radius, clearance, scale):
    gray_im = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    height, width = gray_im.shape
    cv2.line(gray_im,(0,0),(0,height-1),(0,0,0),1)
    cv2.line(gray_im,(0,0),(width-1,0),(0,0,0),1)
    cv2.line(gray_im,(width-1,height-1),(width-1,0),(0,0,0),1)
    cv2.line(gray_im,(width-1,height-1),(0,height-1),(0,0,0),1)
    for row in range(0,height):
        for col in range(0,width):
            if gray_im[row][col] <= 125:
                cv2.circle(image, (col,row), (radius+clearance)*scale, (0, 0, 0), -1)
    return image

def mapApproximation(image, map, scale, resolution):
    black = (0,0,0)
    white = (255,255,255)
    for i in range(0, len(map[0])-1):
        for j in range(0,len(map)-1):
            if map[j][i]==1:
                image = visualize(i +1,len(map) - j,resolution, image, scale, black)
            else:
                image = visualize(i +1,len(map) - j,resolution, image, scale, white)
    return image


def obstaclePlot(resolution, clearance, radius):
    scale = 5
    image = np.ones((150*scale,250*scale,3), dtype="uint8")*255
    image = circle(image, scale)
    image = ellipse(image, scale)
    image = rect(image, scale)
    image = polygon(image, scale)
    image = minkowski(image, radius, clearance, scale)
    map = getMap(image, resolution, scale)
    image = drawGrid(image, scale, resolution)
    image = mapApproximation(image, map, scale, resolution)
    return image, map

if __name__ == "__main__":
    resolution = 2
    image, map = obstaclePlot(resolution)
