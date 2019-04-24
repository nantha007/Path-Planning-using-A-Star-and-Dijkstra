#!/usr/bin/env python3

__author__ = "Nantha Kumar Sunder"
__version__ = "0.1.0"
__license__ = "MIT"

import matplotlib.pyplot as plt
from math import inf
from math import sqrt
import os, sys
# This try-catch is a workaround for Python3 when used with ROS; it is not needed for most platforms
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import cv2
from heapq import heappush, heappop
from Env_rigid import visualize
from Env_rigid import obstaclePlot
import time

def calCost(x,y):
    if (abs(x)==1 and abs(y) ==1):
        return 14
    else:
        return 10

def nearestNode(maze, pos):
    neighbour_pos = [-1,0,1]
    neighbour = []
    for row in neighbour_pos:
        for col in neighbour_pos:
            if (((pos[0] + row )>=0 and (pos[1] + col )>=0 and (pos[0] + row)<len(maze) and (pos[1] + col )<len(maze[0])) and (not(row==0 and col==0))):
                if (maze[pos[0] + row][pos[1] + col] == 0):
                    neighbour.append([calCost(row,col), pos[0] + row , pos[1] + col])
                else:
                    continue
    return neighbour

def AStar(maze, start, end, image, resolution, out):
    node_id = (start[0])*len(maze[0])+start[1];
    image = visualize(start[1]+1,round(150/resolution) -start[0],resolution, image, 5, (0,255,0))
    col_max=round(150/resolution);
    startNode = [0,node_id,-1 ,start[0],start[1], 0 ]
    openlist = []
    closedlist = []
    heappush(openlist, startNode)
    while 1:
        current = heappop(openlist)
        closedlist.append(current)
        if current == startNode:
            image = visualize(current[4]+1,round(150/resolution) -current[3],resolution, image, 5, (0,255,0))
        else:
            image = visualize(current[4]+1,round(150/resolution) -current[3],resolution, image, 5, (0,255,255))
        cv2.imshow('Map', image)
        out.write(image)
        time.sleep(0.0001)
        cv2.waitKey(1)
        if current[4] == end[1] and current[3] ==  end[0]:
            print("Path Found")
            break
        neighbor = nearestNode(maze, [current[3],current[4]])
        for i in range(0,len(neighbor)):
            node_id = neighbor[i][1]*len(maze[0])+neighbor[i][2]
            exist, _ = checkNodeID(node_id, closedlist)
            if exist:
                continue
            h_value = calHeuristic([neighbor[i][1],neighbor[i][2]],end)
            new_node = [current[5] + neighbor[i][0] + h_value,node_id, current[1] ,neighbor[i][1],neighbor[i][2],current[5] + neighbor[i][0]]
            dist = current[5] + neighbor[i][0] + h_value
            #print(dist)
            #print(new_node[0])
            exist, id = checkNodeID(node_id, openlist)
            if exist:
                if dist < openlist[id][0]:
                    openlist[id][0] = new_node[0]
                    openlist[id][2] = new_node[2]
                    openlist[id][5] = new_node[5]
            else:
                heappush(openlist,new_node)
        if not openlist:
            break

    return closedlist, current, out

def findPath(maze, current, closedlist, image, resolution):
    once = 1
    while 1:
        if current[2] == -1:
            image = visualize(current[4]+1,round(150/resolution) -current[3],resolution, image, 5, (0,255,0))
            break
        if once == 1:
            image = visualize(current[4]+1,round(150/resolution) -current[3],resolution, image, 5, (0,0,255))
            once = once + 1
        else:
            exist, id = checkNodeID(current[2], closedlist)
            current = closedlist[id]
            image = visualize(current[4]+1,round(150/resolution) -current[3],resolution, image, 5, (255,0,0))
    return  image

def checkNodeID(item, n_list):
    for i in range(len(n_list)):
        if item == n_list[i][1]:
            return 1, i
    return 0, 0

def calHeuristic(current, goal):
    return round(sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)*10)


def main():
    resolution =  int(input('Size of map is defined as (default size)/resolution \
    \n1 for normal size  \nEnter the resolution ( in postive integer ):'))
    radius =  int(input('Enter the Radius :'))
    clearance =  int(input('Enter the Clearance :'))
    image, maze = obstaclePlot(resolution, clearance, radius)
    col_max = len(maze[0])
    row_max = len(maze)
    frame_width = 250*5
    frame_height = 150*5
    out = cv2.VideoWriter('A_star_Rigid.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 25, (frame_width,frame_height))
    while 1:
        while 1:
            x_start = int(input('Enter the row grid coordinate of start point between 1 and ' + str(row_max) + " :")) -1
            if x_start >= 0 and x_start <= row_max -1:
                break
        while 1:
            y_start = int(input('Enter the col grid coordinate of start point between 1 and ' + str(col_max) + " :")) -1
            if y_start >= 0 and y_start <= col_max-1:
                break
        if maze[row_max - x_start - 1][y_start] > 0:
            print("The value entered is in obstacle")
            continue
        else:
            break
    while 1:

        while 1:
            x_end = int(input('Enter the row grid coordinate of end point between 1 and ' + str(row_max) + " :")) -1
            if x_end >= 0 and x_end <= row_max -1:
                break
        while 1:
            y_end = int(input('Enter the col grid coordinate of end point between 1 and ' + str(col_max) + " :")) -1
            if y_end >= 0 and y_end <= col_max -1:
                break
        if maze[row_max - x_end - 1][y_end] > 0:
            print("The value entered is in obstacle")
            continue
        else:
            break
    start = [row_max - x_start -1, y_start]
    end = [row_max - x_end -1,y_end]
    closedlist, current, out = AStar(maze, start, end,image, resolution, out)
    image = findPath(maze, current, closedlist, image, resolution)
    cv2.imshow('Map', image)
    out.write(image)
    cv2.waitKey(0)

if __name__ == "__main__":
    main()
