#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8

#world axis x = tunnel forward; z = vertical; y = horizontal
#odom axis = laser axis = world axis
#             Tunnel positive
#         111111111111111111111111
#        1           +            1
#       1            +         +   1
#      1             1       +      1
#     1              1     1         1
#    1               1   1            1
#   1                1 1               1
#  1       +++++111111                  1
# 1                                      1
#point cloud axis x = horizontal; y = vertical
#Coding by linshijie, all rights reserve.

import rospy 
import roslib
import numpy as np
import tf
import laser_geometry
import pickle
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from laser_line_extraction.msg import LineSegment
from laser_line_extraction.msg import LineSegmentList
np.set_printoptions(threshold=np.inf)

pc_x = []
pc_y = []

laser_position = []
fusion_position = []

odom_x = odom_y = odom_z = 0
laser_x = 0
laser_y = 0
laser_z = 31
fuse_x = fuse_y = fuse_z = 0
top = 0
init_pitch = init_roll = init_yaw = 0
odom_sec = odom_nsec = 0
laser_sec = laser_nsec = 0
fuse_sec = fuse_nsec = 0
odom_time  = laser_time =  fuse_time = 0.0
laser_right = laser_left = 0
right_line_distance = left_line_distance = 0

odom_roll = odom_pitch = odom_yaw = 0


odom_velx = odom_vely = odom_velz = 0.0
last_odom_velx = last_odom_vely = last_odom_velz = 0

is_right_plain = 1
min_distance = 999999 #set to reduce
record_num = 0 # init record number of point cloud
last_laser_time = 0
SURFACE_GAP = 0.01  #gap use to seperate the wall and others
MIDDLE_ANGLE = 0 #middle angle of laser
MAX_RANGE = 60 #laser max range
MIN_RANGE = 0.3 #laser min range
INIT_FLAG = True #set true to start init 

laser_data_number = 0


min_distance = 0
dyaw = 0

def callback_cloud(cloud):
    global pc_x, pc_y, is_right_plain, record_num, min_distance, dyaw
    #rospy.loginfo("I hearde cloud")
    #TODO use the time stamp
    #x,y(point_cloud) = y,z(world axis)
    if is_right_plain == 1 and record_num < 10:
        pc_x = []
        pc_y = []
        rospy.loginfo("Record the point data")
        print record_num
        for i in range(len(cloud.points)):
            pc_x.append(cloud.points[i].x)
            pc_y.append(cloud.points[i].y)
        #init the front direction
        total_distance =  np.sum(np.square(pc_x))
        min_distance = total_distance / len(cloud.points)
        record_num = record_num + 1
        is_right_plain = 0
    
    #pc_x_sort = np.sort(pc_x,kind='heapsort')
    total_distance = 0
    #TODO用线段分割算法提取有效线段来判断前进方向，而不是直接求二范数
    for i in range(len(cloud.points)):
        total_distance = total_distance + np.square(cloud.points[i].x)
    total_distance = total_distance / len(cloud.points)
    dyaw = total_distance - min_distance
    if dyaw < -0.07:
        min_distance = total_distance
    #遇到数据突变说明隧道进入了另外一个部分，变宽
    if dyaw > 8:
       min_distance = total_distance
    #print min_distance
    #print "dyaw:", dyaw


    
def callback_laser(laser):
    #TODO odom_yaw&odom_pitch&odom_roll judgement
    #TODO check the http://wiki.ros.org/laser_scan_splitter?distro=kinetic  and  http://wiki.ros.org/laser_ortho_projector?distro=kinetic  use the tf tools to replace
    #rospy.loginfo("I hearde laser")
    #data init
    global pc_x, pc_y, laser_position, laser_time, laser_z, laser_y, laser_data_number, dumpdata, top, init_pitch, init_roll, init_yaw
    global laser_right, laser_left, right_line_distance, left_line_distance
    angle_min = laser.angle_min
    angle_max = laser.angle_max
    angle_increment = laser.angle_increment
    range_min = laser.range_min
    range_max = laser.range_max
    ranges = laser.ranges
    range_angle = []
    range_angle_raw = []
    range_filtered = []
    vertical_laser = []
    angle_process = angle = 0
    #Time record
    laser_sec = laser.header.stamp.secs
    laser_nsec = laser.header.stamp.nsecs
    laser_time = laser_sec + laser_nsec * 10**(-9)
    #find usefull range
    #the first element in range is the min angle range
    for i in range(np.shape(ranges)[0]):
        angle = i*angle_increment + angle_min
        angle_process = angle
        angle_process = abs(angle_process)
        #use first quadrant angle
        if angle_process > np.pi/2:
            angle_process = angle_process - np.pi
            angle_process = abs(angle_process)
        range_angle.append(angle_process)
        range_angle_raw.append(angle)
        range_filtered.append(ranges[i])
    #project to y axis
    range_project = range_filtered * np.sin(range_angle)
    #project to z axis
    range_project_top = np.asarray(range_filtered) * np.cos(odom_pitch)
    #find the vertical angle laser
    for i in range(np.shape(range_angle_raw)[0]):
        #vetical = 0
        if range_angle_raw[i]+odom_roll > MIDDLE_ANGLE -0.05 and range_angle_raw[i]+odom_roll < MIDDLE_ANGLE + 0.05:
            vertical_laser.append(range_project_top[i])
    vertical_laser_sorted = np.sort(vertical_laser,kind='heapsort') 
    #simple filter
    if np.shape(vertical_laser)[0] > 5:
        top = np.abs(np.sum(vertical_laser_sorted[np.shape(vertical_laser)[0] / 2 -2 : np.shape(vertical_laser)[0] / 2 +2])) / 4
    else:
        top = np.abs(np.sum(vertical_laser_sorted[np.shape(vertical_laser)[0]/2])) / np.shape(vertical_laser)[0]

    #left and right process
    range_project = range_project * np.cos(odom_yaw - init_yaw)

    range_project_left = range_project[0 : np.shape(range_project)[0] / 2]
    range_project_right = range_project[np.shape(range_project)[0] / 2 : np.shape(range_project)[0]]
    
    #sort the project line use the longest, which is the left or right serface distance
    range_project_right_sorted = np.sort(range_project_right,kind='heapsort')
    range_project_left_sorted = np.sort(range_project_left,kind='heapsort') 
    #fliter the inf and -inf data
    range_project_right_sorted = range_project_right_sorted[ range_project_right_sorted< MAX_RANGE]
    range_project_right_sorted = range_project_right_sorted[ range_project_right_sorted> MIN_RANGE]
    range_project_left_sorted = range_project_left_sorted[ range_project_left_sorted< MAX_RANGE]
    range_project_left_sorted = range_project_left_sorted[ range_project_left_sorted> MIN_RANGE]

    #print range_project_right_sorted

    #use a simple gap to find the clusting of 1D data which represent the wall.
    wall_index_left = [0] #start at 0
    wall_index_right = [0] #start at 0
    for laser in range(np.shape(range_project_right_sorted)[0]-1):
        if range_project_right_sorted[laser+1] - range_project_right_sorted[laser] > SURFACE_GAP:
            wall_index_right.append(laser+1)
    wall_index_right.append(np.shape(range_project_right_sorted)[0]-1)
    d_wall_index_right = np.array(wall_index_right[1:]) - np.array(wall_index_right[0: -1]) 
    max_index_right = np.argmax(d_wall_index_right)
    #lenght = (all laser in gap) / number of laser
    laser_right = np.sum( range_project_right_sorted[wall_index_right[max_index_right] : wall_index_right[max_index_right + 1]] ) /d_wall_index_right[max_index_right]


    for laser in range(np.shape(range_project_left_sorted)[0]-1):
        if range_project_left_sorted[laser+1] - range_project_left_sorted[laser] > SURFACE_GAP:
            wall_index_left.append(laser+1)
    wall_index_left.append(np.shape(range_project_left_sorted)[0]-1)
    d_wall_index_left = np.array(wall_index_left[1:]) - np.array(wall_index_left[0: -1]) 
    max_index_left = np.argmax(d_wall_index_left)
    #lenght = (all laser in gap) / number of laser
    laser_left = np.sum( range_project_left_sorted[wall_index_left[max_index_left] : wall_index_left[max_index_left + 1]] ) /d_wall_index_left[max_index_left]

    
    #TODO try to use dz instead of z
    for i in range(np.shape(pc_x)[0]):
        if np.abs(pc_x[i] - laser_y) < 0.03 and pc_y[i] > 1:
            h = pc_y[i]
            laser_z = h - top
    print "持续输出融合位置  距离顶部：", top, '（一维）左面：',laser_left, '（1维）右面：',laser_right, '（线段分割）左面',left_line_distance,'（线段分割）右面：',right_line_distance,'Y轴：',laser_y



    

def callback_odom(odom):
    global odom_roll, odom_pitch, odom_yaw, init_pitch, init_roll, init_yaw, INIT_FLAG
    global laser_time, laser_y, laser_z, fuse_x, fuse_y, fuse_z, top, last_laser_time, last_odom_vely, last_odom_velz
    
    position_fusion = Odometry()
    #Time record
    odom_sec = odom.header.stamp.secs
    odom_nsec = odom.header.stamp.nsecs
    odom_time = odom_sec + odom_nsec * 10**(-9)



    #rospy.loginfo("I heard Odom")
    #laser_position.append([laser_y,laser_z])

    odom_x = odom.pose.pose.position.x
    odom_y = odom.pose.pose.position.y
    odom_z = odom.pose.pose.position.z

    odom_velx = odom.twist.twist.linear.x
    odom_vely = odom.twist.twist.linear.y
    odom_velz = odom.twist.twist.linear.z

    #Sensor fusion 
    # 这里使用了一个时间差策略，如果把1比作激光雷达更新，0比作odom更新。那么有如下模式：...01000100010..... 仔细思考便能悟出我下面代码的奥妙
    # TODO添加光流的x轴融合
    #laser发生更新,yz轴处理
    if laser_time - last_laser_time != 0 :
        fuse_y = laser_y
        fuse_z = -top  #用和顶部的距离  取反才正确，表示负坐标
        last_odom_time = laser_time
    else:
        last_odom_time = odom_time
        last_odom_vely = odom_vely
        last_odom_velz = odom_velz
    fuse_y = fuse_y + (odom_time - last_odom_time) * last_odom_vely
    fuse_z = fuse_z + (odom_time - last_odom_time) * last_odom_velz
    last_laser_time = laser_time

    #发布数据
    position_fusion.header.frame_id = "laser"
    position_fusion.header.stamp.secs = odom_sec
    position_fusion.header.stamp.nsecs = odom_nsec

    position_fusion.pose.pose.position.x = fuse_x
    position_fusion.pose.pose.position.y = fuse_y
    position_fusion.pose.pose.position.z = fuse_z

    position_fusion.pose.pose.orientation.x =  left_line_distance#暂时用来放left_line_distance，这个和真实含义不对等
    position_fusion.pose.pose.orientation.y =  right_line_distance#暂时用来放right_line_distance，这个和真实含义不对等
    position_fusion.pose.pose.orientation.z = init_yaw #暂时用来放init_yaw，这个和真实含义不对等


    position_fusion.twist.twist.linear.x = odom_velx
    position_fusion.twist.twist.linear.y = odom_vely
    position_fusion.twist.twist.linear.z = odom_velz

    pub.publish(position_fusion)

    #calc the euler angle from quaternion
    qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    (odom_roll,odom_pitch,odom_yaw) = euler_from_quaternion(qn_odom)
    if INIT_FLAG == True:
        init_yaw = odom_yaw
        init_pitch = odom_pitch
        init_roll = odom_roll
        print '初始化角度成功','Pitch:',init_pitch,'Roll:', init_roll,'Yaw', init_yaw
        INIT_FLAG = False

    #print "2", odom_x, odom_y, odom_z
    #print "odom_yaw:",np.degrees(odom_yaw)

def callback_line(line):
    global odom_roll, odom_pitch, odom_yaw, laser_left, laser_right, laser_y, right_line_distance, left_line_distance
    start = []
    end = []
    angle = []

    left_line_length = [0,0]
    right_line_length = [0,0]
    left_surface_start = [0,0]
    left_surface_end = [0,0]
    left_surface_angle = [0]
    right_surface_start = [0,0]
    right_surface_end = [0,0]
    right_surface_angle = [0]

    left_line_length = np.array(left_line_length)
    right_line_length = np.array(right_line_length)
    left_surface_start = np.array(left_surface_start)
    left_surface_end = np.array(left_surface_end)
    left_surface_angle = np.array(left_surface_angle)
    right_surface_start = np.array(right_surface_start)
    right_surface_end = np.array(right_surface_end)
    right_surface_angle = np.array(right_surface_angle)

    
    for i in line.line_segments:
        start.append(i.start)
        end.append(i.end)
        angle.append(i.angle)
    angle = np.array(angle)
    #print "angle", angle
    start = np.array(start)
    #print "start", start
    end = np.array(end)
    #print "end",end
    dline = start - end 
    #print "dline",dline
    gardient = dline[:,1] / dline[:,0]
    #print "roll",odom_roll+np.pi/2
    #k = y/x and add odom_roll as a compensate of the roll movement 
    gardient = np.arctan(gardient) + odom_roll
    #find the verticle(1.57)line and square to make it more differentiable
    gardient = np.square(abs(gardient))
    #print "gar",gardient,np.shape(gardient),odom_roll
    for i in range(len(gardient)):
        #judge the left or right surface
        if abs(gardient[i]) < 0.02:
            # 0度是中点
            if angle[i] < 0:
                left_surface_start = np.vstack((left_surface_start , start[i,:]))
                left_surface_end = np.vstack((left_surface_end,end[i,:]))
                left_surface_angle = np.vstack((left_surface_angle,angle[i]))

            else:
                right_surface_start = np.vstack((right_surface_start,start[i,:]))
                right_surface_end = np.vstack((right_surface_end,end[i,:]))
                right_surface_angle = np.vstack((right_surface_angle,angle[i]))
    #去掉之前放上去的0
    left_surface_start = np.delete(left_surface_start,0,0) 
    left_surface_end = np.delete(left_surface_end,0,0) 
    left_surface_angle = np.delete(left_surface_angle,0,0) 
    right_surface_start = np.delete(right_surface_start,0,0) 
    right_surface_end = np.delete(right_surface_end,0,0) 
    right_surface_angle = np.delete(right_surface_angle,0,0) 

    # print "left start"  ,left_surface_start
    # print "left end"  ,left_surface_end
    # print "left angle"  ,left_surface_angle
    # print "right start"  ,right_surface_start
    # print "right end"  ,right_surface_end
    # print "right angle"  ,right_surface_angle
    #TODO use the height of UAV and other information to select surface(points)
    #if find a line then use line, if not then use the laser 1D cluster points
    if np.shape(left_surface_angle)[0] != 0:
        #use the longest line to indicate a trustable surface 
        left_line_length =  np.sqrt(np.square(left_surface_start - left_surface_end)[:,0] + np.square(left_surface_start - left_surface_end)[:,1])
        pLeftLongest = np.argmax(left_line_length)
        x1_l = left_surface_start[pLeftLongest, 0]
        y1_l = left_surface_start[pLeftLongest, 1]
        x2_l = left_surface_end[pLeftLongest, 0]
        y2_l = left_surface_end[pLeftLongest, 1]
        left_line_distance = abs(x1_l*(y2_l - y1_l) - y1_l*(x2_l - x1_l))/np.sqrt(np.square(y1_l - y2_l) + np.square(x2_l - x1_l))
    else:
        left_line_distance = laser_left
    if np.shape(right_surface_angle)[0] != 0:
        #use the longest line to indicate a trustable surface 
        right_line_length =  np.sqrt(np.square(right_surface_start - right_surface_end)[:,0] + np.square(right_surface_start - right_surface_end)[:,1])
        pRightLongest = np.argmax(right_line_length)        
        x1_r = right_surface_start[pRightLongest, 0]
        y1_r = right_surface_start[pRightLongest, 1]
        x2_r = right_surface_end[pRightLongest, 0]
        y2_r = right_surface_end[pRightLongest, 1]
        right_line_distance = abs(x1_r*(y2_r - y1_r) - y1_r*(x2_r - x1_r))/np.sqrt(np.square(y1_r - y2_r) + np.square(x2_r - x1_r))  
    else:
        right_line_distance = laser_right

    #middle = (left-right)/2
    laser_y = (right_line_distance - left_line_distance) / 2


rospy.init_node('data_process')
seg_line = rospy.Subscriber('/line_segments', LineSegmentList, callback_line)
subcloud = rospy.Subscriber('/point_cloud_ori', PointCloud, callback_cloud)
sublaser = rospy.Subscriber('/scan_filtered', LaserScan, callback_laser)
subodom = rospy.Subscriber('/mavros/local_position/odom', Odometry, callback_odom)

pub = rospy.Publisher('/position_fusion', Odometry, queue_size=1)

rospy.spin()

