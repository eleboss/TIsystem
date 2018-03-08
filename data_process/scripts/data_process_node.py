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
import numpy
import tf
import laser_geometry
import pickle
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion


dumpdata = False

pc_x = []
pc_y = []

laser_position = []
odom_position = []
fusion_position = []

odom_x = odom_y = odom_z = 0
laser_x = 0
laser_y = 0
laser_z = 31
fuse_x = fuse_y = fuse_z = 0
top = 0

odom_sec = odom_nsec = 0
laser_sec = laser_nsec = 0
fuse_sec = fuse_nsec = 0
odom_time  = laser_time =  fuse_time = 0.0

odom_roll = odom_pitch = odom_yaw = 0


odom_velx = odom_vely = odom_velz = 0.0
last_odom_velx = last_odom_vely = last_odom_velz = 0

is_right_plain = 1
min_distance = 999999
record_num = 0
last_laser_time = 0
SURFACE_GAP = 0.05  #gap use to seperate the wall and others
MIDDLE_ANGLE = 0
MAX_RANGE = 60 #laser max range
MIN_RANGE = 0.3 #laser min range

laser_data_number = 0
odom_data_number = 0

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
        total_distance =  numpy.sum(numpy.square(pc_x))
        min_distance = total_distance / len(cloud.points)
        record_num = record_num + 1
        is_right_plain = 0
    
    #pc_x_sort = numpy.sort(pc_x,kind='heapsort')
    total_distance = 0
    #TODO用线段分割算法提取有效线段来判断前进方向，而不是直接求二范数
    for i in range(len(cloud.points)):
        total_distance = total_distance + numpy.square(cloud.points[i].x)
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
    global min_distance, pc_x, pc_y, laser_position, laser_time, laser_z, laser_y, laser_data_number, dumpdata, top
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
    #Time record
    laser_sec = laser.header.stamp.secs
    laser_nsec = laser.header.stamp.nsecs
    laser_time = laser_sec + laser_nsec * 10**(-9)
    #find usefull range
    #the first element in range is the min angle range
    for i in range(numpy.shape(ranges)[0]):
        #let the invaild range become 0, then it will not infect the following process
        if ranges[i] < 0.3 and  ranges[i] > 30:
            range[i] = 0
        angle = i*angle_increment + angle_min
        #use first quadrant angle
        if angle > numpy.pi/2:
            angle_process = angle - numpy.pi
        else:
            angle_process = angle
        range_angle.append(angle_process)
        range_angle_raw.append(angle)
        range_filtered.append(ranges[i])
    #use first quadrant angle
    range_angle = numpy.abs(range_angle)
    #project to y axis
    range_project = range_filtered * numpy.cos(range_angle)
    #project to z axis
    range_project_top = numpy.asarray(range_filtered) * numpy.cos(odom_pitch)
    #find the vertical angle laser
    for i in range(numpy.shape(range_angle_raw)[0]):
        #vetical = 0
        if range_angle_raw[i]+odom_roll > MIDDLE_ANGLE -0.05 and range_angle_raw[i]+odom_roll < MIDDLE_ANGLE + 0.05:
            vertical_laser.append(range_project_top[i])
    vertical_laser_sorted = numpy.sort(vertical_laser,kind='heapsort') 
    #simple filter
    if numpy.shape(vertical_laser)[0] > 5:
        top = numpy.abs(numpy.sum(vertical_laser_sorted[numpy.shape(vertical_laser)[0] / 2 -2 : numpy.shape(vertical_laser)[0] / 2 +2])) / 4
    else:
        top = numpy.abs(numpy.sum(vertical_laser_sorted[numpy.shape(vertical_laser)[0]/2])) / numpy.shape(vertical_laser)[0]

    #left and right process
    range_project = range_project * numpy.cos(odom_yaw)
    range_project_right = range_project[0 : numpy.shape(range_project)[0] / 2]
    range_project_left = range_project[numpy.shape(range_project)[0] / 2 : numpy.shape(range_project)[0]]
    #sort the project line use the longest, which is the left or right serface distance
    range_project_right_sorted = numpy.sort(range_project_right,kind='heapsort')
    range_project_left_sorted = numpy.sort(range_project_left,kind='heapsort') 

    #fliter the inf and -inf data
    range_project_right_sorted = range_project_right_sorted[ range_project_right_sorted< MAX_RANGE]
    range_project_right_sorted = range_project_right_sorted[ range_project_right_sorted> MIN_RANGE]
    range_project_left_sorted = range_project_right_sorted[ range_project_left_sorted< MAX_RANGE]
    range_project_left_sorted = range_project_right_sorted[ range_project_left_sorted> MIN_RANGE]

    #use a simple gap to find the clusting of 1D data which represent the wall.
    wall_index_left = [0] #start at 0
    wall_index_right = [0] #start at 0
    for laser in range(numpy.shape(range_project_right_sorted)[0]-1):
        if range_project_right_sorted[laser+1] - range_project_right_sorted[laser] > SURFACE_GAP:
            wall_index_right.append(laser+1)
    wall_index_right.append(numpy.shape(range_project_right_sorted)[0]-1)
    d_wall_index_right = numpy.array(wall_index_right[1:]) - numpy.array(wall_index_right[0: -1]) 
    max_index_right = numpy.argmax(d_wall_index_right)
    #lenght = (all laser in gap) / number of laser
    laser_right = numpy.sum( range_project_right_sorted[wall_index_right[max_index_right] : wall_index_right[max_index_right + 1]] ) /d_wall_index_right[max_index_right]

    for laser in range(numpy.shape(range_project_left_sorted)[0]-1):
        if range_project_left_sorted[laser+1] - range_project_left_sorted[laser] > SURFACE_GAP:
            wall_index_left.append(laser+1)
    wall_index_left.append(numpy.shape(range_project_left_sorted)[0]-1)
    d_wall_index_left = numpy.array(wall_index_left[1:]) - numpy.array(wall_index_left[0: -1]) 
    max_index_left = numpy.argmax(d_wall_index_left)
    #lenght = (all laser in gap) / number of laser
    laser_left = numpy.sum( range_project_left_sorted[wall_index_left[max_index_left] : wall_index_left[max_index_left + 1]] ) /d_wall_index_left[max_index_left]

    #middle = (left-right)/2
    laser_y = (laser_right - laser_left) / 2
    
    #TODO try to use dz instead of z
    for i in range(numpy.shape(pc_x)[0]):
        if numpy.abs(pc_x[i] - laser_y) < 0.03 and pc_y[i] > 1:
            h = pc_y[i]
            laser_z = h - top



    

def callback_odom(odom):
    global odom_data_number, odom_roll, odom_pitch, odom_yaw, odom_position
    global dumpdata, laser_time, laser_y, laser_z, fuse_x, fuse_y, fuse_z, odom_fusion, top, last_laser_time, last_odom_vely, last_odom_velz
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
        fuse_z = top  #用和顶部的距离
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

    position_fusion.pose.pose.orientation.z = dyaw #暂时用来放dyaw，这个和真实含义不对等

    position_fusion.twist.twist.linear.x = odom_velx
    position_fusion.twist.twist.linear.y = odom_vely
    position_fusion.twist.twist.linear.z = odom_velz


   
    
    pub.publish(position_fusion)

    #calc the euler angle from quaternion
    qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    (odom_roll,odom_pitch,odom_yaw) = euler_from_quaternion(qn_odom)



    #print "2", odom_x, odom_y, odom_z
    #print "odom_yaw:",numpy.degrees(odom_yaw)




rospy.init_node('data_process')

subcloud = rospy.Subscriber('/point_cloud_ori', PointCloud, callback_cloud)
sublaser = rospy.Subscriber('/scan_filtered', LaserScan, callback_laser)
subodom = rospy.Subscriber('/mavros/local_position/odom', Odometry, callback_odom)

pub = rospy.Publisher('/position_fusion', Odometry, queue_size=0)

rospy.spin()

