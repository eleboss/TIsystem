#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8

import rospy 
import roslib
import PID
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget, AttitudeTarget, State
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, PointStamped, Vector3, Vector3Stamped, TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion



feedback_x = feedback_y = feedback_z = feedback_yaw =  0.0
last_feedback_yaw = 0
ukf_x = ukf_y = ukf_z = dyaw = 0
d_dyaw = []
dd_dyaw = []
direction = 1
output_yaw = 0
InitMin = True
InitOdom = True
min_distance = 1
total_distance = 1
odom_roll = odom_pitch = odom_yaw = 0


def PIDinit():
    global pid_x, pid_y, pid_z, pid_yaw
    P_x = 0.1 
    I_x = 1.0 
    D_x = 0.0

    P_y = 1.8
    I_y = 0.0 
    D_y = 0.1

    P_z = 2.8
    I_z = 0.0 
    D_z = 0.5

    P_yaw = 1.2
    I_yaw = 0.0
    D_yaw = 0.0

    pid_x = PID.PID(P_x, I_x, D_x)
    pid_y = PID.PID(P_y, I_y, D_y)
    pid_z = PID.PID(P_z, I_z, D_z)
    pid_yaw = PID.PID(P_yaw, I_yaw, D_yaw)

    #控制频率根据不同的node的速率来设定，ukf和odom的反馈都能达到200HZ，但是PID控制周期是依据OFFBOARD刷新频率确定的也就是100HZ
    pid_x.setSampleTime(0.01)
    pid_y.setSampleTime(0.01)
    pid_z.setSampleTime(0.01)
    pid_yaw.setSampleTime(0.01)

def callback_ukf(ukfpose):
    global ukf_x, ukf_y, ukf_z, init_yaw, InitMin, total_distance, min_distance, dyaw
    ukf_x = ukfpose.pose.pose.position.x
    ukf_y = ukfpose.pose.pose.position.y
    ukf_z = ukfpose.pose.pose.position.z


    left_line_distance = ukfpose.pose.pose.orientation.x  #暂时用来放laser_left，这个和真实含义不对等
    right_line_distance = ukfpose.pose.pose.orientation.y   #暂时用来放laser_right，这个和真实含义不对等
    init_yaw = ukfpose.pose.pose.orientation.z   #暂时用来放init_yaw，这个和真实含义不对等 
 
    #use total distance to represent the flying y-z plain
    total_distance = right_line_distance + left_line_distance
    
    #print right_line_distance, left_line_distance
    #print 'total_distance', total_distance
    while InitMin == True:
        min_distance = total_distance
        InitMin = False
        print "Min_distance 初始化成功：",min_distance

    #dyaw = total_distance - min_distance
    d_distance = total_distance - min_distance
    # minimize the theta to let the plain getting close to the right 
    cosyaw = min_distance / total_distance
    # if cosyaw <= 1 and cosyaw >=-1:
    #     dyaw = np.arccos(min_distance / total_distance )
    # same reselt better flash rate 
    dyaw = abs(cosyaw - 1)
    #print dyaw
    #设定一个阀值，让小数据变化不会被错认为面改变
    if d_distance < -0.02:
        min_distance = total_distance
        print "NARROW FOUND !",total_distance
    #遇到数据突变说明隧道进入了另外一个部分，变宽 单位-米
    if d_distance > 2.0:
       min_distance = total_distance  
       print "WIDE FOUND !",total_distance    




def callback_odom(odom):
    global odom_roll,odom_pitch,odom_yaw, InitOdom
    qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    (odom_roll,odom_pitch,odom_yaw) = euler_from_quaternion(qn_odom)


def callback_pid(pose):
    global ukf_x, ukf_y, ukf_z, pid_x, pid_y, pid_z, pid_yaw, feedback_x, feedback_y ,feedback_z, feedback_yaw, last_feedback_yaw, d_dyaw, dd_dyaw, direction
    global output_yaw,total_distance, odom_roll,odom_pitch,odom_yaw, init_yaw
    vel_control = TwistStamped()
    pos_control = PoseStamped()

    scale_z = 0.08
    scale_y = 0.4
    scale_yaw = 1

    scale_yaw = scale_yaw / total_distance

    pid_y.update(feedback_y)
    output_y = pid_y.output * scale_y
    feedback_y = ukf_y
    pid_y.SetPoint = pose.pose.position.y

    pid_z.update(feedback_z)
    output_z = pid_z.output * scale_z
    feedback_z = ukf_z
    pid_z.SetPoint = pose.pose.position.z

    feedback_yaw = odom_yaw
    if feedback_yaw - last_feedback_yaw != 0: 
        pid_yaw.update(feedback_yaw)
        output_yaw = pid_yaw.output * scale_yaw * direction
        #pid_yaw.SetPoint = pose.pose.orientation.z
        pid_yaw.SetPoint = dyaw + feedback_yaw

        #print  " yaw_set:", pid_yaw.SetPoint," yaw_out:", output_yaw, " yaw_Feb:", feedback_yaw, "dfb:",abs(feedback_yaw) - abs(last_feedback_yaw)
    dfeedback = abs(feedback_yaw) - abs(last_feedback_yaw)    
    if dfeedback < 0:
        d_dyaw = []
        dd_dyaw = []
    elif dfeedback > 0:
        d_dyaw.append(dfeedback)
        print  "d_dyaw", d_dyaw
        if len(d_dyaw) == 3:
            if  d_dyaw[2]>0 and d_dyaw[1] > 0 and d_dyaw[0] > 0:
                dd_dyaw.append(d_dyaw[2] - d_dyaw[1])
                dd_dyaw.append(d_dyaw[1] - d_dyaw[0])
                print  "dd_dyaw", dd_dyaw
                if  dd_dyaw[1] >0 and dd_dyaw[0] > 0:
                    direction = - direction
                    print "CHANGE!!!!!!!!"
            dd_dyaw = []
            d_dyaw = [] 
            # print  "dd_dyaw", dd_dyaw
            # #print dd_dyaw[1] - dd_dyaw[0]
            # if dd_dyaw[1] - dd_dyaw[0] > 0:
            #     direction = - direction
            #     print "CHANGE!!!!!!!!"
            # dd_dyaw = []
            # d_dyaw = [] 

    last_feedback_yaw = feedback_yaw


    #用速度限制防止无人机突然起降导致的不稳定
    if output_y > 0.02:
        output_y = 0.02
    elif output_y < -0.02:
        output_y = -0.02

    if output_z > 0.05:
        output_z = 0.05
    elif output_z < -0.05:
        output_z = -0.05

    setpoint = Vector3()
    setpoint.x = 0
    setpoint.y = output_y
    setpoint.z = output_z
    stamp = rospy.get_rostime()
    #暂时屏蔽yaw控制和y控制
    msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                             type_mask=PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                       PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE,
                             velocity=setpoint, 
                             #加不加初始角还不确定，加初始角容易出现固定偏差，减去pi/2是因为会发生偏转
                             yaw = feedback_yaw + output_yaw)
    msg.header.stamp = stamp
    position_pub.publish(msg)


    # #vel_control.twist.linear.x = 0
    # vel_control.twist.linear.y = output_y
    # vel_control.twist.linear.z = output_z
    # #vel_control.twist.angular.z = 0
    # pub_velocity.publish(vel_control)

    # (x,y,z,w) = quaternion_from_euler(odom_roll, odom_pitch, feedback_yaw + output_yaw)
    # pos_control.pose.orientation.x = x
    # pos_control.pose.orientation.y = y
    # pos_control.pose.orientation.z = z
    # pos_control.pose.orientation.w = w
    # #pub_position.publish(pos_control)

    print  "z_set:", pid_z.SetPoint, "z_out:", output_z, "z_Feb:", feedback_z
    print  "y_set:", pid_y.SetPoint, "y_out:", output_y, "y_Feb:", feedback_y

rospy.init_node('pid_control')
PIDinit()
position_pid = rospy.Subscriber('/setpoint_PID', PoseStamped, callback_pid)
position_ukf = rospy.Subscriber('/position_ukf', Odometry, callback_ukf, queue_size=1)
subodom = rospy.Subscriber('/mavros/local_position/odom', Odometry, callback_odom)
#pub_position = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
#pub_velocity = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
position_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
rospy.spin()