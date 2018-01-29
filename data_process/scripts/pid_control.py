#!/home/eleboss/anaconda3/envs/py27/bin/python
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
from laser_line_extraction.msg import LineSegment
from laser_line_extraction.msg import LineSegmentList


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
init_yaw = 0


def PIDinit():
    global pid_x, pid_y, pid_z, pid_yaw
    P_x = 0.1 
    I_x = 1.0 
    D_x = 0.0

    P_y = 1.8
    I_y = 0.0 
    D_y = 0.1

    P_z = 3.0 
    I_z = 0.0 
    D_z = 1.6

    P_yaw = 1.2
    I_yaw = 0.0
    D_yaw = 0.0

    pid_x = PID.PID(P_x, I_x, D_x)
    pid_y = PID.PID(P_y, I_y, D_y)
    pid_z = PID.PID(P_z, I_z, D_z)
    pid_yaw = PID.PID(P_yaw, I_yaw, D_yaw)

    pid_x.setSampleTime(0.01)
    pid_y.setSampleTime(0.01)
    pid_z.setSampleTime(0.01)
    pid_yaw.setSampleTime(0.01)

def callback_ukf(ukfpose):
    global ukf_x, ukf_y, ukf_z, dyaw
    ukf_x = ukfpose.pose.pose.position.x
    ukf_y = ukfpose.pose.pose.position.y
    ukf_z = ukfpose.pose.pose.position.z

    #dyaw = ukfpose.pose.pose.orientation.z

def callback_odom(odom):
    global odom_roll,odom_pitch,odom_yaw, init_yaw, InitOdom
    qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    (odom_roll,odom_pitch,odom_yaw) = euler_from_quaternion(qn_odom)

    while InitOdom == True:
        init_yaw = odom_yaw
        InitOdom = False
        print "Yaw 初始化成功：",init_yaw


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
    output_z = - pid_z.output * scale_z
    feedback_z = ukf_z
    pid_z.SetPoint = pose.pose.position.z

    feedback_yaw = odom_yaw
    if feedback_yaw - last_feedback_yaw != 0: 
        pid_yaw.update(feedback_yaw)
        output_yaw = pid_yaw.output * scale_yaw * direction
        #pid_yaw.SetPoint = pose.pose.orientation.z
        pid_yaw.SetPoint = dyaw + feedback_yaw

        print  " yaw_set:", pid_yaw.SetPoint," yaw_out:", output_yaw, " yaw_Feb:", feedback_yaw, "dfb:",abs(feedback_yaw) - abs(last_feedback_yaw)
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

    setpoint = Vector3()
    setpoint.x = 0
    setpoint.y = output_y
    setpoint.z = output_z
    stamp = rospy.get_rostime()
    msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                             type_mask=PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                       PositionTarget.IGNORE_YAW_RATE,
                             velocity=setpoint, 
                             yaw = feedback_yaw + output_yaw - np.pi/2 )
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

    #print  "z_set:", pid_z.SetPoint, "z_out:", output_z, "z_Feb:", feedback_z
    #print  "y_set:", pid_y.SetPoint, "y_out:", output_y, "y_Feb:", feedback_y

def callback_line(line):
    global InitMin, total_distance, min_distance,dyaw, odom_roll,odom_pitch,odom_yaw
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
    gardient = np.arctan(abs(gardient))
    #print gardient
    gardient = np.square(gardient - (odom_roll + np.pi/2))
    #print gardient
    
    #print "gar",gardient,np.shape(gardient)
    for i in range(len(gardient)):
        #judge the left or right surface
        if gardient[i] < 0.02:
            if abs(angle[i]) > 1.57:
                left_surface_start = np.vstack((left_surface_start , start[i,:]))
                left_surface_end = np.vstack((left_surface_end,end[i,:]))
                left_surface_angle = np.vstack((left_surface_angle,angle[i]))

            else:
                right_surface_start = np.vstack((right_surface_start,start[i,:]))
                right_surface_end = np.vstack((right_surface_end,end[i,:]))
                right_surface_angle = np.vstack((right_surface_angle,angle[i]))

    left_surface_start = np.delete(left_surface_start,0,0) 
    left_surface_end = np.delete(left_surface_end,0,0) 
    left_surface_angle = np.delete(left_surface_angle,0,0) 
    right_surface_start = np.delete(right_surface_start,0,0) 
    right_surface_end = np.delete(right_surface_end,0,0) 
    right_surface_angle = np.delete(right_surface_angle,0,0) 

    #print "left start"  ,left_surface_start
    #print "left end"  ,left_surface_end
    #print "left angle"  ,left_surface_angle
    #print "right start"  ,right_surface_start
    #print "right start"  ,right_surface_end
    #print "right angle"  ,right_surface_angel

    left_line_length =  np.sqrt(np.square(left_surface_start - left_surface_end)[:,0] + np.square(left_surface_start - left_surface_end)[:,1])
    right_line_length =  np.sqrt(np.square(right_surface_start - right_surface_end)[:,0] + np.square(right_surface_start - right_surface_end)[:,1])
    #print "left_line_length",left_line_length
    #print "right_line_length",right_line_length
    pLeftLongest = np.argmax(left_line_length)
    pRightLongest = np.argmax(right_line_length)
    total_distance = abs((left_surface_start[pLeftLongest, 0] + left_surface_end[pLeftLongest, 0]) / 2) + abs((right_surface_start[pRightLongest, 0] + right_surface_end[pRightLongest, 0]) / 2) 
    #print total_distance
    while InitMin == True:
        min_distance = total_distance
        InitMin = False
        print "Min_distance 初始化成功：",min_distance

    #dyaw = total_distance - min_distance
    d_distance = total_distance - min_distance
    cosyaw = min_distance / total_distance
    if cosyaw <= 1 and cosyaw >=-1:
        dyaw = np.arccos(min_distance / total_distance )
    #print dyaw
    if d_distance < -0.02:
        min_distance = total_distance
        print "NARROW FOUND !"
    #遇到数据突变说明隧道进入了另外一个部分，变宽
    if d_distance > 1.5:
       min_distance = total_distance  
       print "WIDE FOUND !"          



    


rospy.init_node('pid_control')
PIDinit()
position_pid = rospy.Subscriber('/setpoint_PID', PoseStamped, callback_pid)
position_ukf = rospy.Subscriber('/position_ukf', Odometry, callback_ukf)
subodom = rospy.Subscriber('/mavros/local_position/odom', Odometry, callback_odom)
seg_line = rospy.Subscriber('/line_segments', LineSegmentList, callback_line)
#pub_position = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
#pub_velocity = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
position_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
rospy.spin()