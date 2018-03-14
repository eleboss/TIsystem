#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8

from __future__ import division
import rospy 
import roslib
import time
import numpy as np
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget, AttitudeTarget, State
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, PointStamped, Vector3, Vector3Stamped, TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

counter = 500000 #50hz 8*60*60*50
#use to set pid pisition 
set_position = PoseStamped()
set_position.pose.position.x = 0
set_position.pose.position.y = 0
set_position.pose.position.z = 0
set_yaw = 0

TAKE_OFF = True
HOLD = False
LAND = False

#current mode
current_mode = ''

#parameter of ukf callback
ukf_x = ukf_y = ukf_z = init_yaw = left_line_distance = right_line_distance = top = 0
INIT_TOP = True

def callback_state(state):
    global current_mode
    current_mode = state.mode

def callback_ukf(ukfpose):
    global ukf_x, ukf_y, ukf_z, init_yaw, left_line_distance, right_line_distance, INIT_TOP, set_position, top
    ukf_x = ukfpose.pose.pose.position.x
    ukf_y = ukfpose.pose.pose.position.y
    ukf_z = ukfpose.pose.pose.position.z

    left_line_distance = ukfpose.pose.pose.orientation.x  
    right_line_distance = ukfpose.pose.pose.orientation.y   
    init_yaw = ukfpose.pose.pose.orientation.z   

    if INIT_TOP == True:
        set_position.pose.position.z = ukf_z
        top = ukf_z
        print '距离顶部高度初始化成功，当前距离', top
        INIT_TOP = False

def callback_odom(pose):
    global counter, set_position,set_yaw, current_mode, TAKE_OFF, HOLD, LAND
    if current_mode == 'OFFBOARD':
        # when set to offboard, start the control procedure
        print 'Detect OFFBOARD, start looping:', counter
        if counter < 500000 and counter > 0:
            #take off
            if counter % 200 == 0 and TAKE_OFF == True:
                set_position.pose.position.x = 0
                set_position.pose.position.y = 0
                set_position.pose.position.z += 0.03
                set_yaw = 0
                if set_position.pose.position.z >= top + 0.3:
                    TAKE_OFF = False
                    HOLD = True
            elif counter % 200 == 0 and HOLD == True:
                set_position.pose.position.x = 0
                set_position.pose.position.y = 0
                set_position.pose.position.z = top + 0.3
                set_yaw = 0
                if counter % 2000 == 0:
                    HOLD = False
                    LAND = True
            elif counter % 200 == 0 and LAND == True:
                set_position.pose.position.x = 0
                set_position.pose.position.y = 0
                set_position.pose.position.z -= 0.02
                set_yaw = 0
            
            print "STATE:",'X:',TAKE_OFF,HOLD,LAND,set_position.pose.position.x,'Y:',set_position.pose.position.y,'Z:',set_position.pose.position.z,top
            stamp = rospy.get_rostime()
            set_position.header.stamp = stamp
            position_pub.publish(set_position)


        if counter == 0:
            print 'Flying test over hold still at 1 1 1'
            set_position.z = 0

            stamp = rospy.get_rostime()
            set_position.header.stamp = stamp
            position_pub.publish(set_position)

        counter = counter -1
    else:
        # continuous set vel for OFFBOARD switch
        setvel = Vector3()
        setvel.x = 0
        setvel.y = 0
        setvel.z = 0
        stamp = rospy.get_rostime()
        msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                             type_mask=PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                       PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE,
                             velocity=setvel )
        msg.header.stamp = stamp
        raw_pub.publish(msg)
        print 'No OFFBOARD, current mode is:', current_mode, 'set: vel_x',setvel.x, 'set: vel_y',setvel.y, 'set: vel_z',setvel.z

rospy.init_node('fly_control')

subodom = rospy.Subscriber('/mavros/local_position/odom', Odometry, callback_odom)
position_ukf = rospy.Subscriber('/position_ukf', Odometry, callback_ukf)
substate = rospy.Subscriber('mavros/state', State, callback_state)
position_pub = rospy.Publisher('/setpoint_PID', PoseStamped, queue_size=1)
raw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

rospy.spin()