#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8

# Example of vel contorl and position control

    # msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
    #                          type_mask=PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
    #                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
    #                                    PositionTarget.IGNORE_YAW_RATE,
    #                          velocity=set_velocity, 
    #                          yaw = set_yaw )

    # msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
    #                          type_mask=PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
    #                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
    #                                    PositionTarget.IGNORE_YAW_RATE,
    #                          position=set_position, 
    #                          yaw = set_yaw )

from __future__ import division
import rospy 
import roslib
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget, AttitudeTarget, State
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, PointStamped, Vector3, Vector3Stamped, TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

counter = 500000 #50hz 8*60*60*50
r = 8
theta = 0
set_position = Point()
set_position.x = 0
set_position.y = 0
set_position.z = 0
set_yaw = 0
wn = 0.03
setvel = Vector3()
msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                            type_mask=PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                                    PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                    PositionTarget.IGNORE_YAW_RATE,
                            position=set_position, 
                            yaw = set_yaw )

def callback_odom(pose):
    global r, counter,theta,msg, set_position,set_yaw,wn,setvel
    
    print 'looping:', counter

    if counter < 500000 and counter > 450000:
    # random fly
        
        if counter % 1 == 0:
            setvel.x = 0
            setvel.y = 0
            setvel.z += 0.001
        print "vel fly ",'set: vel_x',setvel.x, 'set: vel_y',setvel.y, 'set: vel_z',setvel.z
        msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                             type_mask=PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ +
                                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                       PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE,
                             velocity=setvel )
        stamp = rospy.get_rostime()
        msg.header.stamp = stamp
        position_pub.publish(msg)


    if counter == 0:
        print 'Flying test over hold still at 1 1 1'
        set_position.x = 1
        set_position.y = 1
        set_position.z = 1
        set_yaw = 0
        msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                                type_mask=PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ +
                                        PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                        PositionTarget.IGNORE_YAW_RATE,
                                position=set_position, 
                                yaw = set_yaw )
        stamp = rospy.get_rostime()
        msg.header.stamp = stamp
        position_pub.publish(msg)

    counter = counter -1


rospy.init_node('fly_control')

subodom = rospy.Subscriber('/mavros/local_position/odom', Odometry, callback_odom)
position_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)

rospy.spin()
