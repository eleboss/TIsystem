#!/home/eleboss/anaconda3/envs/py27/bin/python
# coding=<encoding name> 例如，可添加# coding=utf-8


import rospy 
import roslib
import pickle
from numpy.random import randn
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import unscented_transform, MerweScaledSigmaPoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

ukf_result = []

def f_cv(x, dt):
    """ state transition function for a 
    constant velocity aircraft"""
    
    F = np.array([[1, dt, 0,  0],
                  [0,  1, 0,  0],
                  [0,  0, 1, dt],
                  [0,  0, 0,  1]],dtype=float)
    return np.dot(F, x)

def h_cv(x):
    return np.array([x[0], x[1], x[2], x[3]])

def UKFinit():
    global ukf
    ukf_fuse = []
    std_y, std_z = 0.03, 0.03
    vstd_y = 0.2
    vstd_z = 0.2
    dt = 0.005


    sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=-1.0)
    ukf = UKF(dim_x=4, dim_z=4, fx=f_cv, hx=h_cv, dt=dt, points=sigmas)
    ukf.x = np.array([0., 0., 0., 0.])
    ukf.R = np.diag([std_y, vstd_y,std_z,vstd_z]) 
    ukf.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=0.2)
    ukf.Q[2:4, 2:4] = Q_discrete_white_noise(2, dt=dt, var=0.2)
    ukf.P = np.diag([4**2, 1**2, 4**2, 1**2])


def callback_fuse(fuse):
    global ukf_result,ukf
    position_ukf = Odometry()

    fuse_x = fuse.pose.pose.position.x
    fuse_y = fuse.pose.pose.position.y
    fuse_z = fuse.pose.pose.position.z

    dyaw = fuse.pose.pose.orientation.z 

    odom_velx = fuse.twist.twist.linear.x 
    odom_vely = fuse.twist.twist.linear.y 
    odom_velz = fuse.twist.twist.linear.z

    odom_sec = fuse.header.stamp.secs
    odom_nsec = fuse.header.stamp.nsecs
    fuse_time = odom_sec + odom_nsec * 10**(-9)

    position_fuse = [fuse_y, odom_vely, fuse_z, odom_velz]

    ukf.predict()
    ukf.update([position_fuse[0], position_fuse[1], position_fuse[2], position_fuse[3]])

#记录数据，发布数据
    #ukf_result.append(np.hstack((fuse_time, ukf.x)))

    position_ukf.header.frame_id = "rplidar_link"
    position_ukf.header.stamp.secs = odom_sec
    position_ukf.header.stamp.nsecs = odom_nsec

    position_ukf.pose.pose.position.x = fuse_x
    position_ukf.pose.pose.position.y = ukf.x[0]
    position_ukf.pose.pose.position.z = ukf.x[2]

    position_ukf.pose.pose.orientation.z = dyaw

    position_ukf.twist.twist.linear.x = odom_velx
    position_ukf.twist.twist.linear.y = ukf.x[1]
    position_ukf.twist.twist.linear.z = ukf.x[3]

    pub.publish(position_ukf)
#print uxs


rospy.init_node('ukf_process_node')
UKFinit()
position_fusion = rospy.Subscriber('/position_fusion', Odometry, callback_fuse)
pub = rospy.Publisher('/position_ukf', Odometry, queue_size=0)
rospy.spin()