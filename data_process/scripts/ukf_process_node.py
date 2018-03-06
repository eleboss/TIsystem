#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8

###############UKF参数选取和系统方程测定方法##############
#参数选取原则  
#首先按照动力学方程建立F方程  这里我使用的是匀速运动方程，因为我们的巡逻系统的设计是用来匀速巡检，不是一直变化速度的竞赛，所以不引入加速度不会有很大的影响。 S（i+1） = S（i） + V*dt
#F要和H相乘，H=【sy,vy,sz,vz】，所以写为下面的形式
#用H来选择有几个输入参数，这里我有四个参数，分别是y位置y速度,z位置z速度，xy轴的数据已经在之前的融合做完了所以这里H直接返回四个参数。  
#dim_x = 4， 表示输入方程有四个输入  
#dim_z = 4 表示观测方程z有四个数  
#std_y, std_z，表示yz两轴的测量误差，我感觉激光雷达应该有3cm的精度，所以用了0.03  
#vstd表示速度测量误差，我写了个0.2发现结果还不错，但是感觉误差应该没那么大，但是速度测量结果其实不可信，所以写高了结果就不太好  
#Q噪声项，var这里我发现用0.2的结果比较好，目前还不知道为什么  
#P里头分别填上前面y位置y速度，z位置z速度能达到的最大值。  
#ukf.x = np.array([0., 0., 0., 0.])表示起始位置和起始速度都是0  
#######################################################
#数据融合方法说明：
#这里使用的数据融合方法是这样的：
#首先激光雷达更新数据（较低频率）30HZ
#然后在两个激光雷达的数据帧中间里程计会以较高的频率更新数据200HZ
#这样就可以用激光雷达的位置作为初始位置，中间用里程计作积分得到位置累加上去。
#最后融合的数据就能达到200HZ


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