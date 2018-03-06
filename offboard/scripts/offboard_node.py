#!/usr/bin/python2.7

import rospy 
import roslib
import numpy
import tf
import laser_geometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import CommandBool
from mavros_msgs.msg import SetMode
from mavros_msgs.msg import State
from tf.transformations import quaternion_from_euler, euler_from_quaternion



rospy.init_node('offboard_node')

rospy.wait_for_service('mavros/cmd/arming')
rospy.wait_for_service('mavros/set_mode')

arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

offb_set_mode = SetMode() 
offb_set_mode.request.custom_mode = "OFFBOARD"

arm_cmd = CommandBool() 
arm_cmd.request.value = True

last_request = rospy.Time.now()
last_request1 = rospy.Time.now()

###用C++###
subcloud = rospy.Subscriber('/point_cloud', PointCloud, callback_cloud)
sublaser = rospy.Subscriber('/scan_filtered', LaserScan, callback_laser)
subodom = rospy.Subscriber('/mavros/local_position/odom', Odometry, callback_odom)
subogazebo = rospy.Subscriber('/gazebo/model_states', ModelStates, callback_gazebo)
pub = rospy.Publisher('/position_lidar', PoseStamped, queue_size=10)

rospy.spin()
