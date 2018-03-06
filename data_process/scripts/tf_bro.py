#!/usr/bin/python2.7
import roslib  
#roslib.load_manifest('learning_tf')  
import rospy   
import tf  
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler, euler_from_quaternion

ukf_x = ukf_y = ukf_z = 0

def handle_pose(msg, dronename): 
    global ukf_x, ukf_y, ukf_z
    #qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    #(odom_roll,odom_pitch,odom_yaw) = euler_from_quaternion(qn_odom) 

    br = tf.TransformBroadcaster()  
    br.sendTransform((ukf_z, ukf_y, ukf_x), #the translation of the transformtion as a tuple (x, y, z)  
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),   #the rotation of the transformation as a tuple (x, y, z, w)  
                     rospy.Time.now(), #the time of the transformation, as a rospy.Time()  
                     dronename, #child frame in tf, string  
                     "world") #parent frame in tf, string  

def callback_ukf(ukfpose):
    global ukf_x, ukf_y, ukf_z
    ukf_x = ukfpose.pose.pose.position.x
    ukf_y = ukfpose.pose.pose.position.y
    ukf_z = ukfpose.pose.pose.position.z

if __name__ == '__main__':  
    rospy.init_node('turtle_tf_broadcaster')  
    dronename = rospy.get_param('~drone')    
                     #takes parameter "turtle", which specifies a turtle name, e.g. "turtle1" or "turtle2"   
    rospy.Subscriber('/mavros/local_position/odom',  # subscribe to turtlename's /pose topic  
                     Odometry,             # Pose message data structure  
                     handle_pose,                    # callback function,  
                     dronename)                      # argument for callback function

    position_ukf = rospy.Subscriber('/position_ukf', Odometry, callback_ukf)

    rospy.spin()  