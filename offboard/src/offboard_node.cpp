#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>


//INSTRUCTION：
//Using STABLIZE mode to start, then the program will auto switch to OFFBOARD and then ARM the drone.
//IF the OFFBOARD lost control, switch to MANUAL to re-control the drone or just kill the rotor.
//Attention! set_point should start before switch to OFFBOARD.

mavros_msgs::State current_state;
char swithc_sign = 0;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // ros::Publisher setpoint_PID = nh.advertise<geometry_msgs::PoseStamped>
    //         ("setpoint_PID", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    //ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            //("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.pose.orientation.z = 0;
    
    
    // geometry_msgs::TwistStamped vel;
    // vel.twist.linear.x = 0;
    // vel.twist.linear.y = 0;
    // vel.twist.linear.z = 0.1;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::SetMode manu_set_mode;
    manu_set_mode.request.custom_mode = "MANUAL";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time last_request1 = ros::Time::now();

    while(ros::ok()){
	//ROS_INFO_STREAM("HEY \"" << swithc_sign);
	//ROS_INFO_STREAM("HEY \"" << current_state.mode);
	if(swithc_sign == 0)
	{

        if( current_state.mode == "OFFBOARD" && !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
            
    }
    if ( current_state.mode == "MANUAL" && swithc_sign!=2)
    {
        swithc_sign = 1;
	    if(swithc_sign == 1)
        {
	        if( set_mode_client.call(manu_set_mode) && manu_set_mode.response.mode_sent)
            {
                ROS_INFO("manu_set_mode enabled");
	            swithc_sign=2;
            }
        }
            ROS_INFO("Manual control enabled");
    }
        // if(ros::Time::now() - last_request1 > ros::Duration(9999999999)){           
        //     //pose.pose.position.x += 0.01;
        //     pose.pose.position.y -= 1;
        //     //pose.pose.position.z += 0.01;
        //     //if(pose.pose.position.x >= 3) pose.pose.position.x = 0.5; 
        //     if(pose.pose.position.y < -1) pose.pose.position.y = 1; 
        //     //if(pose.pose.position.z >= 2) pose.pose.position.z = 1; 
        //     last_request1 = ros::Time::now();
            
        //     }
            

        //ROS_INFO_STREAM("HEY \"" << 123);
        //local_pos_pub.publish(pose);
        //local_vel_pub.publish(vel);
        //setpoint_PID.publish(pose);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
