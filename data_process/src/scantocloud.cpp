#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  ros::Publisher scan_pub_ori;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "scan_filtered", 10),
    laser_notifier_(laser_sub_,listener_, "rplidar_link", 10)  //use the lidar link name
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/point_cloud",1);
    scan_pub_ori = n_.advertise<sensor_msgs::PointCloud>("/point_cloud_ori",1);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::PointCloud cloud_ori;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "world",*scan_in, cloud,listener_);  // use the transfromed name, in tf_bor we trans  rplidar_link to world link
        projector_.transformLaserScanToPointCloud(
          "world",*scan_in, cloud_ori,listener_);  // use the transfromed name, in tf_bor we trans  rplidar_link to world link
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
    
    // Do something with cloud.

    scan_pub_.publish(cloud);
    scan_pub_ori.publish(cloud_ori);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "scantocloud_node");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;
}