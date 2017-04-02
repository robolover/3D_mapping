#include <ros/ros.h>
#include <string>
#include <sstream>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <laser_assembler/AssembleScans2.h>
#include <msg/laser_assemble.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Time           g_lidar_move_start_time;

ros::Publisher      g_point_cloud2_pub;

ros::Subscriber     g_lidar_turn_start_sub;
ros::Subscriber     g_lidar_turn_end_sub;

ros::ServiceClient  g_assemble_chest_laser_client;

typedef pcl::PointXYZ PointT;

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void assembleLaserScans(ros::Time before_time, ros::Time end_time)
{
  ros::Time now = ros::Time::now();

  laser_assembler::AssembleScans2 service;
  service.request.begin = before_time;
  service.request.end = end_time;

  if (g_assemble_chest_laser_client.call(service))
  {
    ros::Time assemble_time = ros::Time::now();
    sensor_msgs::PointCloud2 assembler_output = service.response.cloud;
    if (assembler_output.data.size() == 0)
    {
      // ROS_INFO("No scan data");
      return;
    }

    ROS_INFO("  ---  publish pointcloud data!!  ---  %f", (ros::Time::now() - assemble_time).toSec());

    g_point_cloud2_pub.publish(assembler_output);
  }
}

void lidarTurnCallBack(const msg::laser_assemble::ConstPtr& laser_pub)
{
  ros::Time now = ros::Time::now();

  if (laser_pub->msg == 1)
  {
    g_lidar_move_start_time = now;
  }
  else if (laser_pub->msg == 2)
  {
    // assemble laser
    assembleLaserScans(g_lidar_move_start_time, now);
    g_lidar_move_start_time = now;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thor_lidar_assembler");
  ros::NodeHandle nh;

  // Add your ros communications here.
  g_lidar_turn_end_sub          = nh.subscribe("laser_scan", 1, &lidarTurnCallBack);
  g_point_cloud2_pub            = nh.advertise<sensor_msgs::PointCloud2>("assembler_output", 0);
  g_assemble_chest_laser_client = nh.serviceClient<laser_assembler::AssembleScans2>("/assemble_scans2");

  g_lidar_move_start_time = ros::Time::now();

  ros::spin();

  return 0;
}
