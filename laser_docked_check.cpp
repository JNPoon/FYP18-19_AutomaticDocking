#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "tf/tfMessage.h"
#include "tf/transform_listener.h"
#include <stdlib.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <ros/xmlrpc_manager.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include "ros/time.h"
#include <sensor_msgs/LaserScan.h>
 

void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int check = 0;
  double mid_pt_dist = msg->ranges[msg->ranges.size()/2 - 0.5];
  double single_error = sqrt( pow(msg->ranges[0],2) + pow(msg->ranges[1],2) - 
    2*msg->ranges[0]*msg->ranges[1]*cos(msg->angle_increment));

  for (int i=0, n=(msg->ranges.size()/2-0.5); i < (msg->ranges.size()/2-0.5); i++, n--)
  {
    double lower_half = msg->ranges[i]*cos(n*msg->angle_increment);
    double upper_half = msg->ranges[msg->ranges.size()-i-1]*cos(n*msg->angle_increment);
    double tolerance = 0.015;

    if ( (upper_half <= (mid_pt_dist + tolerance) && upper_half >= (mid_pt_dist - tolerance) ) && 
      (lower_half <= (mid_pt_dist + tolerance) && lower_half >= (mid_pt_dist - tolerance) ) )
    {
      check += 1; //if check equal 16 mean it's perfectly docked, if check > 9 then the orientation is ok!
    }

    // ROS_INFO("%d element after cos: %lf", i, msg->ranges[i]*cos(n*msg->angle_increment));
    // ROS_INFO("first element: %lf", msg->ranges[0]);
    // ROS_INFO("second element: %lf", msg->ranges[1]);
    // ROS_INFO("mid point distance: %lf", mid_pt_dist);
    // ROS_INFO("%d element after cos: %lf", msg->ranges.size()-i-1, msg->ranges[msg->ranges.size()-i-1]*cos(n*msg->angle_increment));
  }

  double error = (check - 16) * single_error * 100; //convert into cm

  ROS_INFO("check: %d", check);
  ROS_INFO("error: %lf", error);
  //ROS_INFO("%d", msg->ranges.size());

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Laser_Data");
  ros::NodeHandle nh;
  ros::Subscriber pose= nh.subscribe("scan1",50,&LaserCallBack);
  ros::spin();
}