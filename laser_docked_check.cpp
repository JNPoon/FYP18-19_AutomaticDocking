#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "docking/FinalCheck.h"
#include <math.h>
#include <sensor_msgs/LaserScan.h>

bool g_CheckPointReached = 0;
bool g_MovingBack = 0;
bool g_Docked = 0;

ros::Publisher pub_reverse;

void FinalCheckCallBack(const docking::FinalCheck::ConstPtr& check)
{
  g_CheckPointReached = 0; //Reset to 0
  g_MovingBack = 0; //Reset to 0
  if (check->CheckPointReached == 1) g_CheckPointReached = 1;
  if (check->MovingBack == 1) g_MovingBack = 1;
}

void ChargingVoltageCallBack(const std_msgs::Float32::ConstPtr& chargingvoltage)
{
  if(chargingvoltage->data>2.6)
  {
    g_Docked = 1;
    // ROS_WARN("Docking DONE! Start Charging...");
  }
  else if(chargingvoltage->data<2.5)
  {
    g_Docked = 0;
  }
}

void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  docking::FinalCheck reverse;
  double mid_pt_dist = msg->ranges[msg->ranges.size()/2 - 0.5];
  double single_error = sqrt( pow(msg->ranges[0],2) + pow(msg->ranges[1],2) -
    2*msg->ranges[0]*msg->ranges[1]*cos(msg->angle_increment));

  if (g_CheckPointReached == 1)
  {
    int orient = 0; //when compare mid_pt_dist with cos(other scan dist), ++ when within tolerance
    int orient_value = 16; //a value set to compare with orient

    for (int i=0, n=(msg->ranges.size()/2-0.5); i < (msg->ranges.size()/2-0.5); i++, n--)
    {
      double lower_half = msg->ranges[i]*cos(n*msg->angle_increment);
      double upper_half = msg->ranges[msg->ranges.size()-i-1]*cos(n*msg->angle_increment);
      double tolerance = 0.015; //the tolerance of cos(other scan dist) with respect to mid_pt_dist

      if ( (upper_half <= (mid_pt_dist + tolerance) && upper_half >= (mid_pt_dist - tolerance) ) &&
        (lower_half <= (mid_pt_dist + tolerance) && lower_half >= (mid_pt_dist - tolerance) ) )
      {
        orient += 1; //if check equal 16 mean it's perfectly docked, if check > 9 then the orientation is ok!
      }

      // ROS_INFO("%d element after cos: %lf", i, msg->ranges[i]*cos(n*msg->angle_increment));
      // ROS_INFO("first element: %lf", msg->ranges[0]);
      // ROS_INFO("second element: %lf", msg->ranges[1]);
      // ROS_INFO("mid point distance: %lf", mid_pt_dist);
      // ROS_INFO("%d element after cos: %lf", msg->ranges.size()-i-1, msg->ranges[msg->ranges.size()-i-1]*cos(n*msg->angle_increment));
    }

    // double error = (check - 16) * single_error * 100; //calculate shift in x direction and convert into cm

    if (orient < orient_value)
    {
      reverse.ReverseCheck = 1;
      pub_reverse.publish(reverse);
      ROS_WARN("Orientation not correct at CheckPoint!! Reversing....");
    }

    // ROS_INFO("check: %d", check);
    // ROS_INFO("error: %lf", error);
  }

  if (g_MovingBack == 1)
  {
    double range = 0.15;

    if (mid_pt_dist < 0.15 && g_Docked == 0)
    {
      reverse.ReverseCheck = 1;
      pub_reverse.publish(reverse);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Laser_Data");
  ros::NodeHandle nh;
  pub_reverse = nh.advertise<docking::FinalCheck>("FinalCheck",1);
  ros::Subscriber sub_Laser = nh.subscribe("scan1",50,&LaserCallBack);
  ros::Subscriber sub_FinalCheck = nh.subscribe("FinalCheck",10,&FinalCheckCallBack);
  ros::Subscriber sub_ChargingVoltage = nh.subscribe("chargingvoltage",1000,&ChargingVoltageCallBack);
  ros::spin();
}
