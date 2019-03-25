#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "docking/FinalCheck.h"
#include <math.h>
#include <sensor_msgs/LaserScan.h>

//-------------------------------------------------------
//GLOBAL VARIABLE
//-------------------------------------------------------
bool g_CheckPointReached = 0;
bool g_MovingBack = 0;
bool g_Docked = 0;
// double g_max_upper = 0;
// double g_min_upper = 0;
// double g_max_lower = 0;
// double g_min_lower = 0;

//-------------------------------------------------------
//PARAM VARIABLE
//-------------------------------------------------------
int p_OrientValue = 16; //a reference to compare with orient (set by user)
double p_Tolerance = 0.016; //the tolerance of cos(other scan dist) with respect to mid_pt_dist (set by user)
double p_DockedDistance = 0.178; //the distance from laser to arcylic board when docked (set by user)

//-------------------------------------------------------
//PUBLISHER VARIABLE
//-------------------------------------------------------
ros::Publisher pub_reverse;

//-------------------------------------------------------
//CALLBACK FUNCTION
//-------------------------------------------------------
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
    ROS_WARN("Docking DONE! Start Charging...");
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
    2*msg->ranges[0]*msg->ranges[1]*cos(msg->angle_increment)); //cosine rule

  if (g_CheckPointReached == 1)
  {
    int orient = 0; //when compare mid_pt_dist with cos(other scan dist), ++ when within p_Tolerance

    for (int i=0, n=(msg->ranges.size()/2-0.5); i < (msg->ranges.size()/2-0.5); i++, n--)
    {
      double lower_half = msg->ranges[i]*cos(n*msg->angle_increment);
      double upper_half = msg->ranges[msg->ranges.size()-i-1]*cos(n*msg->angle_increment);

      // if (i==0)
      // {
      // 	g_max_lower = lower_half;
      // 	g_min_lower = lower_half;
      // 	g_max_upper = upper_half;
      // 	g_min_upper = upper_half;
      // }

      // if (lower_half>g_max_lower) g_max_lower = lower_half;
      // if (lower_half<g_min_lower) g_min_lower = lower_half;
      // if (upper_half>g_max_upper) g_max_upper = upper_half;
      // if (upper_half<g_min_upper) g_min_upper = upper_half;

      if ( (upper_half <= (mid_pt_dist + p_Tolerance) && upper_half >= (mid_pt_dist - p_Tolerance) ) &&
        (lower_half <= (mid_pt_dist + p_Tolerance) && lower_half >= (mid_pt_dist - p_Tolerance) ) )
      {
        orient += 1;
      }

      // ROS_INFO("%d element after cos: %lf", i, msg->ranges[i]*cos(n*msg->angle_increment));
      // ROS_INFO("mid point distance: %lf", mid_pt_dist);
      // ROS_INFO("array size: %d", msg->ranges.size());
      // ROS_INFO("%d element after cos: %lf", msg->ranges.size()-i-1, msg->ranges[msg->ranges.size()-i-1]*cos(n*msg->angle_increment));
    }

    // double error = (check - 16) * single_error * 100; //calculate shift in x direction and convert into cm

    if (orient < p_OrientValue)
    {
      reverse.ReverseCheck = 1;
      pub_reverse.publish(reverse);
      ROS_WARN("Orientation not correct at CheckPoint!! Reversing....");
    }

    // ROS_WARN("lower error: %lf", g_max_lower - g_min_lower);
    // ROS_WARN("upper error: %lf", g_max_upper - g_min_upper);
    // ROS_INFO("check: %d", orient);
    // ROS_INFO("error: %lf", error);
  }

  if (g_MovingBack == 1)
  {
   	if (mid_pt_dist < p_DockedDistance && g_Docked==0)
    {
      reverse.ReverseCheck = 1;
      pub_reverse.publish(reverse);
      ROS_WARN("Cannot Docked!! Reversing....");
  	}
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Laser_Data");
  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  pnh.param<int>("OrientValue", p_OrientValue, 16);
  pnh.param<double>("Tolerance", p_Tolerance, 0.016);
  pnh.param<double>("DockedDistance", p_DockedDistance, 0.178);

  pub_reverse = nh.advertise<docking::FinalCheck>("FinalCheck",1);

  ros::Subscriber sub_Laser = nh.subscribe("scan1",50,&LaserCallBack);
  ros::Subscriber sub_FinalCheck = nh.subscribe("FinalCheck",10,&FinalCheckCallBack);
  ros::Subscriber sub_ChargingVoltage = nh.subscribe("chargingvoltage",1000,&ChargingVoltageCallBack);

  ros::spin();
}
