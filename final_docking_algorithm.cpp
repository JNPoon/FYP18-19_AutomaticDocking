#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "docking/FinalCheck.h"
#include <math.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "ros/time.h"

bool g_Docked = 0;
int g_CheckPoint = 0;
bool g_GetNear = 1; //concept for getting near using larger marker with 10cm

ros::Publisher pub_vel;
ros::Publisher pub_FinalCheck;

void DataChecking(double transx[], double transz[], const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& dockingpose)
{
  if (dockingpose->markers.size()==0)
  	g_GetNear = 0;

  //if only 1 marker is detected, check for which marker using marker id
  if (dockingpose->markers.size()==1)
  {
   int id = dockingpose->markers[0].id;
   g_GetNear = 1;

   transx[id] = dockingpose->markers[0].pose.pose.position.x;
   transz[id] = dockingpose->markers[0].pose.pose.position.z;
  }

  //if 2 markers detected, assign data of markers accordingly with respect to marker id
  if (dockingpose->markers.size()==2)
  {
    int id1 = dockingpose->markers[0].id;
    int id2 = dockingpose->markers[1].id;
    g_GetNear = 1;

    transx[id1] = dockingpose->markers[0].pose.pose.position.x;
    transz[id1] = dockingpose->markers[0].pose.pose.position.z;

    transx[id2] = dockingpose->markers[1].pose.pose.position.x;
    transz[id2] = dockingpose->markers[1].pose.pose.position.z;
  }
}

void SearchSecondMarker(double &sv, double &rv, const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& dockingpose)
{
  if (dockingpose->markers[0].id==0)
  {
    rv = -0.05;
    sv = 0.0; //added for trial on 17 Jan 2019
  }
  if (dockingpose->markers[0].id==1)
  {
    rv = 0.05;
    sv = 0.0; //added for trial on 17 Jan 2019
  }
}

void AdjustOrientation(double transz[], double &sv, double &rv, bool &heading)
{
  if (transz[0]>transz[1] && fabs(transz[0]-transz[1])>0.03)
  {
    //current is assuming robot is on the right region if it detect marker 0 first
    sv = 0.1;
    rv = -0.05;
  }
  if (transz[1]>transz[0] && fabs(transz[0]-transz[1])>0.03)
  {
    //current is assuming robot is on the left region if it detect marker 1 first
    sv = -0.1;
    rv = 0.05;
  }
  if (fabs(transz[0]-transz[1])>0.00 && fabs(transz[0]-transz[1])<0.03)
  {
    heading = 1;
    rv = 0.0;
  }
}

void Reverse(double &tv, geometry_msgs::Twist &vel)
{
  tv = -0.1;
  vel.linear.x = -tv;
  pub_vel.publish(vel);
  ros::Duration(1).sleep();
  tv = 0.0;
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

void ReverseCallBack(const docking::FinalCheck msg)
{
  geometry_msgs::Twist vel;
  docking::FinalCheck check;
  double tv=0.0;

  if (msg->ReverseCheck == 1)
  {
    Reverse(tv, vel);
    check.CheckPointReached = 0;
    check.MovingBack = 0;
    check.ReverseCheck = 0;
    pub_FinalCheck.publish(check);
  }
}

void DockingCallBack(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& dockingpose)
{
  geometry_msgs::Twist vel;
  docking::FinalCheck check;
  double transx[2], transz[2];
  double x, z;
  double tv=0.0, sv=0.0, rv=0.0;
  bool heading;

  x = transx[0] + transx[1];
  z = (transz[0] + transz[1]) / 2;

  //to check for number of markers detected (to avoid segfault)
  DataChecking(transx, transz, dockingpose);

  //Printing Info
  if (g_Docked == 0 && g_CheckPoint < 50)
  {
    ROS_WARN("x0:%f, z0:%f", transx[0], transz[0]);
    ROS_WARN("x1:%f, z1:%f\n\n", transx[1], transz[1]);
  }

  if (dockingpose->markers.size()==1 && g_CheckPoint<50)
    SearchSecondMarker(sv, rv, dockingpose);

  //use different in z value from 2 markers to detect orientation of robot and turn accordingly
  if (dockingpose->markers.size()==2 && g_CheckPoint<50)
    AdjustOrientation(transz, sv, rv, heading);

  //Mobe Backward to CheckPoint if heading is correct
  if ( (x<0.05 || x>-0.05) && heading==1 && g_CheckPoint<50)
  {
    if (z>0.3)
      tv = 0.1;
    else
    {
      tv = 0.0;
      check.CheckPointReached = 1;
      check.MovingBack = 0;
      pub_FinalCheck.publish(check);

      //To Start the CheckPoint count
      if (dockingpose->markers.size()==2) g_CheckPoint += 1;
    }
  }

  //Final Adjustmnet for Left&Right Position
  if (heading==1 && g_CheckPoint<50 && dockingpose->markers.size()==2)
  {
    if (x>0.05) sv = -0.1;
    if (x<-0.05) sv = 0.1;
  }

  //Last step to move closer because still too far from station
  //when both marker at edge of camera detection
  if (g_CheckPoint >= 50 && g_Docked == 0)
  {
    tv = 0.05;
    check.CheckPointReached = 0;
    check.MovingBack = 1;
    pub_FinalCheck.publish(check);
    ROS_INFO("Final Step: Moving %.2f backward to charge", tv);
  }

  //Robot Charging, Stop Moving and Reset CheckPoint Count
  if (g_Docked==1)
  {
    tv = 0.0;
    sv = 0.0;
    rv = 0.0;
    g_CheckPoint = 0;
    ROS_WARN("Docking DONE! Start Charging...");
  }

  //Reverse the robot for recalibrate if it get too close and pose is wrong
  if (dockingpose->markers.size()==1 && g_CheckPoint<50)
    if ( (dockingpose->markers[0].id==0 && transz[0]<0.27) || (dockingpose->markers[0].id==1 && transz[1]<0.27) )
      Reverse(tv, vel);

  vel.angular.z = rv;
  vel.linear.x = -tv;
  vel.linear.y = -sv;
  pub_vel.publish(vel);

  //Printing Info
  if (g_Docked == 0 && g_CheckPoint < 50)
  {
    ROS_INFO("tv:%f, sv:%f, rv:%f\n\n", tv, sv, rv);
    ROS_INFO("dock done: %d", g_CheckPoint);
    ROS_INFO("Heading: %d", heading);
    ROS_INFO("Docked: %d", g_Docked);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Docking");
  ros::NodeHandle nh;
  pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  pub_FinalCheck = nh.advertise<docking::FinalCheck>("FinalCheck",1);
  ros::Subscriber sub_pose = nh.subscribe("ar_pose_marker_Small",50,&DockingCallBack);
  ros::Subscriber sub_ChargingVoltage = nh.subscribe("chargingvoltage",1000,&ChargingVoltageCallBack);
  ros::Subscriber sub_reverse = nh.subscribe("FinalCheck",10,&ReverseCallBack);
  ros::spin();
}
