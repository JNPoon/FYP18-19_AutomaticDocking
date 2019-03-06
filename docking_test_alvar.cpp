#include "ar_single.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include "ARMarker.h"
#include "geometry_msgs/Twist.h"
#include "tf/tfMessage.h"
#include "tf/transform_listener.h"
#include <m4api_battery/PowerReading.h>
#include <stdlib.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <ros/xmlrpc_manager.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "ros/time.h"
 
ros::Publisher pub_vel;

void DockingCallBack(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& dockingpose)
{
  geometry_msgs::Twist vel;
  tf::Quaternion q;
  double transx[2], transy[2], transz[2];
  double x, z;
  double dummy_x0, dummy_x1;
  double tv, sv, rv;
  double roll, pitch, yaw;
  int heading;

//to check for number of markers detected (to avoid segfault)
  if (dockingpose->markers.size()==1)
  {
   //if only 1 marker is detected, check for which marker using marker id
    if (dockingpose->markers[0].id==0)
   {
     transx[0] = dockingpose->markers[0].pose.pose.position.x;
     transy[0] = dockingpose->markers[0].pose.pose.position.y;
     transz[0] = dockingpose->markers[0].pose.pose.position.z;
   }
   if (dockingpose->markers[0].id==1)
   {
     transx[1] = dockingpose->markers[0].pose.pose.position.x;
     transy[1] = dockingpose->markers[0].pose.pose.position.y;
     transz[1] = dockingpose->markers[0].pose.pose.position.z;
   }
  }

  if (dockingpose->markers.size()==2)
  {
    transx[0] = dockingpose->markers[0].pose.pose.position.x;
    transy[0] = dockingpose->markers[0].pose.pose.position.y;
    transz[0] = dockingpose->markers[0].pose.pose.position.z;
 
    transx[1] = dockingpose->markers[1].pose.pose.position.x;
    transy[1] = dockingpose->markers[1].pose.pose.position.y;
    transz[1] = dockingpose->markers[1].pose.pose.position.z;
  }
 
  ROS_INFO("x0:%f, y0:%f, z0:%f", transx[0], transy[0], transz[0]);
  ROS_INFO("x1:%f, y1:%f, z1:%f\n\n", transx[1], transy[1], transz[1]);
 // ROS_INFO("item in array:%d",dockingpose->markers.size());

  x=transx[0]+transx[1];
  z=(transz[0]+transz[1])/2;

  //ros::Time::init();

//if no markers are detected, rotate until 1 marker is detected
  //if (dockingpose->markers.size()==0)
    //rv = 0.2;

//use different in z value from 2 markers to detect orientation of robot and turn accordingly
  if (dockingpose->markers.size()==1)
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

  if (dockingpose->markers.size()==2)
  {
    dummy_x0 = transx[0];
    dummy_x1 = transx[1];
    rv = 0.0;
    //sv = 0.075;
    if (transz[0]>transz[1] && fabs(transz[0]-transz[1])>0.03)
    {
      //current is assuming robot is on the right region if it detect marker 0 first
      sv = 0.1;
      //to check whether the robot should move left or move right
      /*if (transx[0]<dummy_x0 && transx[1]<dummy_x1)
      {
        sv = -0.075;
      }*/
    } 
    if (transz[1]>transz[0] && fabs(transz[0]-transz[1])>0.03)
    {
      //current is assuming robot is on the left region if it detect marker 1 first
      sv = -0.1;
      //to check whether the robot should move left or move right
      /*if (transx[0]<dummy_x0 && transx[1]<dummy_x1)
      {
        sv = -0.075;
      }*/
    }
    if (fabs(transz[0]-transz[1])>0.00 && fabs(transz[0]-transz[1])<0.03)
    {
      heading = 1;
      rv = 0.0;
    }
  }


  if (x<0.05 || x>-0.05 && heading==1)
  {
    //tv = (z>0.3)?0.1:0.0;
    if (z>0.3)
    {
      tv = 0.1;
    }
    else
    {
      tv = 0.0;
      //_bDockDone += 1;
    }
  }
  if (x>0.05 && heading==1)
  {
    sv = -0.1;
    //tv = (z>0.5)?0.1:0.0;
  }  
  if (x<-0.05 && heading==1)
  {
    sv = 0.1;
    //tv = (z>0.5)?0.1:0.0;
  }

  /*if (_bDocked==1)
  {
    tv = 0.0;
    sv = 0.0;
    rv = 0.0;
    _bDockDone = 0;
  }*/

  vel.angular.z = rv;
  vel.linear.x = -tv;
  vel.linear.y = -sv;
  pub_vel.publish(vel);
  ROS_INFO("tv:%f, sv:%f, rv:%f\n\n", tv, sv, rv);
  //ROS_INFO("dock done: %d", _bDockDone);

  //Last step to move closer because still too far from station 
  //when both marker at edge of camera detection
  /*if (_bDockDone >= 50 && _bDocked == 0 && heading ==1)
  {
    tv = 0.05;
    vel.linear.x = -tv;
    pub_vel.publish(vel);
    ROS_INFO("Final Step: Moving %.2f backward to charge", tv);
    if (_bDocked==1)
    {
      tv = 0.0;
      vel.linear.x = -tv;
      pub_vel.publish(vel);
      ROS_INFO("Docking DONE! Start Charging");
    }
  }*/
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Docking");
  ros::NodeHandle nh;
  pub_vel=nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  ros::Subscriber pose= nh.subscribe("ar_pose_marker",50,&DockingCallBack);
  ros::spin();
}
