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

/*
//for waypoint
#include <actionlib/client/simple_action_client.h>
#include "ros/xmlrpc_manager.h"
#include <move_base_msgs/MoveBaseAction.h>
*/
//
//08 Oct 2014
//Getting info from charging node, ar_pose and waypoint node.
//Publish to odom tv/sv/rv


int _bTest = 0;
int _dPart1 ;
int _dPart1_reset;
int clockwise;
//n.getParam("_dPart1", _dPart1_reset);
//long int time_to_turn2 = 0;
//int n;

//Actual code = 0, Test = 9//
int _bReachedWP = 0;

long int _fMoveForward2;
int _bCondition = 0;
bool _bNeedtouse = 0;
int _bCheck = 0;
// double transx = 0.0;
// double transy = 0.0;
// double transz = 0.0;
// double rotx = 0.0;
// double roty = 0.0;
// double rotz = 0.0;
double rotw = 0.0;
geometry_msgs::PoseWithCovarianceStamped initialPose;

int _bDocked = 0;
int _bDockDone = 0;
int _bDockingPose = 1;
/*
//for waypoint
int goalPoint = 1;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
  MoveBaseClient;
move_base_msgs::MoveBaseGoal goal;
XmlRpc::XmlRpcValue pose1, pose2, pose3, orient1, orient2, orient3, b_pose, b_orient;
int tempPoint=1;
int battStatus;
*/

class DockingClass
{
public:
  DockingClass();

private:
  void DockingCallBack(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& dockingpose);
  //void GetNearCallBack(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& distance);
  void batteryCallBack(const std_msgs::String::ConstPtr& batterylevel);
  void batterylevelCallBack(const m4api_battery::PowerReading::ConstPtr& batterylevel);
  void chargingvoltageCallBack(const std_msgs::Float32::ConstPtr& chargingvoltage);
  void needtouseCallBack(const std_msgs::Bool::ConstPtr& needtouse);
  void waypointreachedCallBack(const std_msgs::String::ConstPtr& waypointreached);
  void signal3fromremotecontrolCallBack(const std_msgs::String::ConstPtr& signalfromremotecontrol);
  void getIMUCallBack(const std_msgs::String::ConstPtr& imu);

//void settimer();

int _bLowBattery;
int chargingdone;
double _fRobotHeading;

ros::NodeHandle n;

ros::Publisher pub_vel;
ros::Publisher pub_wprqt;
ros::Publisher pub_chargingdone;
ros::Publisher plotbattery;
ros::Publisher pub_startmotor;
ros::Publisher set_pose_pub;

ros::Subscriber pose;
//ros::Subscriber get_near;
ros::Subscriber sub_battery;
ros::Subscriber sub_battery_status_m4atx;
ros::Subscriber sub_chargingvoltage;
ros::Subscriber sub_needtouse;
ros::Subscriber sub_waypointreached;
ros::Subscriber sub_signal3fromremotecontrol;
ros::Subscriber sub_imu;
};

DockingClass::DockingClass()
{
  pub_vel = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  pub_wprqt = n.advertise<std_msgs::String>("waypointrequest",1);
  pub_chargingdone = n.advertise<std_msgs::String>("chargingdone",1);
  plotbattery = n.advertise<std_msgs::Float32>("plotbattery",1);
  pub_startmotor = n.advertise<std_msgs::String>("startmotor",1);
  set_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);

  pose = n.subscribe("ar_pose_marker_Small",50,&DockingClass::DockingCallBack,this);
  //get_near = n.subscribe("ar_pose_marker_Big",50,&DockingClass::GetNearCallBack,this);
  sub_battery = n.subscribe("Vin",1000,&DockingClass::batteryCallBack,this);
  sub_battery_status_m4atx = n.subscribe("sub_battery_status_m4atx",10,&DockingClass::chargingvoltageCallBack,this);
  sub_chargingvoltage = n.subscribe("chargingvoltage",1000,&DockingClass::chargingvoltageCallBack,this);
  sub_needtouse = n.subscribe("needtouse",1000,&DockingClass::needtouseCallBack,this);
  sub_waypointreached = n.subscribe("waypointreached",20,&DockingClass::waypointreachedCallBack,this);
  sub_signal3fromremotecontrol = n.subscribe("button3pressed",1000,&DockingClass::signal3fromremotecontrolCallBack,this);
  sub_imu = n.subscribe("imu_rpy",100,&DockingClass::getIMUCallBack,this);
}

void DockingClass::getIMUCallBack(const std_msgs::String::ConstPtr& imu)
{
  geometry_msgs::Twist vel;
  char *pch;
  double temp;
  double tv,sv,rv;
  double angle_small;
  double angle_large;
  n.getParam("angle_small",angle_small);
  n.getParam("angle_large",angle_large);
  /*
  if(clockwise==0)
  {
  angle_small = 107;
  angle_large = 110;
  }
  */
  pch = strtok((char*)imu->data.c_str(),"_,");

  for(int i=0; i<5; i++)
  {
    pch = strtok (NULL, "_,");
    temp = atof(pch);
  }
  _fRobotHeading = temp;
  //ROS_INFO("My heading is %2.1f degrees, part1 = %d, _bReachedWP = %d, docked = %d, Needtouse = %d, chargingdone = %d, _bCondition = %d", temp, _dPart1, _bReachedWP, _bDocked, _bNeedtouse, chargingdone, _bCondition);
  //ROS_INFO("My heading is %2.1f degrees, clockwise = %d", temp, clockwise);
  //if(_dPart1 == 0)
  //_bDocked = 0;
  if(_bReachedWP == 9) //to confirm that picture is received and check that MAVEN has reached the waypoint for docking
  {
    if(_bDocked == 0)
    {
      if(_dPart1 == 0)
      {
        if((_fRobotHeading<=angle_small && _bCheck == 0) || (rotw<0 && clockwise == 1))
        {
          clockwise = 1;
          if(_bTest == 1)
          {
            tv = 0;
            sv = 0;
            rv = 0.2;
            _bTest = 0;
          }
          else
          {
            tv = 0;
            sv = 0;
            rv = 0.199999;
            _bTest = 1;
          }
        }
        else if((_fRobotHeading>=angle_large && _bCheck ==0) || (rotw<0 && clockwise == 0))
        {
          clockwise = 0;
          if(_bTest == 1)
          {
            tv = 0;
            sv = 0;
            rv = -0.2;
            _bTest = 0;
          }
          else
          {
            tv = 0;
            sv = 0;
            rv = -0.199999;
            _bTest = 1;
          }
        }
        if(_fRobotHeading<angle_large && _fRobotHeading>angle_small)
        {
          _bCheck = 1;
          _dPart1 = 2;
        }

        vel.angular.z = -rv;
        vel.linear.x = -tv;
        vel.linear.y = -sv;
        pub_vel.publish(vel);
        ROS_INFO("rotating");
      }
    }
  }
}

void DockingClass::signal3fromremotecontrolCallBack(const std_msgs::String::ConstPtr& signal3fromremotecontrol)
{

}

void DockingClass::waypointreachedCallBack(const std_msgs::String::ConstPtr& waypointreached)
{
  _bReachedWP = atoi(waypointreached->data.c_str());
  //ROS_INFO("_bReachedWP = %d",_bReachedWP);
}

void DockingClass::needtouseCallBack(const std_msgs::Bool::ConstPtr& needtouse)
{
  int temp;
  temp = needtouse->data;
  if(temp == 1)
  {
    _bNeedtouse = 1;
    _bReachedWP = 0;
  }
  if(temp == 1 && _bDocked == 0)
  {
    _bNeedtouse = 0;
  }
  //ROS_INFO("_bReachedWP = %d", _bReachedWP);
}

void DockingClass::chargingvoltageCallBack(const std_msgs::Float32::ConstPtr& chargingvoltage)
{
  double chargingnodevoltagelevel = chargingvoltage->data;

  //Data to be retrieved from docking.yaml
  XmlRpc::XmlRpcValue battery_pose, battery_orient;
  double chargingnode_thresholdvoltage;

  geometry_msgs::Twist vel;
  std_msgs::String startmotor;

  n.getParam("battery_charging_pose",battery_pose);
  n.getParam("battery_charging_orient",battery_orient);
  n.getParam("chargingnode_thresholdvoltage",chargingnode_thresholdvoltage);

  ros::Time time_now = ros::Time::now();
  long int time_now2;
  time_now2 = time_now.sec;
  std_msgs::String msg;
  //ROS_WARN("CHARGINGVOLTAGE = %3.2f", chargingnodevoltagelevel);

  if(chargingnodevoltagelevel>2.6)
  {
    _bDocked = 1;
    //ROS_WARN("Charging...");
  }
  else if(chargingnodevoltagelevel<2.5)
  {
    _bDocked = 0;
  }

  if(chargingnodevoltagelevel < chargingnode_thresholdvoltage && _bDocked == 1 && _bNeedtouse == 1)
  {
    chargingdone = 1;
    startmotor.data = "1";
    if(_bCondition == 0)
    {
      //ros::Time moveforward = ros::Time::now() + ros::Duration(2);
      _fMoveForward2 = time_now2 + 4;

      for(int j = 0; j < 100; j++)
      {
        initialPose.pose.pose.position.x = battery_pose [0];
        initialPose.pose.pose.position.y = battery_pose [1];
        initialPose.pose.pose.orientation.z = battery_orient [2];
        initialPose.pose.pose.orientation.w = battery_orient [3];
        set_pose_pub.publish(initialPose);
        ROS_WARN("Publishing_Initial_Pose");
      }
    }
    _bCondition = 1;
  }
  else
  {
    //_bCondition = 0;
  }

  pub_startmotor.publish(startmotor);

  if(chargingdone == 1)
  {
    if(time_now2<_fMoveForward2)
    {
      if(_bTest == 0)
      {
        vel.linear.y = 0;
        vel.linear.x = 0.2;
        vel.angular.z = 0;
        _bTest = 1;
      }
      else
      {
        vel.linear.y = 0;
        vel.linear.x = 0.199999;
        vel.angular.z = 0;
        _bTest = 0;
      }
      pub_vel.publish(vel);
      ROS_INFO("Check");
    }
    else
    {
      msg.data = "1";
      for(int j=0; j<100; j++)
      {
        pub_chargingdone.publish(msg);
        ROS_WARN("Publishing_Charging_Done");
      }
      _bReachedWP = 0;
      _bDocked = 0;
      _dPart1 = _dPart1_reset;
      _bNeedtouse = 0;
      chargingdone = 0;
      _bCondition = 0;
      startmotor.data = "0";
      _bCheck = 0;
    }
  }
  // ROS_INFO("time_now2 = %ld, _fMoveForward2 = %ld",time_now2, _fMoveForward2);
  // ROS_INFO("chargindone = %d, _bDocked = %d",chargingdone, _bDocked);
}

void DockingClass::batterylevelCallBack(const m4api_battery::PowerReading::ConstPtr& batterylevel)
{
  std_msgs::String msg;
  std::stringstream ss;
  std_msgs::Float32 plotbatterylevel;
  float Vin = batterylevel->volts_read_value[0];
  plotbatterylevel.data = Vin;

  if(Vin < 20.0)  //1:Request wp, 2:charging done, 0:No need chargingdone
  {
    _bLowBattery = 1;
    //system("paplay /usr/share/sounds/ubuntu/stereo/message-new-instant.ogg");
    msg.data = "1";
  }
  else
  {
    _bLowBattery = 0;
    msg.data = "0";
  }

  if(Vin > 27.0)
  {
    //_bDocked = 1;
  }
  //ROS_INFO("Vin = %3.2f", Vin);
  pub_wprqt.publish(msg);
  plotbattery.publish(plotbatterylevel);
}

void DockingClass::batteryCallBack(const std_msgs::String::ConstPtr& batterylevel)
{

}

void DockingClass::DockingCallBack(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& dockingpose)
{
  geometry_msgs::Twist vel;
  tf::Quaternion q;
  double transx[2], transy[2], transz[2];
  double x, z;
  double dummy_x0, dummy_x1;
  double tv=0.0, sv=0.0, rv=0.0;
  double roll, pitch, yaw;
  bool heading;

//to check for number of markers detected (to avoid segfault)
  if (dockingpose->markers.size()==0)
  	_bDockingPose = 0;

  if (dockingpose->markers.size()==1)
  {
   //if only 1 marker is detected, check for which marker using marker id
   if (dockingpose->markers[0].id==0)
   {
   	 _bDockingPose = 1;

     transx[0] = dockingpose->markers[0].pose.pose.position.x;
     transy[0] = dockingpose->markers[0].pose.pose.position.y;
     transz[0] = dockingpose->markers[0].pose.pose.position.z;
   }
   if (dockingpose->markers[0].id==1)
   {
   	 _bDockingPose = 1;

     transx[1] = dockingpose->markers[0].pose.pose.position.x;
     transy[1] = dockingpose->markers[0].pose.pose.position.y;
     transz[1] = dockingpose->markers[0].pose.pose.position.z;
   }
  }

//if 2 markers detected, assign data of markers accordingly with respect to marker id
  if (dockingpose->markers.size()==2)
  {
  	if (dockingpose->markers[0].id==0)
  	{
  	  _bDockingPose = 1;

  	  transx[0] = dockingpose->markers[0].pose.pose.position.x;
      transy[0] = dockingpose->markers[0].pose.pose.position.y;
      transz[0] = dockingpose->markers[0].pose.pose.position.z;

      transx[1] = dockingpose->markers[1].pose.pose.position.x;
      transy[1] = dockingpose->markers[1].pose.pose.position.y;
      transz[1] = dockingpose->markers[1].pose.pose.position.z;
  	}
  	if (dockingpose->markers[0].id==1)
  	{
  	  _bDockingPose = 1;

  	  transx[0] = dockingpose->markers[1].pose.pose.position.x;
      transy[0] = dockingpose->markers[1].pose.pose.position.y;
      transz[0] = dockingpose->markers[1].pose.pose.position.z;

      transx[1] = dockingpose->markers[0].pose.pose.position.x;
      transy[1] = dockingpose->markers[0].pose.pose.position.y;
      transz[1] = dockingpose->markers[0].pose.pose.position.z;
  	}
  }
   
  x=transx[0]+transx[1];
  z=(transz[0]+transz[1])/2;
    

  //if (_bDockingPose == 1)
  //{
    ROS_WARN("x0:%f, y0:%f, z0:%f", transx[0], transy[0], transz[0]);
    ROS_WARN("x1:%f, y1:%f, z1:%f\n\n", transx[1], transy[1], transz[1]);
    // ROS_INFO("item in array:%d",dockingpose->markers.size());
  //}


//if no markers are detected, rotate until 1 marker is detected
  //if (dockingpose->markers.size()==0)
    //rv = 0.2;

//use different in z value from 2 markers to detect orientation of robot and turn accordingly
  if (dockingpose->markers.size()==1 && _bDockDone<50)
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

  if (dockingpose->markers.size()==2 && _bDockDone<50)
  {
  	dummy_x0 = transx[0];
  	dummy_x1 = transx[1];
    if (transz[0]>transz[1] && fabs(transz[0]-transz[1])>0.03)
    {
      //current is assuming robot is on the right region if it detect marker 0 first
      sv = 0.1;
      rv = -0.05;
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
      rv = 0.05;
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


  if ( (x<0.05 || x>-0.05) && heading==1 && _bDockDone<50)
  {
    //tv = (z>0.3)?0.1:0.0;
    if (z>0.3)
      tv = 0.1;
    else
    {
      tv = 0.0;
      if (dockingpose->markers.size()==2)
      	_bDockDone += 1;
    }
  }
  if (x>0.05 && heading==1 && _bDockDone<50 && dockingpose->markers.size()==2)
    sv = -0.1;

  if (x<-0.05 && heading==1 && _bDockDone<50 && dockingpose->markers.size()==2)
    sv = 0.1;

  if (_bDocked==1)
  {
  	tv = 0.0;
  	sv = 0.0;
  	rv = 0.0;
  	_bDockDone = 0;
  }

//Reverse the robot for recalibrate if it get too close and not in the right pose
  if (dockingpose->markers.size()==1 && _bDockDone<50)
  {
    if ( (dockingpose->markers[0].id==0 && transz[0]<0.27) || (dockingpose->markers[0].id==1 && transz[1]<0.27) )
    {
    	tv = -0.1;
    	vel.linear.x = -tv;
    	pub_vel.publish(vel);
    	ros::Duration(1).sleep();
    	tv = 0.0;
    }
  }

  vel.angular.z = rv;
  vel.linear.x = -tv;
  vel.linear.y = -sv;
  pub_vel.publish(vel);

  //if (_bDockingPose == 1)
  //{
    ROS_INFO("tv:%f, sv:%f, rv:%f\n\n", tv, sv, rv);
    ROS_INFO("dock done: %d", _bDockDone);
    ROS_INFO("Heading: %d", heading);
  //}

  //Last step to move closer because still too far from station 
  //when both marker at edge of camera detection
  if (_bDockDone >= 50 && _bDocked == 0)
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
  }
}

/*void DockingClass::GetNearCallBack(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& distance)
{
  geometry_msgs::Twist vel;
  double transx, transy, transz;
  double tv=0.0, sv=0.0, rv=0.0;

  if (_bDockingPose==0 && distance->markers.size()==0)
  	rv = 0.1;

  if ( distance->markers.size()>0)
  {
  	if (distance->markers[0].id==2)
  	{
  	  transx = distance->markers[0].pose.pose.position.x;
  	  transy = distance->markers[0].pose.pose.position.y;
  	  transz = distance->markers[0].pose.pose.position.z;
  	}
  	if (distance->markers[1].id==2)
  	{
  	  transx = distance->markers[1].pose.pose.position.x;
  	  transy = distance->markers[1].pose.pose.position.y;
  	  transz = distance->markers[1].pose.pose.position.z;
  	}
  	if (distance->markers[2].id==2)
  	{
  	  transx = distance->markers[2].pose.pose.position.x;
  	  transy = distance->markers[2].pose.pose.position.y;
  	  transz = distance->markers[2].pose.pose.position.z;
  	}

  	tv = 0.15;
  	sv = (transx>0)?-0.1:0.1;
  }

  ROS_WARN("x2:%f, y2:%f, z2:%f", transx, transy, transz);
  
  vel.angular.z = rv;
  vel.linear.x = -tv;
  vel.linear.y = -sv;
  pub_vel.publish(vel);
}*/


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Docking");
  DockingClass shaunenode;

  ros::spin();
}
