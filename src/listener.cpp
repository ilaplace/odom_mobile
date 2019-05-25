#include "ros/ros.h"
#include "std_msgs/String.h"
#include "odev/floatStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <stdlib.h>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
using namespace message_filters;

const double PI=3.1415926535897932;

int count = 0;
double L=1.3; //distance between two wheel (meter)

double x = 0.0;
double y = 0.0;
double steer_angle = 0.0;

double Ts;

double linear_speed;
double angular_speed;

double time_left_wheel_old;
double time_left_wheel_now;

double vL = 0;
double vR = 0;

class pub_sub
{

  private:
  ros::NodeHandle n;    
  ros::Publisher odom_pub;
  message_filters::Subscriber<odev::floatStamped> L_speed;
  message_filters::Subscriber<odev::floatStamped> R_speed;
  message_filters::Subscriber<odev::floatStamped> steer;
  
  tf::TransformBroadcaster odom_broadcaster;

  typedef sync_policies::ApproximateTime <odev::floatStamped, odev::floatStamped, odev::floatStamped> MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;
  

  public:

    pub_sub()
    {

      L_speed.subscribe(n, "speedL_stamped", 1);
      R_speed.subscribe(n,"speedR_stamped",1);
      steer.subscribe(n, "steer_stamped", 1);
      //odom_pub.publish(n.advertise<nav_msgs::Odometry>("odom",50));
      odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  

      sync.reset(new Sync(MySyncPolicy(10), L_speed, R_speed, steer));
      sync->registerCallback(boost::bind(&pub_sub::callback, this, _1, _2, _3));

    }

    void diffdrive_kinematic_forward(double right_wheel_speed, double left_wheel_speed, double steer_angle ){

      count=count+1;

      if (count == 1){
            Ts = time_left_wheel_now - time_left_wheel_now; 
        }
          else{
            Ts = time_left_wheel_now - time_left_wheel_old;
          }  
        
          //double v = (vL+vR)/2.0;
          //double w = (vR-vL)/L;

          linear_speed = (vL+vR)/2.0;
          angular_speed = (vR-vL)/L;

          //x = x + v*Ts*cos((steer_angel*PI/180.0)+((w*Ts)/2));
          x = x + linear_speed*Ts*cos((steer_angle*PI/180.0)+((angular_speed*Ts)/2));
          y = y + linear_speed*Ts*sin((steer_angle*PI/180.0)+((angular_speed*Ts)/2));
          steer_angle = steer_angle*PI/180.0 + (angular_speed*Ts); 

    }

  void callback(const odev::floatStamped::ConstPtr& msg1, const odev::floatStamped::ConstPtr& msg2, const odev::floatStamped::ConstPtr& msg3)
  {
    
    //ROS_INFO ("[counter = %d] Received two messages: (%lf) and (%lf) steer (%lf)",count, msg1->data , msg2->data, msg3->data);
    //ROS_INFO ("[counter = %d] Received two time: (%lf) and (%lf) steer time (%lf)",count, msg1->header.stamp.toSec() , msg2->header.stamp.toSec(), msg3->header.stamp.toSec());

    vL = msg1->data;
    vR = msg2->data;
    steer_angle = (msg3->data)/18.0;

    time_left_wheel_now = msg1->header.stamp.toSec();
 
    diffdrive_kinematic_forward(vR, vL, steer_angle);

    time_left_wheel_old = time_left_wheel_now;
    ROS_INFO ("[counter = %d] x_new (%lf) y_new (%lf) Ts (%lf) ",count, x , y, Ts);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(steer_angle*PI/180.0);


    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = msg1->header.stamp;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    //odom_broadcaster.sendTransform(odom_trans.transform, ros::Time::now(), "world", "turtle"));
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = msg1->header.stamp;
    odom.header.frame_id = "world";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vL;
    odom.twist.twist.linear.y = vR;
    odom.twist.twist.angular.z = steer_angle*PI/180.0;

    //publish the message
    odom_pub.publish(odom); 
  }


};



int main(int argc, char **argv){
  	
	ros::init(argc, argv, "listener");

  pub_sub my_pub_sub;

  ros::spin();

  return 0;
}


