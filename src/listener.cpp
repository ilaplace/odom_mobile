#include "ros/ros.h"
#include "std_msgs/String.h"

#include <math.h>
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

#include <dynamic_reconfigure/server.h>

#include "odev/floatStamped.h"
#include "odev/sourcedOdom.h"
#include "odev/dyn_paramConfig.h"



using namespace message_filters;

//const double PI=3.1415926535897932;

int count = 0;
double L=1.3; //distance between two wheel (meter)
double DfrW = 1.765; //distance from front to rear wheels (meter)

class pub_sub
{

  private:
  double x = 0.0;
  double y = 0.0;
  double steer_angle = 0.0;
  double theta = 0;

  double Ts;

  double linear_speed;
  double angular_speed;

  double time_left_wheel_old;
  double time_left_wheel_now;

  double vL = 0;
  double vR = 0;


  bool diffdrive = true;

  ros::NodeHandle n;    

  ros::Publisher odom_pub;
  ros::Publisher custom_odom_pub;
  
  message_filters::Subscriber<odev::floatStamped> L_speed;
  message_filters::Subscriber<odev::floatStamped> R_speed;
  message_filters::Subscriber<odev::floatStamped> steer;
  
  dynamic_reconfigure::Server<odev::dyn_paramConfig> server;
  dynamic_reconfigure::Server<odev::dyn_paramConfig>::CallbackType f;
  
  tf::TransformBroadcaster odom_broadcaster;

  typedef sync_policies::ApproximateTime <odev::floatStamped, odev::floatStamped, odev::floatStamped> MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;
  

  public:

    pub_sub()
    {
      f = boost::bind(&pub_sub::dynamic_callback, this, _1, _2);
      server.setCallback(f);

      L_speed.subscribe(n, "speedL_stamped", 1);
      R_speed.subscribe(n,"speedR_stamped",1);
      steer.subscribe(n, "steer_stamped", 1);
      odom_pub = n.advertise<nav_msgs::Odometry>("odom",50);
      custom_odom_pub = n.advertise<odev::sourcedOdom>("custom_odom", 50);
  

      sync.reset(new Sync(MySyncPolicy(10), L_speed, R_speed, steer));
      sync->registerCallback(boost::bind(&pub_sub::callback, this, _1, _2, _3));

    }

    void diffdrive_kinematic_forward(double right_wheel_speed, double left_wheel_speed){

      count=count+1;

      if (count == 1){
            Ts = time_left_wheel_now - time_left_wheel_now; 
        }
          else{
            Ts = time_left_wheel_now - time_left_wheel_old;
          }  
        
          linear_speed = (right_wheel_speed+left_wheel_speed)/2.0;
          angular_speed = (right_wheel_speed-left_wheel_speed)/L;   
    }

 void ackerman_kinematic_forward(double right_wheel_speed, double left_wheel_speed, double steer_angle ){

      count=count+1;

      if (count == 1){
            Ts = time_left_wheel_now - time_left_wheel_now; 
        }
          else{
            Ts = time_left_wheel_now - time_left_wheel_old;
          }  
        
          linear_speed = (right_wheel_speed+left_wheel_speed)/2.0;
          angular_speed = (linear_speed*tan(steer_angle))/DfrW;   
    }

  void dynamic_callback(odev::dyn_paramConfig &config, uint32_t level){
  
    diffdrive=config.bool_param;
    x= config.double_param_x;
    y= config.double_param_y;
    ROS_INFO("diffdrive is %d", config.bool_param);
  }

  void callback(const odev::floatStamped::ConstPtr& msg1, const odev::floatStamped::ConstPtr& msg2, const odev::floatStamped::ConstPtr& msg3)
  {
    
    //ROS_INFO ("[counter = %d] Received two messages: (%lf) and (%lf) steer (%lf)",count, msg1->data , msg2->data, msg3->data);
    //ROS_INFO ("[counter = %d] Received two time: (%lf) and (%lf) steer time (%lf)",count, msg1->header.stamp.toSec() , msg2->header.stamp.toSec(), msg3->header.stamp.toSec());

    vL = msg1->data;
    vR = msg2->data;
    steer_angle = (msg3->data);
    steer_angle = (steer_angle/18.0)*M_PI/180.0;

    time_left_wheel_now = msg1->header.stamp.toSec();
    
    x = x + linear_speed*Ts*cos(theta+((angular_speed*Ts)/2));
    y = y + linear_speed*Ts*sin(theta+((angular_speed*Ts)/2));  
   
    ROS_INFO ("[counter = %d] x_new (%lf) y_new (%lf) Ts (%lf) ",count, x , y, Ts);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);


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
    odom.twist.twist.linear.x = linear_speed*cos(theta);
    odom.twist.twist.linear.y = linear_speed*sin(theta);
    odom.twist.twist.angular.z = angular_speed;


    
    //publish the message
    odom_pub.publish(odom);

    customOdomPublis(odom);
    
    theta = theta + (angular_speed*Ts); 
    time_left_wheel_old = time_left_wheel_now;
  }
   
   
  void customOdomPublis(nav_msgs::Odometry custOdom){

    odev::sourcedOdom customOdomMsg;

    customOdomMsg.header = custOdom.header;
    customOdomMsg.child_frame_id = custOdom.child_frame_id;
    customOdomMsg.pose = custOdom.pose;
    customOdomMsg.twist = custOdom.twist;
    
    if (diffdrive){
      diffdrive_kinematic_forward(vR, vL);
      customOdomMsg.source = "diffdrive";
    }
    else
    {
      ackerman_kinematic_forward(vR, vL, steer_angle);
      customOdomMsg.source = "ackerman";
    }

  custom_odom_pub.publish(customOdomMsg); 
  }

};



int main(int argc, char **argv){
  	
	ros::init(argc, argv, "listener");

  pub_sub my_pub_sub;

  ros::spin();

  return 0;
}


