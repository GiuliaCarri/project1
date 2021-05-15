#include "ros/ros.h"
//#include "project1/MotorSpeed.h" /* Collegamento al nuovo tipo del messaggio*/
//#include "message_filters/subscriber.h"
//#include "message_filters/time_synchronizer.h"
//#include "message_filters/sync_policies/approximate_time.h"
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <sstream>

#define _USE_MATH_DEFINES
#define STEERING_FACTOR 18.0
#define BASELINE 1.3
#define INTERAXIS 1.765
#define Ts 0.1


class Odometry{

    private:

	double x;
	double y;
	double th;
    double vl;
    double va;

	ros::NodeHandle n;
    ros::Subscriber sub; 
  	ros::Publisher pub; 
  	ros::Timer timer1;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    nav_msgs::Odometry odom;

    int initial_pose[3] = {0,0,0};



	public:

    Odometry(){

        //subscriber part:
        sub = n.subscribe("/chatterVel", 1, &Odometry::callback_s, this);

        //publisher part
		pub = n.advertise<nav_msgs::Odometry>("/rechatterOdom", 1);
    	
        
        //timer1 = n.createTimer(ros::Duration(1), &Odometry::callback_p, this);
        //param initial_pose
        //n.getParam("/Initial_pose", initial_pose);
    }

	void callback_s(const geometry_msgs::TwistStamped::ConstPtr& mt) {
        vl = mt->twist.linear.x;
        va = mt->twist.angular.z;
        eulerOdom(vl, va);
	}

	void eulerOdom(double vl, double va){
        x = x + vl * Ts * cos(th); /*x(t+1)=x(t) + v * T * cos(th)*/
	    y = y + vl * Ts * sin(th);
	    th = th + va * Ts;
    }

    void rkOdom(double vl, double va){
        
        x = x + vl * Ts * cos(th + va*Ts/2);
        y = y + vl * Ts * sin(th + va*Ts/2);
        th = th + va * Ts;
    }

	void publish() {
    	
        ros::Time ct = ros::Time::now();
        tf2::Quaternion q;
        geometry_msgs::Quaternion qm;
        q.setRPY(0, 0, th);
        qm = tf2::toMsg(q);


		//tf
        transformStamped.header.stamp = ct;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";

        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;
        
        transformStamped.transform.rotation = qm;
       /* transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();*/


        //nav_msgs Odometry

        odom.header.stamp = ct;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation = qm;

        odom.twist.twist.linear.x = vl;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = va;




        //custom message
        

        //publish all

        br.sendTransform(transformStamped);
		pub.publish(odom);

		ROS_INFO_STREAM("sono in publish");
    }
    

	


};




int main(int argc, char **argv) {
  ros::init(argc, argv, "subBag_pubVel");
  Odometry my_Odometry;
  ros::spin();
  return 0;
}

