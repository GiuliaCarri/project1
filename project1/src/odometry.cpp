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
#include <dynamic_reconfigure/server.h>
#include <project1/integrationConfig.h>
#include "project1/RiCentra.h"
#include "project1/RiPosiziona.h"


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

    //pub-sub
    ros::Subscriber sub; 
  	ros::Publisher pub; 
  	ros::Timer timer1;

    //services
    ros::ServiceServer riCentra;
    ros::ServiceServer riPosiziona;

    //odometry
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    nav_msgs::Odometry odom;

    //param reconfigure

    /*double initial_x;
    double initial_y;
    double initial_th;*/
    int integration_type; //0 -> Euler 1 -> Runge-Kutta




	public:

    Odometry(ros::NodeHandle node){

        n = node;

        //static param configuration
        n.getParam("/Initial_x", x);
        n.getParam("/Initial_y", y);
        n.getParam("/Initial_th", th);

        /*x=initial_x;
        y=initial_y;
        th=initial_th;*/

        ROS_INFO ("Initial pose x: ( %f ) y: ( %f ) z: ( %f )", x, y, th);

        //subscriber part
        sub = n.subscribe("/rechatterVel", 1, &Odometry::callback_s, this);


        //publisher part
		pub = n.advertise<nav_msgs::Odometry>("/rechatterOdom", 1);
    	ROS_INFO_STREAM ("creatore");

        //service part
        riCentra = n.advertiseService("ricentra", &Odometry::ricentra, this);
        riPosiziona = n.advertiseService("riposiziona", &Odometry::riposiziona, this);
        
        //timer1 = n.createTimer(ros::Duration(1), &Odometry::callback_p, this);
        //param initial_pose
        //n.getParam("/Initial_pose", initial_pose);
    }

    //param reconfigure: getter and setter
    int getIntegrationType(){ 
        return integration_type; 
    }
    void setIntegrationType(int integrationType){ 
        integration_type = integrationType; 
    }
	

	void callback_s(const geometry_msgs::TwistStamped::ConstPtr& mt) {
        ROS_INFO_STREAM ("sono nel callabak");
        vl = mt->twist.linear.x;
        va = mt->twist.angular.z;
        if (getIntegrationType() == 0)
            eulerOdom(vl, va);
        else
            if (getIntegrationType() == 1)
                rkOdom(vl, va);

        publish();
        
	}

	void eulerOdom(double vl, double va){
        ROS_INFO_STREAM ("sono nella euleroOdom");
        x = x + vl * Ts * cos(th); /*x(t+1)=x(t) + v * T * cos(th)*/
	    y = y + vl * Ts * sin(th);
	    th = th + va * Ts;
        
    }

    void rkOdom(double vl, double va){
        ROS_INFO_STREAM("sono nella rkOdom");
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



    bool ricentra(project1::RiCentra::Request  &req, project1::RiCentra::Response  &resp)
    {
        ROS_INFO("Richiesta di ricentrare l'odom");
        x=0;
        y=0;
        th=0;
        return true;
    }
    
    bool riposiziona(project1::RiPosiziona::Request  &req, project1::RiPosiziona::Response  &resp)
    {
        ROS_INFO("Richiesta di riposizionarmi in X=(%f) Y=(%f) th=(%f)", req.x, req.y, req.th);
        x=req.x;
        y=req.y;
        th=req.th;
        return true;
    }



};


void dynrec(project1::integrationConfig &config, uint32_t level, Odometry *o) {  //code to adapt to parameter change
        ROS_INFO("Reconfigure Request: %d", config.integration);
            o->setIntegrationType((int) config.integration);
        ROS_INFO ("%d",level);
}






int main(int argc, char **argv) {
    ros::init(argc, argv, "my_odom");
    ros::NodeHandle node;
    Odometry *my_odometry=NULL;
    my_odometry = new Odometry(node);
  
    
    dynamic_reconfigure::Server<project1::integrationConfig> server;  //defining the obj that will deal with listener parameters changing
    dynamic_reconfigure::Server<project1::integrationConfig>::CallbackType f;  // obj of type callback to wrap your function. Uses boost library

    //dynamic reconfigure part
    f = boost::bind(&dynrec, _1, _2, my_odometry);  //2 arguments
    server.setCallback(f);  // passed to the server



    ros::spin();

    return 0;
}

