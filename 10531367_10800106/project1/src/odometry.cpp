#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <project1/CustomOdom.h>
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
#define Ts 0.1


class Odometry{

    private:

	double x;
	double y;
	double th;

    double vl;  //linear velocity
    double va;  //angular velocity

	ros::NodeHandle n;

    //pub-sub, timer
    ros::Subscriber sub; 
  	ros::Publisher pub; 
    ros::Publisher c_pub;
  	ros::Timer timer;

    //services
    ros::ServiceServer riCentra;
    ros::ServiceServer riPosiziona;

    //odometry: msgs and other utilities
    ros::Time ct;
    tf2::Quaternion q;
    geometry_msgs::Quaternion qm;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    nav_msgs::Odometry odom;
    project1::CustomOdom custom_odom;

    //param reconfigure
    int integration_type; //0 -> Euler, 1 -> Runge-Kutta



	public:

    Odometry(ros::NodeHandle node){

        n = node;

        //static param configuration
        n.getParam("/Initial_x", x);
        n.getParam("/Initial_y", y);
        n.getParam("/Initial_th", th);

        //ROS_INFO ("Initial pose x: ( %f ) y: ( %f ) z: ( %f )", x, y, th);


        //subscriber part
        sub = n.subscribe("/rechatterVel", 1, &Odometry::callback_s, this);


        //publisher part
		pub = n.advertise<nav_msgs::Odometry>("/rechatterOdom", 1);
        c_pub = n.advertise<project1::CustomOdom>("/rechatterCustomOdom", 1);


        //service part
        riCentra = n.advertiseService("ricentra", &Odometry::ricentra, this);
        riPosiziona = n.advertiseService("riposiziona", &Odometry::riposiziona, this);
        
    }


    //param reconfigure: getter and setter of integration_type
    int getIntegrationType(){ 
        return integration_type; 
    }
    void setIntegrationType(int integrationType){ 
        integration_type = integrationType; 
    }
	

	void callback_s(const geometry_msgs::TwistStamped::ConstPtr& mt) {
        
        //ROS_INFO_STREAM ("sono nel callabak del subscriber di odometry");
        
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
        
        //ROS_INFO_STREAM ("sono nella euleroOdom");
        
        x = x + vl * Ts * cos(th);
	    y = y + vl * Ts * sin(th);
	    th = th + va * Ts;
        
    }

    void rkOdom(double vl, double va){
        
        //ROS_INFO_STREAM("sono nella rkOdom");
        
        x = x + vl * Ts * cos(th + va*Ts/2);
        y = y + vl * Ts * sin(th + va*Ts/2);
        th = th + va * Ts;
    }

	void publish() {
    	
        //ROS_INFO_STREAM("sono in publish di odometry");

        ct = ros::Time::now();
        q.setRPY(0, 0, th);
        qm = tf2::toMsg(q); //transform quaternion msgs from tf2 to geometry_msgs, so we can put it in TransformStamped msg


		//tf

        transformStamped.header.stamp = ct;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";

        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;
        
        transformStamped.transform.rotation = qm;


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

        custom_odom.odom.header.stamp = ct;
        custom_odom.odom.header.frame_id = "odom";
        custom_odom.odom.child_frame_id = "base_link";

        custom_odom.odom.pose.pose.position.x = x;
        custom_odom.odom.pose.pose.position.y = y;
        custom_odom.odom.pose.pose.position.z = 0.0;

        custom_odom.odom.pose.pose.orientation = qm;

        custom_odom.odom.twist.twist.linear.x = vl;
        custom_odom.odom.twist.twist.linear.y = 0.0;
        custom_odom.odom.twist.twist.angular.z = va;

        if (integration_type == 0)
            custom_odom.method.data = "euler";
        else
            if (integration_type == 1)
                custom_odom.method.data = "rk";
        

        //publish all

        br.sendTransform(transformStamped);
        //ROS_INFO_STREAM("tf:"<<" header time = "<<transformStamped.header.stamp<<" translation = "<<transformStamped.transform.translation<<" rotation = "<<transformStamped.transform.rotation);
	
		pub.publish(odom);
        //ROS_INFO_STREAM("odom:"<<" header frame = "<<odom.header.frame_id<<" position = "<<odom.pose.pose.position<<" orientation = "<<odom.pose.pose.orientation<<" linear vel = "<<odom.twist.twist.linear.x);
	
        c_pub.publish(custom_odom);
        //ROS_INFO_STREAM("c_odom:"<<" frame child = "<<custom_odom.odom.child_frame_id<<" position = "<<custom_odom.odom.pose.pose.position<<" orientation = "<<custom_odom.odom.pose.pose.orientation<<" angular vel = "<<odom.twist.twist.angular.z);

    }


    //service to reset odometry to (0,0)

    bool ricentra(project1::RiCentra::Request  &req, project1::RiCentra::Response  &resp)
    {
        //ROS_INFO("Richiesta di ricentrare l'odom");

        x=0;
        y=0;
        th=0;
        return true;
    }
    

    //service to reset odometry to any given pose (x,y,th)

    bool riposiziona(project1::RiPosiziona::Request  &req, project1::RiPosiziona::Response  &resp)
    {
        //ROS_INFO("Richiesta di riposizionarmi in X=(%f) Y=(%f) th=(%f)", req.x, req.y, req.th);
        x=req.x;
        y=req.y;
        th=req.th;
        return true;
    }



};


//code to adapt to parameter change

void dynrec(project1::integrationConfig &config, uint32_t level, Odometry *o) { 
        
    //ROS_INFO("Reconfigure Request: %d", config.integration);
    //ROS_INFO ("%d",level);

    o->setIntegrationType((int) config.integration);
        
}




int main(int argc, char **argv) {

    ros::init(argc, argv, "my_odom");
    ros::NodeHandle node;

    Odometry *my_odometry=NULL;
    my_odometry = new Odometry(node);
  
    //dynamic reconfigure for parameter integration
    dynamic_reconfigure::Server<project1::integrationConfig> server;    //defining the obj that will deal with listener parameters changing
    dynamic_reconfigure::Server<project1::integrationConfig>::CallbackType f;   //obj of type callback to wrap our function.

    f = boost::bind(&dynrec, _1, _2, my_odometry);
    server.setCallback(f);


    ros::spin();

    return 0;
}

