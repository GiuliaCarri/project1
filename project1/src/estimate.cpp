#include "ros/ros.h"
#include "project1/MotorSpeed.h" /* Collegamento al nuovo tipo del messaggio*/
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <geometry_msgs/TwistStamped.h>
#include <math.h>

#define _USE_MATH_DEFINES
#define STEERING_FACTOR 18.0
#define BASELINE 1  //real baseline = 0.583 m
#define INTERAXIS 1.765
#define CIRC 0.9896  //2*pi*R, where R = 0.1575 m
#define GR 390 //per ogni giro del motore la ruota fa 1/35 - 1/40 giri

typedef message_filters::sync_policies::ApproximateTime<project1::MotorSpeed, project1::MotorSpeed, project1::MotorSpeed, project1::MotorSpeed> MySyncPolicy;


class Estimator{

    private:

	double vx;
	double vy;
	double vth;
	double motor_s_fl;
	double motor_s_fr;
	double motor_s_rl;
	double motor_s_rr;

	ros::NodeHandle n; 
  	ros::Publisher pub; 
  	ros::Timer timer1; 
  	ros::Timer timerSync;
	ros::Subscriber sub_fl;
	ros::Subscriber sub_fr;
	ros::Subscriber sub_rl;
	ros::Subscriber sub_rr;

	
    geometry_msgs::TwistStamped msg;


	public:

    Estimator(ros::NodeHandle node){

        vx =0;
        vth=0;
		n = node;
/*
        //subscriber part: message filters
        //message_filters::Subscriber<project1::MotorSpeed> sub_fl(n,  "/motor_speed_fl", 1);	
	    //message_filters::Subscriber<project1::MotorSpeed> sub_fr(n,  "/motor_speed_fr", 1);
	    //message_filters::Subscriber<project1::MotorSpeed> sub_rl(n,  "/motor_speed_rl", 1);
	    //message_filters::Subscriber<project1::MotorSpeed> sub_rr(n,  "/motor_speed_rr", 1);	
	
  	    sub_fl= n.subscribe("/motor_speed_fl", 1, &Estimator::callback_s_fl, this);
  	    sub_fr= n.subscribe("/motor_speed_fr", 1, &Estimator::callback_s_fr, this);
  	    sub_rl= n.subscribe("/motor_speed_rl", 1, &Estimator::callback_s_rl, this);
  	    sub_rr= n.subscribe("/motor_speed_rr", 1, &Estimator::callback_s_rr, this);

	
    	timerSync = n.createTimer(ros::Duration(1), &Estimator::callback_sync, this);

 		
	    //message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_fl, sub_fr, sub_rl, sub_rr);
  	    //sync.registerCallback(boost::bind(&Estimator::callback_s, this, _1, _2, _3, _4));
*/	
        //publisher part

		pub = n.advertise<geometry_msgs::TwistStamped>("/rechatterVel", 1);
    	timer1 = n.createTimer(ros::Duration(1), &Estimator::callback_p, this);
        
    }

/*	void callback_s_fl(const project1::MotorSpeed::ConstPtr& speedMotor_fl){
		motor_s_fl=speedMotor_fl->rpm;
	}
	
	void callback_s_fr(const project1::MotorSpeed::ConstPtr& speedMotor_fr){
		motor_s_fr=speedMotor_fr->rpm;
	}
	void callback_s_rl(const project1::MotorSpeed::ConstPtr& speedMotor_rl){
		motor_s_rl=speedMotor_rl->rpm;
	}
	
	void callback_s_rr(const project1::MotorSpeed::ConstPtr& speedMotor_rr){
		motor_s_rr=speedMotor_rr->rpm;
	}

	void callback_sync(const ros::TimerEvent&) {
       
		computeVel(motor_s_fl, motor_s_fr, motor_s_rl, motor_s_rr);
		ROS_INFO ("Front-left: ( %f ) Front-right: ( %f ) Rear-left: ( %f ) Rear-right: ( %f )", motor_s_fl, motor_s_fr, motor_s_rl, motor_s_rr);
	}
*/
	/*void callback_s(const project1::MotorSpeed::ConstPtr& speedMotor_fl, 
		      const project1::MotorSpeed::ConstPtr& speedMotor_fr,
		      const project1::MotorSpeed::ConstPtr& speedMotor_rl, 
		      const project1::MotorSpeed::ConstPtr& speedMotor_rr) {
        ROS_INFO_STREAM ("sono nella s");
	    ROS_INFO ("Front-left: ( %f ) Front-right: ( %f ) Rear-left: ( %f ) Rear-right: ( %f )", 
	      speedMotor_fl->rpm, speedMotor_fr->rpm, speedMotor_rl->rpm, speedMotor_rr->rpm);
	    computeVel(speedMotor_fl->rpm, speedMotor_fr->rpm, speedMotor_rl->rpm, speedMotor_rr->rpm);
	}*/

	void computeVel(double rpm_fl, double rpm_fr, double rpm_rl, double rpm_rr){
	    double right_wheel_speed, left_wheel_speed;
	    left_wheel_speed = (-(rpm_fl + rpm_rl)/2) * CIRC / GR; /*la velocità delle ruote di sinistra è uguale alla media dei giri delle ruote per un fattore di conversione*/
	    right_wheel_speed = ((rpm_fr + rpm_rr)/2) * CIRC / GR;

        //skid-steering velocity
        vx = ((left_wheel_speed + right_wheel_speed)/2);
	    vy = (0);
	    vth = ((right_wheel_speed - left_wheel_speed)/ (BASELINE)); /*(vr-vl)/(2*lunghezza_asse)*/
        //ROS_INFO_STREAM ("sono nella vel");
	}

	void callback_p(const ros::TimerEvent&) {
       
		msg.twist.linear.x = vx;
		msg.twist.angular.z = vth;

		pub.publish(msg);

		ROS_INFO_STREAM("Wheels' velocity:"<<" linear="<<msg.twist.linear.x<<" angular="<<msg.twist.angular.z);
	}
    

	/*double getVx() const { return vx; }
	void setVx(double vx_) { vx = vx_; }
	double getVy() const { return vy; }
	void setVy(double vy_) { vy = vy_; }
	double getVth() const { return vth; }
	void setVth(double vth_) { vth = vth_; }*/


};

void callback_s(const project1::MotorSpeed::ConstPtr& speedMotor_fl, const project1::MotorSpeed::ConstPtr& speedMotor_fr, const project1::MotorSpeed::ConstPtr& speedMotor_rl, const project1::MotorSpeed::ConstPtr& speedMotor_rr, Estimator *robot) {
	   // ROS_INFO ("Front-left: ( %f ) Front-right: ( %f ) Rear-left: ( %f ) Rear-right: ( %f )", speedMotor_fl->rpm, speedMotor_fr->rpm, speedMotor_rl->rpm, speedMotor_rr->rpm);
	    robot->computeVel(speedMotor_fl->rpm, speedMotor_fr->rpm, speedMotor_rl->rpm, speedMotor_rr->rpm);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "subBag_pubVelo");
  ros::NodeHandle node;
  Estimator *my_estimator=NULL;
  my_estimator = new Estimator(node);
  

  message_filters::Subscriber<project1::MotorSpeed> sub_fl(node,  "/motor_speed_fl", 1);	/*mi "iscrivo ai 4 topic*/
  message_filters::Subscriber<project1::MotorSpeed> sub_fr(node,  "/motor_speed_fr", 1);
  message_filters::Subscriber<project1::MotorSpeed> sub_rl(node,  "/motor_speed_rl", 1);
  message_filters::Subscriber<project1::MotorSpeed> sub_rr(node,  "/motor_speed_rr", 1);	
	
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_fl, sub_fr, sub_rl, sub_rr);
  sync.registerCallback(boost::bind(&callback_s, _1, _2, _3, _4, my_estimator));
  ros::spin();
  return 0;
}