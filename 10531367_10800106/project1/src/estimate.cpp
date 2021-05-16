#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h" /* Collegamento al tipo di messaggio nel package robotics_hw1*/
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <geometry_msgs/TwistStamped.h>
#include <math.h>

#define _USE_MATH_DEFINES
#define BASELINE 1.2  //real baseline = 0.583 m
#define CIRC 0.9896  //2*pi*R, where R = 0.1575 m
#define GR 247.57 //per ogni giro del motore la ruota fa 1/35 - 1/40 giri

typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> MySyncPolicy;


class Estimator{

    private:

	double vx;
	double vy;
	double vth;

	double motor_s_fl;
	double motor_s_fr;
	double motor_s_rl;
	double motor_s_rr;

	double right_wheel_speed;
	double left_wheel_speed;

	ros::NodeHandle n;
 
  	ros::Timer timer;
	
	ros::Publisher pub;

	//subscriber per ogni ruota/motore
	ros::Subscriber sub_fl;
	ros::Subscriber sub_fr;
	ros::Subscriber sub_rl;
	ros::Subscriber sub_rr;

	
    geometry_msgs::TwistStamped msg;


	public:

    Estimator(ros::NodeHandle node){

        vx = 0;
        vth = 0;
		n = node;
	
        //publisher part

		pub = n.advertise<geometry_msgs::TwistStamped>("/rechatterVel", 1);
     	timer = n.createTimer(ros::Duration(0.5), &Estimator::callback_p, this);
        
    }


	void computeVel(double rpm_fl, double rpm_fr, double rpm_rl, double rpm_rr){
	    
	    left_wheel_speed = (-(rpm_fl + rpm_rl)/2) * CIRC / GR; /*la velocità delle ruote di sinistra è uguale alla media dei giri delle ruote per un fattore di conversione CIRC/GR*/
	    right_wheel_speed = ((rpm_fr + rpm_rr)/2) * CIRC / GR;

        //skid-steering velocity
        vx = ((left_wheel_speed + right_wheel_speed)/2);
	    vy = (0);
	    vth = ((right_wheel_speed - left_wheel_speed)/ (BASELINE)); /*(vr-vl)/(2*lunghezza_asse)*/

        //ROS_INFO_STREAM ("sono nella computeVel di estimate");
	}

	void callback_p(const ros::TimerEvent&) {
       
		msg.twist.linear.x = vx;
		msg.twist.angular.z = vth;

		pub.publish(msg);

		//ROS_INFO_STREAM("Wheels' velocity:"<<" linear="<<msg.twist.linear.x<<" angular="<<msg.twist.angular.z);
	}

};


void callback_s(const robotics_hw1::MotorSpeed::ConstPtr& speedMotor_fl, const robotics_hw1::MotorSpeed::ConstPtr& speedMotor_fr, const robotics_hw1::MotorSpeed::ConstPtr& speedMotor_rl, const robotics_hw1::MotorSpeed::ConstPtr& speedMotor_rr, Estimator *robot) {
	   
	//ROS_INFO ("Front-left: ( %f ) Front-right: ( %f ) Rear-left: ( %f ) Rear-right: ( %f )", speedMotor_fl->rpm, speedMotor_fr->rpm, speedMotor_rl->rpm, speedMotor_rr->rpm);
	    
	robot->computeVel(speedMotor_fl->rpm, speedMotor_fr->rpm, speedMotor_rl->rpm, speedMotor_rr->rpm);
}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "subBag_pubVel");
	ros::NodeHandle node;

	Estimator *my_estimator=NULL;
	my_estimator = new Estimator(node);
  
	//iscrizione ai 4 topic
	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_fl(node,  "/motor_speed_fl", 1);
	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_fr(node,  "/motor_speed_fr", 1);
  	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_rl(node,  "/motor_speed_rl", 1);
  	message_filters::Subscriber<robotics_hw1::MotorSpeed> sub_rr(node,  "/motor_speed_rr", 1);	
	
  	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_fl, sub_fr, sub_rl, sub_rr);
  	sync.registerCallback(boost::bind(&callback_s, _1, _2, _3, _4, my_estimator));


  	ros::spin();

  	return 0;

}