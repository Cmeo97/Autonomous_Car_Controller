#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <prius_msgs/Control.h>


double linear_x=0,angular_z=0; // linear motion along x axis, angular motion around z axis
double steer, throttle;
prius_msgs::Control ctr;
int set_gears;
ros::Publisher pub;

void poseMessageReceived(const geometry_msgs::Twist &msg){
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "linear_velocity= (" << msg.linear.x << "), angular_velocity = (" << msg.angular.z << ")");

	linear_x = msg.linear.x;
	angular_z = msg.angular.z;

	//Changing gears
	if(linear_x!=0){
		ctr.throttle=throttle;
	}
	else{
		ctr.throttle=0;
	}
	//Forward
	if (linear_x>0){
		ctr.shift_gears = 2;
	}
	//Backward
	else if(linear_x<0){
		ctr.shift_gears = 3;
	}
	//Turning left
	if(angular_z>0){
		ctr.steer = steer;
	}
	//Turning right
	else if(angular_z<0){
		ctr.steer = -steer;
	}
	else{
		ctr.steer = 0;
	}

	ros::NodeHandle nh2;
	pub = nh2.advertise<prius_msgs::Control>("/prius",1000);
	
	ROS_INFO_STREAM("throttle= ("<< ctr.throttle<<"), steer: ("<< ctr.steer << ")");
        pub.publish(ctr);
}	

int main(int argc, char **argv){
	ros::init(argc, argv, "simple_control_node");
	ros::NodeHandle nh;
	ros::param::set("throttle_param",2);
	ros::param::set("steer_param",1);

	bool flag_throttle = ros::param::get("throttle_param",throttle);
	bool flag_steer = ros::param::get("steer_param",steer);
	if(!flag_throttle || !flag_steer){
		ROS_FATAL_STREAM("Errors in the parameters");
		exit(1);
	}

	// initial changes to the throttle in case of severe differences
	if(throttle<0 || throttle>1){
		ROS_WARN_STREAM("Throttle must be between 0 and 1!");
		if(throttle<0){
			throttle = -throttle;
		}
		if(throttle>1){
			throttle = 1;
		}

		// and set the throttle parameter
		ros::param::set("throttle_param", throttle);
        }
	// initial changes to the steering in case of severe differences
	if(steer<0 || steer>1){
		ROS_WARN_STREAM("Steer must be between 0 and 1!");

		if(steer<0){
			steer = -steer;
		}
		if(steer>1){
			steer = 1;
		}

		// and set the steer parameter
		ros::param::set("steer_param", steer);
	}

	ros::Subscriber sub = nh.subscribe("turtle1/cmd_vel", 1000, &poseMessageReceived);
	ros::spin();
}
