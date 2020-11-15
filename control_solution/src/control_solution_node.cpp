#include <ros/ros.h>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3DArray.h>
#include <prius_msgs/Control.h>

using namespace std;
	ros::Publisher pub;
	prius_msgs::Control ctr;

void pclDetectionReceived(const vision_msgs::Detection3DArray& detection){
 	float distances[sizeof(detection.detections)];
	for (unsigned i=0; i<sizeof(detection.detections); i++){
		// initially set the radius to 100m
		distances[i] = 100.0;
		float x = detection.detections[i].bbox.center.position.x;
		float y = detection.detections[i].bbox.center.position.y;
		// but if its positive, set it to the actual value
		// in this way, we dont consider far away but to the back
		if (x > 0){
			distances[i] = sqrt(pow(x,2) + pow(y,2));
		}
	}
	// size of the array
	const int N = sizeof(distances)/sizeof(float);
	// find the index of the smallest detection at the front
	int min_idx = std::distance(distances,std::min_element(distances, distances + N));
	// and determine subsequent x and y location
	float x = detection.detections[min_idx].bbox.center.position.x;
	float y = detection.detections[min_idx].bbox.center.position.y;
	float distance = distances[min_idx];
	if (x > 0 && distance < 4){
		// depending on the y-location, change the steering action
		if (y > 0){
			ctr.throttle = 1;
			ctr.steer = -1;
		}
		else{
			ctr.throttle = 1;
			ctr.steer = 1;
		}
	}
	// if there is no obstacle at the front and closer than 4 meters, just drive straight
	else{
		ctr.throttle = 1;
		ctr.steer = 0;
	}
	// publish the message
	ros::NodeHandle _nh3;
        pub =_nh3.advertise<prius_msgs::Control>("/prius",1);
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "throttle: " << ctr.throttle << ", steer: " << ctr.steer);
	pub.publish(ctr);
       }


void ocvDetectionReceived(const vision_msgs::Detection2DArray& detection)
{
// is someone is detected and the box area is greater than 25000 pixels, stop the car
	for(int i=0; i<detection.detections.size(); i++){
		if(detection.detections[i].bbox.size_x * detection.detections[i].bbox.size_y > 25000){

			ctr.throttle = 0;
			ctr.brake = 1;
			ctr.shift_gears = 0;
			ctr.steer = 0;
			
			ros::NodeHandle _nh4;
			pub =_nh4.advertise<prius_msgs::Control>("/prius",1000);
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "throttle: " << ctr.throttle << ", steer: " << ctr.steer);
			pub.publish(ctr);

			//ros::shutdown();
		}
	}

}





int main(int argc, char **argv)
{
  	ros::init(argc, argv, "control_solution_node");

	ros::NodeHandle _nh, _nh2;
  	ros::Subscriber sub_pcl = _nh2.subscribe("pcl_solution_node/detections",1,&pclDetectionReceived);
	ros::Subscriber sub_oc = _nh.subscribe("opencv_solution_node/detections",1,&ocvDetectionReceived);

  	ros::spin();
  return 0;
}
