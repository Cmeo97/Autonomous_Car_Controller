#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>

using namespace std;
using namespace cv;

class Detector
{
  ros::NodeHandle _nh;
  image_transport::ImageTransport _it;
  // subscribe to the raw image data
  image_transport::Subscriber sub = _it.subscribe("/prius/front_camera/image_raw", 1, &Detector::imageCallback, this);
  // and we publish to the opencv_solution_node which will be used in the control_solution_node
  ros::Publisher pub = _nh.advertise<vision_msgs::Detection2DArray>("/opencv_solution_node/detections", 1);
  image_transport::Publisher imgpub = _it.advertise("/opencv_solution_node/visual", 1);

  HOGDescriptor _hog;


public:
  Detector() : _it(_nh)
  {
    _hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
  }
  vector<Rect> detect(InputArray img)
  {
    vector<Rect> h_det;
    _hog.detectMultiScale(img, h_det, 0, Size(8,8));
    return h_det;
  }
  ~Detector()
  {

  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    // we use cv_bridge in order to provide a translation of OpenCV to the ROS framework
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // conversion to Mat type
    Mat mat_image = cv_ptr->image;

    vision_msgs::Detection2DArray detections;
    // copy header to detections
    detections.header = msg->header;

    // human detection
    vector<Rect> h_det = detect(mat_image);
    for (int i = 0; i < h_det.size(); ++i)
    {
      // draw rectangle
      rectangle(mat_image, h_det[i], Scalar(0, 255, 0), 3, 8, 0);

      vision_msgs::Detection2D detection;
      detection.header = msg->header;
      // get the center of each image and the corresponding height and width
      detection.bbox.center.x = h_det[i].x;
      detection.bbox.center.y = h_det[i].y;
      detection.bbox.size_x = h_det[i].width;
      detection.bbox.size_y = h_det[i].height;
      // add the new detection to the 'detections' array
      detections.detections.push_back(detection);
    }

    // conversion
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_image).toImageMsg();

    // publish detection
    pub.publish(detections);
    // publish image
    imgpub.publish(image_msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opencv_solution");
  Detector detector;
  ros::spin();
  return 0;
}
