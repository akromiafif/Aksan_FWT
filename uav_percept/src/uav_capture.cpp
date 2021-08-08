#include <ros/ros.h> 
#include <image_transport/image_transport.h> 
#include <opencv2/highgui/highgui.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <sstream>

// For custom message
#include <uav_commander/impro_info.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
  ros::init(argc, argv, "uav_percept_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh); 
  image_transport::Publisher itPublisher = it.advertise("camera/image", 1);
  cv::VideoCapture cap(0); 
  sensor_msgs::ImagePtr msg;

  if(!cap.isOpened()) return 1;
  cv::Mat frame; 

  while (nh.ok()) {
    cap >> frame; 
    //imshow("Video Test", frame);

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    itPublisher.publish(msg);
    cv::waitKey(1);
    
    ros::Rate rate(30);
    ros::spinOnce();
    rate.sleep();
  }
}