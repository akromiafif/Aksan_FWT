#include <ros/ros.h> 
#include <image_transport/image_transport.h> 
#include <opencv2/highgui/highgui.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <sstream>

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

  // Obtain frame size information using get() method
  // int frame_width = static_cast<int>(cap.get(3));
  // int frame_height = static_cast<int>(cap.get(4));
  // cv::Size frame_size(frame_width, frame_height);
  // int fps = 20;

  // VideoWriter output("output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'),fps, frame_size);

  while (nh.ok()) {
    cap >> frame; 
    // output.write(frame);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    itPublisher.publish(msg);
    cv::waitKey(1);
    
    ros::Rate rate(30);
    ros::spinOnce();
    rate.sleep();
  }
}