#include <ros/ros.h> 
#include <image_transport/image_transport.h> 
#include <opencv2/highgui/highgui.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

// For custom message
#include <uav_commander/impro_info.h>

using namespace std;
using namespace cv;

// OpenCV Window Name for imshow
static const std::string OPENCV_WINDOW = "Vision Original";

// Topics
static const std::string IMAGE_TOPIC = "/camera/image";

int main(int argc, char** argv) {
  ros::init(argc, argv, "uav_percept_publisher");
  ros::NodeHandle node;
  image_transport::ImageTransport it(node); 
  image_transport::Publisher itPublisher = it.advertise(IMAGE_TOPIC, 1);

  // GANTI INDEX KE 0 KALO MODE FLIGHT
  cv::VideoCapture capture(0, cv::CAP_V4L2);
  // GANTI INDEX KE 0 KALO MODE FLIGHT

  capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, 360);

  sensor_msgs::ImagePtr msg;

  cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);

  double fps = capture.get(cv::CAP_PROP_FPS);
  // cv::Size size(
  //   (int)capture.get( cv::CAP_PROP_FRAME_WIDTH ),
  //   (int)capture.get( cv::CAP_PROP_FRAME_HEIGHT )
  // );

  if(!capture.isOpened()) return 1;

  // BUAT SIMPEN VIDEO
  cv::Size size(
    (int) 640,
    (int) 360
  );

  cv::VideoWriter writer;
  writer.open("vision_original.avi", VideoWriter::fourcc('M','J','P','G'), 30, size);
  // BUAT SIMPEN VIDEO

  cv::Mat frame; 
  ros::Rate rate(30);

  while (node.ok()) {
    capture >> frame; 

    // DISABLE KALO MODE FLIGHT
    // imshow(OPENCV_WINDOW, frame);
    writer << frame;
    // DISABLE KALO MODE FLIGHT

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    itPublisher.publish(msg);
    cv::waitKey(1);
    
    ros::spinOnce();
    rate.sleep();
  }
}