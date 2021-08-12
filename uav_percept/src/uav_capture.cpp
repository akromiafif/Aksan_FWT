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
  cv::VideoCapture capture(4, cv::CAP_V4L2);

  capture.set(cv::CAP_PROP_FRAME_WIDTH, 176);
  capture.set(cv::CAP_PROP_FRAME_HEIGHT, 144);

  sensor_msgs::ImagePtr msg;

  cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE );

  double fps = capture.get(cv::CAP_PROP_FPS);
  // cv::Size size(
  //   (int)capture.get( cv::CAP_PROP_FRAME_WIDTH ),
  //   (int)capture.get( cv::CAP_PROP_FRAME_HEIGHT )
  // );

  if(!capture.isOpened()) return 1;

  // BUAT SIMPEN VIDEO
  cv::Size size(
    (int) 176,
    (int) 144
  );

  cv::VideoWriter writer;
  writer.open("vision_original.avi", VideoWriter::fourcc('M','J','P','G'), 20, size);
  // BUAT SIMPEN VIDEO

  cv::Mat frame; 
  ros::Rate rate(30);

  while (node.ok()) {
    capture >> frame; 

    // DISABLE KALO MODE FLIGHT
    imshow(OPENCV_WINDOW, frame);
    writer << frame;
    // DISABLE KALO MODE FLIGHT

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    itPublisher.publish(msg);
    cv::waitKey(1);
    
    ros::spinOnce();
    rate.sleep();
  }
}












































// #include <ros/ros.h> 
// #include <image_transport/image_transport.h> 
// #include <opencv2/highgui/highgui.hpp> 
// #include <cv_bridge/cv_bridge.h>
// #include <sstream>
// #include <stdio.h>
// #include <stdlib.h>
// #include <iostream>
// #include <string>


// // For custom message
// #include <uav_commander/impro_info.h>

// using namespace std;
// using namespace cv;

// int main(int argc, char** argv) {
//   cv::namedWindow("Video Original", cv::WINDOW_AUTOSIZE );

//   ros::init(argc, argv, "uav_percept_publisher");
//   ros::NodeHandle nh;
//   image_transport::ImageTransport it(nh); 
//   image_transport::Publisher itPublisher = it.advertise("camera/image", 1);
//   sensor_msgs::ImagePtr msg;

//   cv::VideoCapture capture(0); 


//   double fps = capture.get( cv::CAP_PROP_FPS );
//   cv::Size size(
//     (int)capture.get( cv::CAP_PROP_FRAME_WIDTH ),
//     (int)capture.get( cv::CAP_PROP_FRAME_HEIGHT )
//   );

//   if(!capture.isOpened()) return 1;

//   cv::VideoWriter writer;
//   writer.open("output.avi", VideoWriter::fourcc('M','J','P','G'), fps, size);
//   cv::Mat frame; 

//   while (nh.ok()) {
//     capture >> frame; 

//     // if (!frame.empty()) {
//       imshow("Video Original", frame);
//       // writer << frame;

//       // Save image frame by frame
//       // cv::imwrite("romi" + std::to_string(rand()) + ".jpg", frame);

//       msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
//       itPublisher.publish(msg);
//       cv::waitKey(1);
//     // }

//     ros::Rate rate(30);
//     ros::spinOnce();
//     rate.sleep();
//   }
// }