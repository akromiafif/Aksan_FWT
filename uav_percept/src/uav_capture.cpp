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

int main(int argc, char** argv) {
  // cv::namedWindow("Video Original", cv::WINDOW_AUTOSIZE );

  ros::init(argc, argv, "uav_percept_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh); 
  image_transport::Publisher itPublisher = it.advertise("camera/image", 1);
  sensor_msgs::ImagePtr msg;

  cv::VideoCapture capture(0); 


  double fps = capture.get( cv::CAP_PROP_FPS );
  cv::Size size(
    (int)capture.get( cv::CAP_PROP_FRAME_WIDTH ),
    (int)capture.get( cv::CAP_PROP_FRAME_HEIGHT )
  );

  if(!capture.isOpened()) return 1;

  cv::VideoWriter writer;
  writer.open("output.avi", VideoWriter::fourcc('M','J','P','G'), fps, size);
  cv::Mat frame; 

  while (nh.ok()) {
    capture >> frame; 

    // if (!frame.empty()) {
      //imshow("Video Original", frame);
      // writer << frame;

      // Save image frame by frame
      // cv::imwrite("romi" + std::to_string(rand()) + ".jpg", frame);

      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      itPublisher.publish(msg);
      cv::waitKey(1);
    // }

    ros::Rate rate(30);
    ros::spinOnce();
    rate.sleep();
  }
}
































// #include <opencv2/opencv.hpp>
// #include <iostream>

// using namespace cv;

// int main( int argc, char* argv[] ) {
//   cv::namedWindow( "Output", cv::WINDOW_AUTOSIZE );


//   cv::VideoCapture capture(0);
// //   capture.set(cv::CAP_PROP_FRAME_WIDTH, 320);
// //   capture.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

//   double fps = capture.get( cv::CAP_PROP_FPS );
//   cv::Size size(
//     (int)capture.get( cv::CAP_PROP_FRAME_WIDTH ),
//     (int)capture.get( cv::CAP_PROP_FRAME_HEIGHT )
//   );

//   cv::VideoWriter writer;
//   writer.open("output.avi", VideoWriter::fourcc('M','J','P','G'), fps, size );
//   cv::Mat bgr_frame;
  
//   for(;;) {
//     capture >> bgr_frame;
//     if( bgr_frame.empty() ) break;

//     cv::imshow( "Output", bgr_frame );
//     writer << bgr_frame;
    
//     char c = cv::waitKey(10);
//     if( c == 27 ) break;
//   }
  
//   capture.release();
// }