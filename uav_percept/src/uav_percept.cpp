#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>

// Import header files
#include <uav_percept.hpp>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;


namespace uav_percept {
  UAVPercept::UAVPercept(ros::NodeHandle* node) {
    //Create viewable window - Image overlay output
    cv::namedWindow(OPENCV_WINDOW, WINDOW_AUTOSIZE);

    //Begin window view and image display
    cv::startWindowThread(); 

    //Define source of image
    image_transport::ImageTransport it(*node); 

    // VARIABLE BUAT SIMPEN VIDEO
    cv::Size size(
      (int) 640,
      (int) 360
    );

    counter = 0;

    writer.open("vision_detected.avi", VideoWriter::fourcc('M','J','P','G'), 30, size);
    // VARIABLE BUAT SIMPEN VIDEO

    // VARIABLE BUAT SIMPEN Lat Long
    latLongDroppingZone.open("latLongDropZone.txt");
    // VARIABLE BUAT SIMPEN Lat Long

    // //Image subscriber to /camera/image topic
    itSubscriber = it.subscribe(IMAGE_TOPIC, 1000, &UAVPercept::improCB, this); 

    //VFR_HUD Subscriber
    vfrHUDSubscriber = node->subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud", 1000, &UAVPercept::vfrCB, this);

    //GPS Subscriber
    nsfSubscriber = node->subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 1000, &UAVPercept::nsfCB, this);

    //Altitude Subscriber
    altSubscriber = node->subscribe<std_msgs::Float64>("/mavros/global_position/rel_alt", 1000, &UAVPercept::altCB, this); 

    //LapInfo Subscriber
    lapInfoSubscriber = node->subscribe<uav_commander::lap_info>("/lap_info", 1000, &UAVPercept::lapInfoCB, this); 

    //improInfo Subscriber
    improInfoSubscriber = node->subscribe<uav_commander::impro_info>("/impro_info", 1000, &UAVPercept::improInfoCB, this);
  }

  UAVPercept::~UAVPercept() {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void UAVPercept::vfrCB(const mavros_msgs::VFR_HUD::ConstPtr& msg) {
    vfrHUD = *msg;
  }

  void UAVPercept::nsfCB(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    GPS = *msg;
  }

  void UAVPercept::altCB(const std_msgs::Float64::ConstPtr& msg) {
    Alt = *msg;
  }

  void UAVPercept::lapInfoCB(const uav_commander::lap_info::ConstPtr& msg) {
    lapInfo = *msg;
  }

  void UAVPercept::improInfoCB(const uav_commander::impro_info::ConstPtr& msg) {
    improInfo = *msg;
  }


  void UAVPercept::improCB(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    // if (lapInfo.lap_one.data) {
    //   ROS_INFO("Lap 1 Completed");
    // }

    // if (lapInfo.lap_two.data) {
    //   ROS_INFO("Lap 2 Completed");
    // }

    // if (lapInfo.lap_three.data) {
    //   ROS_INFO("Lap 3 Completed");
    // }

    if (improInfo.impro_enabled.data) {

      latLongAircraft.open("latLongAircraft_" + std::to_string(counter) + std::string(".txt"));

      testWrite.open("testWrite.txt");

      try {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

        latRef = GPS.latitude * (pi/180); //Reference latitude (current latitude of aircraft)
        longRef = GPS.longitude * (pi/180); //Reference longitude (current longitude of aircraft)
        headingCurrent = vfrHUD.heading; //Current heading
        altCurrent = Alt.data; //Current altitude
        
        //Convert image from msg to BGR format
        Mat BGR = cv_ptr->image;

        //Make copy of image for processing
        Mat orig_image = BGR.clone(); 
        
        //Introduce blur for image flitering and converting
        medianBlur(BGR, BGR, 3); 
        
        // Convert input image to HSV
        Mat HSV;
        //Convert BGR image to HSV image
        cvtColor(BGR, HSV, COLOR_BGR2HSV); 

        //Hough transform to detect circles
        //Detected circles array
        vector<Vec3f> circles;

        // CUSTOM FOR RED RANGE //
        // Threshold the HSV image, keep only the red pixels
        Mat lower_hue;
        Mat upper_hue;

        //Lower hue threshold range
        inRange(HSV, Scalar(0, 170, 160), Scalar(10, 255, 255), lower_hue); 

        //Upper hue threshold range
        inRange(HSV, Scalar(165, 100, 100), Scalar(180, 255, 255), upper_hue); 
        
        // Combine the above two images
        Mat hue_image;

        //Combine hue images for object recognition
        cv::addWeighted(lower_hue, 1.0, upper_hue, 1.0, 0.0, hue_image); 
        
        //Introduce interference
        //Introduce noise
        cv::GaussianBlur(hue_image, hue_image, Size(9, 9), 2, 2);

        //HOUGH CIRCLE TRANSFORNATION
        cv::HoughCircles(hue_image, circles, CV_HOUGH_GRADIENT, 1, hue_image.rows/4, 100, 25, 25, 100); 
        // imshow("Original", orig_image);
        // CUSTOM FOR RED RANGE //

        // ============== CUSTOM FOR PINK RANGE ============== //
        Mat lower_pink;

        cv::inRange(HSV, cv::Scalar(135, 40, 160), cv::Scalar(160, 255, 255), lower_pink);

        cv::GaussianBlur(lower_pink, lower_pink, Size(9, 9), 2, 2);

        cv::HoughCircles(lower_pink, circles, CV_HOUGH_GRADIENT, 1, lower_pink.rows/4, 100, 25, 25, 100); 
        // ============== CUSTOM FOR PINK RANGE ============== //
        

        // Highlight detected object
        for(size_t cur = 0; cur < circles.size(); ++cur) {
          //Define centre point of detected circle
          Point center(circles[cur][0], circles[cur][1]);
          //Define radius of detected circle
          int radius = circles[cur][2]; 
          
          //Overlay detected cricle outline onto origional image
          circle(orig_image, center, radius, Scalar(239, 152, 38), 2); 
          
          //allow for display of image for given milliseconds (Image overlay refreshrate)
          waitKey(10);  
        }

        // DISABLE KALO MODE FLIGHT
        // cv::imshow(OPENCV_WINDOW, orig_image);
        // DISABLE KALO MODE FLIGHT

        writer << orig_image;

        if (circles.size() != 0) {
          //Target bearing
          bearingDropZone = 0;
          x = circles[0][0] - (orig_image.cols / 2); //x pixel coordinate
          y = circles[0][1] - (orig_image.rows / 2); //y pixel coordinate
          // xm = (abs(x)/(orig_image.cols/2)) * (altCurrent*tan(pi/6)); //x displacement in meters
          // ym = (abs(y)/(orig_image.rows/2)) * (altCurrent*tan(pi/8)); //y displacement in meters

          xm = (abs(x)/(orig_image.cols/2)) * (altCurrent*tan(53.13010236)); //x displacement in meters
          ym = (abs(y)/(orig_image.rows/2)) * (altCurrent*tan(33.3985)); //y displacement in meters

          /* Region designation - (0,0) in top-left corner of 2
          2|1
          ---
          3|4
          */
          
          //Determine bearing of target based on image segment
          // region 2 and 3
          if(circles[0][0] >= 0 && circles[0][0] <= (orig_image.cols / 2))  {
            if(circles[0][1] >= 0 && circles[0][1] <= (orig_image.rows /2)) { //region 2
              bearingDropZone = headingCurrent + atan(abs(x)/abs(y));
            } else { //region 3
              bearingDropZone = headingCurrent + (180 - atan(abs(x)/abs(y)));
            }
          } else { //region 1 and 4
            if(circles[0][1] > (orig_image.rows / 2)) { //region 4
              bearingDropZone = headingCurrent + 180 + atan(abs(x)/abs(y));
            } else { //region 1
              bearingDropZone = headingCurrent - atan(abs(x)/abs(y));
            }
          }

          //Bearing regression (0 - 359)
          if(bearingDropZone >= 360) {
            bearingDropZone = bearingDropZone - 360;
          } else if(bearingDropZone < 0) {
            bearingDropZone = bearingDropZone + 360;
          }

          GPS.position_covariance_type = 2; //Type of GPS frame reference (3 - Covariance known)
          d = sqrt(pow(abs(xm), 2) + pow(abs(ym), 2)) / 1000; //magnitude of vector to target location in km
          R = 6378.1; //Radius of the Earth (km)
          bearing = bearingDropZone * pi/180; //Bearing upward (0 degrees -> radians)

          float x_lat = (asin(sin(latRef)*cos(d/R) + cos(latRef)*sin(d/R)*cos(bearing))) * (180/pi); //Target Latitude
          float y_long = (longRef + atan2(sin(bearing)*sin(d/R)*cos(latRef), cos(d/R) - sin(latRef)*sin(x_lat))) * (180/pi); //Target Longitude
          
          // Display target details deduced
          ROS_INFO("LatDropZone: %f", x_lat); //Target latitude
          ROS_INFO("LongDropZone: %f", y_long); //Target Longitude
          ROS_INFO("LatAircraft: %f", GPS.altitude); //Reference latitude
          ROS_INFO("LongAircraft: %f", GPS.longitude); //Reference longitude
          ROS_INFO("xm: %f", xm); //X axis displacement (km)
          ROS_INFO("ym: %f", ym); //Y axis displacment (km)
          ROS_INFO("displacement: %f", d); //Total displacment (km)
          ROS_INFO("bearing: %f", bearingDropZone); //Bearing
          ROS_INFO("altitude: %f", altCurrent);
          ROS_INFO("Red circle detected");

          testWrite << "Hello from testWrite" << "\n";
          testWrite.close();

          if (latLongDroppingZone.is_open() || latLongAircraft.is_open()) {
            latLongDroppingZone << "LatDropZone: " + std::to_string(x_lat) + " | LongDropZone: " + std::to_string(y_long) + " \n";

            latLongAircraft << "LatAircraft: " + std::to_string(GPS.latitude) + " | LongAircraft: " + std::to_string(GPS.longitude) + " \n";

            counter++;
            latLongAircraft.close();
            latLongDroppingZone.close();
          }
        }
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }
  }
}