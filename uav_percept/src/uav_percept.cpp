#include <uav_percept.hpp>



namespace uav_percept {
  UAVPercept::UAVPercept(ros::NodeHandle node) {
    //Create viewable window - Image overlay output
    cv::namedWindow("Vision Output", WINDOW_AUTOSIZE);

    //Begin window view and image display
    cv::startWindowThread(); 

    //Define source of image
    image_transport::ImageTransport it(node); 

    //Image subscriber to "camera/image" topic
    itSubscriber = it.subscribe("camera/image", 10, &UAVPercept::improCB, this); 
  }

  UAVPercept::~UAVPercept() {}

  void UAVPercept::improCB(const sensor_msgs::ImageConstPtr& msg) {
    try {
      //Convert image from msg to BGR format
      Mat BGR = cv_bridge::toCvShare(msg, "bgr8")->image; 
      //Make copy of image for processing
      Mat orig_image = BGR.clone(); 
      
      //Introduce blur for image flitering and converting
      medianBlur(BGR, BGR, 3); 
      
      // Convert input image to HSV
      Mat HSV;
      //Convert BGR image to HSV image
      cvtColor(BGR, HSV, COLOR_BGR2HSV); 
      
      // Threshold the HSV image, keep only the red pixels
      Mat lower_hue;
      Mat upper_hue;
      //Lower hue threshold range
      inRange(HSV, Scalar(0, 100, 100), Scalar(10, 255, 255), lower_hue); 
      //Upper hue threshold range
      inRange(HSV, Scalar(165, 100, 100), Scalar(179, 255, 255), upper_hue); 
      
      // Combine the above two images
      Mat hue_image;
      //Combine hue images for object recognition
      addWeighted(lower_hue, 1.0, upper_hue, 1.0, 0.0, hue_image); 
      
      // Introduce interference
      //Introduce noise
      GaussianBlur(hue_image, hue_image, Size(9, 9), 2, 2); 
      
      // Hough transform to detect circles
      //Detected circles array
      vector<Vec3f> circles;
      //HOUGH CIRCLE TRANSFORNATION
      HoughCircles(hue_image, circles, CV_HOUGH_GRADIENT, 1, hue_image.rows/8, 100, 25, 0, 0); 
      imshow("Original", orig_image);

      // Highlight detected object
      for(size_t cur = 0; cur < circles.size(); ++cur) {
        //Define centre point of detected circle
        Point center(circles[cur][0], circles[cur][1]);
        //Define radius of detected circle
        int radius = circles[cur][2]; 
        
        //Overlay detected cricle outline onto origional image
        circle(orig_image, center, radius, Scalar(239, 152, 38), 2); 

        //Display circle image overlay
        imshow("Vision Output", orig_image);
        //Define image size
        ROS_INFO("Size: (W) %i x (H) %i", orig_image.cols, orig_image.rows);

        //allow for display of image for given milliseconds (Image overlay refreshrate)
        waitKey(30);  
      }
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }
}