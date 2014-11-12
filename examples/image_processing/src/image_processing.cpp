// Include the ROS C++ API
#include <ros/ros.h>
// Include the ROS image transport library, which provides the ImageTransport type
#include <image_transport/image_transport.h>
// Include cv_bridge, which converts ROS images to OpenCV format
#include <cv_bridge/cv_bridge.h>
// Specifies the format for the OpenCV image
#include <sensor_msgs/image_encodings.h>
// Include some OpenCV basics
#include <opencv2/imgproc/imgproc.hpp>

/**
  This example uses a container class to keep the code modular.
  The equivalent functional version would be simpler.
*/
class ImageConverter
{
  // A NodeHandle is used to access resources from other ROS nodes
  ros::NodeHandle node_handle;
  // The image_transport knows how to speak ROS images
  image_transport::ImageTransport image_transport;
  // The subscription object, which can be used to change our subscription later on
  image_transport::Subscriber image_sub;
  // A publishing object, which we use to publish our processed images
  image_transport::Publisher image_pub;
  
public:
  ImageConverter()
    : image_transport(node_handle) // Member initialisation
  {
    // Subscribe to input video feed and publish output video feed
    // We provide a class local callback, along with the class.
    image_sub = image_transport.subscribe("/sensors/eo/visible/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub = image_transport.advertise("/apps/image_processing/output", 1);
  }

  // As we haven't specified an update rate, our callback will be called for every image delivered
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // Pointer to an opencv image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      // Full copy as we are modifying. Alternatively we could have a read only version with the copy overhead
      // BGR8 is the standard OpenCV colour image format expected by most functions
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Check that the image is properly formed
    if (cv_ptr->image.rows > 1 && cv_ptr->image.cols > 1)
      // Execute a basic OpenCV function on the image
      cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2), 100, CV_RGB(255,0,0));
    
    // Output modified video stream
    image_pub.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ROS_INFO("Setup complete");
  ros::spin();
  return 0;
}
