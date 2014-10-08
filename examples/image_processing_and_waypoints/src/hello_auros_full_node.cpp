#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <mavros/Waypoint.h>
#include <mavros/WaypointList.h>

ros::Publisher image_pub;
std::vector<mavros::Waypoint> waypoints;

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    {
      cv::putText(cv_ptr->image, , 
        cv::Point(cv_ptr->image.cols/2, 
        cv_ptr->image.rows/2), 
        cv::FONT_HERSHEY_SIMPLEX, 
        2.0, 
        CV_RGB(0,0,255));
    }
    
    // Output modified video stream
    image_pub.publish(cv_ptr->toImageMsg()); 
}

void wp_cb(const mavros::WaypointList& wp_list)
{
  waypoints = wp_list.waypoints;
  
  for(int i = 0; i < waypoints.size(); i++) 
  {
    ROS_INFO("Waypoint loaded @ %f, %f", waypoints[i].x_lat, waypoints[i].y_long);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_painter");
  ros::NodeHandle nh;
  ros::Subscriber wp_sub = nh.subscribe("/mavros/mission/waypoints", 1, wp_cb);
  ros::Subscriber image_sub = nh.subscribe("/v4l/camera/image_raw", 1, image_cb);
  image_pub = nh.advertise<sensor_msgs::Image>("/image_converter/output_video", 1);
  ROS_INFO("Setup complete");
  ros::spin();
  return 0;
}