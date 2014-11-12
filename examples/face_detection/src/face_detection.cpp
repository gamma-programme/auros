#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class FaceDetect
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier eyes_cascade;
  
  public:
    FaceDetect()
      : it_(nh_)
    {
      // Subscrive to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/sensors/eo/visible/image_raw", 1, 
        &FaceDetect::imageCb, this);
        
      image_pub_ = it_.advertise("/apps/face_detection/monitor", 1);
      
      if(!face_cascade.load( "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"))
      { 
        ROS_ERROR("Error loading face cascade");
      }
      if(!eyes_cascade.load("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml"))
      { 
        ROS_ERROR("Error loading eye cascade");
      }
    }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
      detectAndDisplay(cv_ptr);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  void detectAndDisplay(cv_bridge::CvImagePtr cv_ptr)
  {
    std::vector<cv::Rect> faces;

    cv::Mat frame_gray;

    cv::cvtColor( cv_ptr->image, frame_gray, CV_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );

    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

    for( size_t i = 0; i < faces.size(); i++ )
    {
      cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
      cv::ellipse( cv_ptr->image, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );
      
      cv::Mat faceROI = frame_gray( faces[i] );
      std::vector<cv::Rect> eyes;

      //-- In each face, detect eyes
      eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

      for( size_t j = 0; j < eyes.size(); j++ )
      {
        cv::Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
        int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
        cv::circle( cv_ptr->image, center, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
      }
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_detection");
  FaceDetect ic;
  ROS_INFO("Setup complete");
  ros::spin();
  return 0;
}
