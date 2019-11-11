#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ricoh_camera {

class ImageConverter : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
  {
  }

  ~ImageConverter()
  {
  }

  void onInit()
  {
    ros::NodeHandle &nh         = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    it_.reset(new image_transport::ImageTransport(nh)); 

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_->subscribe("image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_->advertise("image_processed", 1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //cv_bridge::CvImagePtr cv_ptr;
    try
    {
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      const cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;   
      if (image.rows > 60 && image.cols > 60)
        cv::circle(image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
	  sensor_msgs::ImagePtr new_msg = cv_bridge::CvImage(msg->header, msg->encoding, image).toImageMsg();
  	  image_pub_.publish(new_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

} //namespace ricoh_camera

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ricoh_camera::ImageConverter, nodelet::Nodelet)

