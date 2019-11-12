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

class EqRect : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_frnt_;
  image_transport::Publisher image_back_;
  
public:

  void onInit()
  {
    ros::NodeHandle &nh         = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    it_.reset(new image_transport::ImageTransport(nh)); 

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_->subscribe("image_raw", 1, &EqRect::imageCb, this);
    image_frnt_ = it_->advertise("front/image_raw", 1);
    image_back_ = it_->advertise("back/image_raw", 1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
		const cv::Mat image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;   
		const cv::Mat img_frnt = image( cv::Rect(0, 0, 640, 640) );
		const cv::Mat img_back = image( cv::Rect(640, 0, 640, 640) );
		
		cv::rotate(img_frnt, img_frnt, cv::ROTATE_90_CLOCKWISE);
		cv::rotate(img_back, img_back, cv::ROTATE_90_COUNTERCLOCKWISE);
		
		sensor_msgs::ImagePtr frnt_msg = cv_bridge::CvImage(msg->header, msg->encoding, img_frnt).toImageMsg();
		sensor_msgs::ImagePtr back_msg = cv_bridge::CvImage(msg->header, msg->encoding, img_back).toImageMsg();
		image_frnt_.publish(frnt_msg);
		image_back_.publish(back_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  }
};

} //namespace ricoh_camera

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ricoh_camera::EqRect, nodelet::Nodelet)

