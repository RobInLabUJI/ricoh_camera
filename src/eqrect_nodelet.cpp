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
  image_transport::Publisher image_rect_;
  
  cv::Mat map1_f;
  cv::Mat map1_b;
  cv::Mat map2_f;
  cv::Mat map2_b;
  
  cv::Mat f_mask;
  cv::Mat b_mask;
  cv::Mat intersect;
  cv::Mat not_intersect;  
  
public:

  void onInit()
  {
    ros::NodeHandle &nh         = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    it_.reset(new image_transport::ImageTransport(nh)); 

    image_sub_ = it_->subscribe("image_raw", 1, &EqRect::imageCb, this);
    image_rect_ = it_->advertise("image_rect", 1);
    
    double intrinsics_f[5] = {3.075376076482602, 779.3889270920181, 774.414730567985, 320.3469010920599, 319.52706568389664};
    double xi_f = intrinsics_f[0];
    cv::Mat K_f = (cv::Mat_<double>(3,3) << intrinsics_f[1], 0, intrinsics_f[3], 0, intrinsics_f[2], intrinsics_f[4], 0, 0, 1);
    cv::Mat D_f = (cv::Mat_<double>(4,1) << 0.3362739275084884, 8.363920195094627, -0.0016964769829006534, -0.0006806304247636069);
    
    double intrinsics_b[5] = {3.0131291282809958, 764.4581045655107, 759.0784553375285, 320.2977364951788, 320.57265795866255};
    double xi_b = intrinsics_b[0];
    cv::Mat K_b = (cv::Mat_<double>(3,3) << intrinsics_b[1], 0, intrinsics_b[3], 0, intrinsics_b[2], intrinsics_b[4], 0, 0, 1);
    cv::Mat D_b = (cv::Mat_<double>(4,1) << 0.25759808511957877, 7.956525781716198, -0.0023664921407204787, -0.008953352024511028);
    
    double rho_limit = 95.0 * CV_PI / 180.0;
    double baseline = 0.013;
    
    create_spherical_proj(K_f, xi_f, D_f, 0.0, 0.0, rho_limit, map1_f, map2_f, f_mask);
    create_spherical_proj(K_b, xi_b, D_b, CV_PI, -baseline, rho_limit, map1_b, map2_b, b_mask);
    
    intersect = f_mask * b_mask;
    not_intersect = 1 - intersect;
  }

  void create_spherical_proj(cv::Mat& K, double& xi, cv::Mat& D, double plus_theta, double zi, double rho_limit, cv::Mat& map1, cv::Mat& map2, cv::Mat& mask)
  {
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
		
		
		const cv::Mat img_rect = img_frnt;
		sensor_msgs::ImagePtr rect_msg = cv_bridge::CvImage(msg->header, msg->encoding, img_rect).toImageMsg();
		image_rect_.publish(rect_msg);
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
