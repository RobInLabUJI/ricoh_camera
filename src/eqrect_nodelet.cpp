#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ccalib/omnidir.hpp>

#include "opencv2/cudaarithm.hpp"
#include <opencv2/cudawarping.hpp>
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaimgproc.hpp"

#include <fstream>

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
  
  cv::cuda::GpuMat gpumap1_f;
  cv::cuda::GpuMat gpumap1_b;
  cv::cuda::GpuMat gpumap2_f;
  cv::cuda::GpuMat gpumap2_b;

  cv::Mat f_mask;
  cv::Mat b_mask;
  cv::Mat intersect;
  cv::Mat not_intersect; 

  cv::cuda::GpuMat gpuintersect;
  cv::cuda::GpuMat gpunot_intersect;

  bool invert;
  
public:

  void onInit()
  {
    ros::NodeHandle &nh         = getNodeHandle();
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    
    private_nh.getParam("invert", invert);
    
    float intrinsics_f[5] = {3.075376076482602, 779.3889270920181, 774.414730567985, 320.3469010920599, 319.52706568389664};
    float xi_f = intrinsics_f[0];
    cv::Mat K_f = (cv::Mat_<float>(3,3) << intrinsics_f[1], 0, intrinsics_f[3], 0, intrinsics_f[2], intrinsics_f[4], 0, 0, 1);
    cv::Mat D_f = (cv::Mat_<float>(4,1) << 0.3362739275084884, 8.363920195094627, -0.0016964769829006534, -0.0006806304247636069);
    
    float intrinsics_b[5] = {3.0131291282809958, 764.4581045655107, 759.0784553375285, 320.2977364951788, 320.57265795866255};
    float xi_b = intrinsics_b[0];
    cv::Mat K_b = (cv::Mat_<float>(3,3) << intrinsics_b[1], 0, intrinsics_b[3], 0, intrinsics_b[2], intrinsics_b[4], 0, 0, 1);
    cv::Mat D_b = (cv::Mat_<float>(4,1) << 0.25759808511957877, 7.956525781716198, -0.0023664921407204787, -0.008953352024511028);
    
    float rho_limit = 95.0 * CV_PI / 180.0;
    float baseline = 0.013;
    
    create_spherical_proj(K_f, xi_f, D_f, 0.0, 0.0, rho_limit, map1_f, map2_f, f_mask);
    create_spherical_proj(K_b, xi_b, D_b, CV_PI, -baseline, rho_limit, map1_b, map2_b, b_mask);
    
    //intersect = f_mask & b_mask;
    //not_intersect = ~intersect;   
    intersect = f_mask.mul(b_mask);
    not_intersect = 1.0 - intersect;

    gpuintersect.upload(intersect);
    gpunot_intersect.upload(not_intersect);
 
    gpumap1_f.upload(map1_f);
    gpumap2_f.upload(map2_f);
    gpumap1_b.upload(map1_b);
    gpumap2_b.upload(map2_b);

    it_.reset(new image_transport::ImageTransport(nh)); 

    image_sub_ = it_->subscribe("image_raw", 1, &EqRect::imageCb, this);
    image_rect_ = it_->advertise("image_rect", 1);
  }

  void create_spherical_proj(const cv::Mat& K, float xi, const cv::Mat& D, float plus_theta, float zi, 
							 float rho_limit, cv::Mat& map1, cv::Mat& map2, cv::Mat& mask)
  {
	  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
	  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);
  	  
	  int height = 640;
	  int width = 1280;
	  
  	  map1 = cv::Mat::zeros(height, width, CV_32F);
	  map2 = cv::Mat::zeros(height, width, CV_32F);
	  mask = cv::Mat::zeros(height, width, CV_32F);

	  float step_theta = 2 * CV_PI / width;
	  float step_phi = CV_PI / height;
	  
	  cv::Mat d = cv::Mat::zeros(1, 1, CV_64FC3);
	  cv::Mat m;
	  
	  float theta, phi, rho;
	  for (int i=0; i<height; i++) 
	    for (int j=0; j<width; j++)
		{
			theta = j * step_theta - CV_PI + plus_theta;
			phi = i * step_phi - CV_PI/2;
			d.at<cv::Vec3d>(0,0)[0] = sin(theta) * cos(phi);
			d.at<cv::Vec3d>(0,0)[1] = sin(phi);
			d.at<cv::Vec3d>(0,0)[2] = cos(theta) * cos(phi); 
			rho = acos(d.at<cv::Vec3d>(0,0)[2]);
			d.at<cv::Vec3d>(0,0)[2] += zi;
			if (rho < rho_limit) {
				cv::omnidir::projectPoints(d, m, rvec, tvec, K, xi, D);
				map1.at<float>(i,j) = (float) m.at<cv::Vec3d>(0,0)[0];
				map2.at<float>(i,j) = (float) m.at<cv::Vec3d>(0,0)[1];
                mask.at<float>(i,j) = 1.0;
			} else {
				map1.at<float>(i,j) = (float) -1;
				map2.at<float>(i,j) = (float) -1;
			}
		}
	  //mask = (map1 != (float) -1);
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
		cv::Mat image_host = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;   


        cv::cuda::GpuMat image(image_host);

		cv::cuda::GpuMat img_frnt = image( cv::Rect(0, 0, 640, 640) );
		cv::cuda::GpuMat img_back = image( cv::Rect(640, 0, 640, 640) );
		cv::cuda::GpuMat img_frnt_r, img_back_r, img_rect;
        cv::cuda::GpuMat img_rect_frnt, img_rect_back, s, hs, r1, r2;
        cv::Mat img_rect_host;

		if (invert) {
		  cv::cuda::rotate(img_frnt, img_frnt_r, img_frnt.size(), 90.0, 0, 640);
		  cv::cuda::rotate(img_back, img_back_r, img_back.size(), -90.0, 640, 0);
		} else {
		  cv::cuda::rotate(img_frnt, img_frnt_r, img_frnt.size(), -90.0, 640, 0);
		  cv::cuda::rotate(img_back, img_back_r, img_back.size(), 90.0, 0, 640);
		}

		cv::cuda::remap(img_frnt_r, img_rect_frnt, gpumap1_f, gpumap2_f, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
		cv::cuda::remap(img_back_r, img_rect_back, gpumap1_b, gpumap2_b, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
	
		//s = img_rect_frnt + img_rect_back;
        cv::cuda::add(img_rect_frnt, img_rect_back, s);
		//hs = img_rect_frnt/2 + img_rect_back/2;
		cv::cuda::addWeighted(img_rect_frnt, 0.5, img_rect_back, 0.5, 0.0, hs);
/*		
		cv::cuda::bitwise_and(hs, hs, r2, gpuintersect);
		cv::cuda::bitwise_and(s, s, r1, gpunot_intersect);
		//img_rect = r1 + r2;
		cv::cuda::add(r1, r2, img_rect);
*/
/*
        cv::cuda::multiply(hs, gpuintersect, r2);
        cv::cuda::multiply(s, gpunot_intersect, r1);
        cv::cuda::add(r1, r2, img_rect);
*/
        s.download(img_rect_host);

		sensor_msgs::ImagePtr rect_msg = cv_bridge::CvImage(msg->header, msg->encoding, img_rect_host).toImageMsg();
		image_rect_.publish(rect_msg);
/*
*/
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

