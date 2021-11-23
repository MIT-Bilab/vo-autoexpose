/*--------------------------------------------------------------------
 * ROS wrapper for motion-blur aware automatic exposure and gain
 *------------------------------------------------------------------*/
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/serialization.h"
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/GroupState.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <spinnaker_sdk_camera_driver/SpinnakerImageParams.h>

#include <vo_autoexpose/vo_autoexpose.h>
#include <chrono>  

ExpGainController controller; // Instance of exposure/gain automatic controller
string service_call;

void send_params(float exposure_time, float gain){
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter gain_double;
  dynamic_reconfigure::IntParameter exp_time_int;
  dynamic_reconfigure::Config conf;

  // Update exposure time
  exp_time_int.name = "exposure_time"; 
  exp_time_int.value = int(1000000*exposure_time); // exposure time is integer in microseconds. 
  conf.ints.push_back(exp_time_int); 

  // Update gain
  gain_double.name = "gain"; 
  gain_double.value = double(gain); // gain is double in dB.
  conf.doubles.push_back(gain_double);


  srv_req.config = conf;
  ros::service::call(service_call,srv_req, srv_resp);

  // Debug
  // std::cout << "Service call: " << service_call << std::endl;
  // std::cout << "Service request: " << srv_req << std::endl;
  // std::cout << "Service resp: " << srv_resp << std::endl;
  
}

void imu_cb(const sensor_msgs::Imu& msg)
{
  Eigen::Matrix<float,3,1> lin_accel; 
  lin_accel << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
  Eigen::Matrix<float,3,1> rot_vel; 
  rot_vel << msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z;
  controller.add_imu_meas(0,lin_accel,rot_vel,msg.header.stamp.toSec());
};

void image_cb(const sensor_msgs::Image::ConstPtr& left_image, const sensor_msgs::Image::ConstPtr& right_image, const spinnaker_sdk_camera_driver::SpinnakerImageParams::ConstPtr&  image_params)
{  
  cv_bridge::CvImagePtr cv_left_ptr;
  cv_bridge::CvImagePtr cv_right_ptr;
  cv_left_ptr = cv_bridge::toCvCopy(left_image,"mono8");
  cv_right_ptr = cv_bridge::toCvCopy(right_image,"mono8");

  // Add new frames
  controller.add_frame(0,cv_left_ptr->image,float(image_params->exposure_time_us)/1000000,image_params->gain_dB,cv_left_ptr->header.seq,cv_left_ptr->header.stamp.toSec());
  controller.add_frame(1,cv_right_ptr->image,float(image_params->exposure_time_us)/1000000,image_params->gain_dB,cv_right_ptr->header.seq,cv_right_ptr->header.stamp.toSec());

  // Update the parameters
  ExposureParameters new_exposure_params = controller.update_params(STEREOMATCHES_SCORE);

  // Send parameters
  send_params(new_exposure_params.exposure,new_exposure_params.gain);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mba_aeg_node");
  ros::NodeHandle* nh;
  nh = new ros::NodeHandle("~");

  // Initialize node parameters 
  nh->getParam("service_call", service_call);

  // Create subscribers and publishers.
  // TODO: Create a publisher for the controller stats ? 

  // Create subscribers (synchronized subscribers for image and image info)
  ros::Subscriber imu_sub = nh->subscribe("/imu",1,&imu_cb);
  message_filters::Subscriber<sensor_msgs::Image> cam_left_sub(*nh, "/left/image_raw", 0);
  message_filters::Subscriber<sensor_msgs::Image> cam_right_sub(*nh, "/right/image_raw", 0);
  message_filters::Subscriber<spinnaker_sdk_camera_driver::SpinnakerImageParams> cam_left_params_sub(*nh, "/left/image_params", 0);
  using sync_pol = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, spinnaker_sdk_camera_driver::SpinnakerImageParams>;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(1), cam_left_sub,cam_right_sub, cam_left_params_sub);
  sync.registerCallback(boost::bind(&image_cb, _1, _2, _3));


  /*---------------------------------------------
  * INITIALIZATION OF EXPOSURE/GAIN CONTROLLER
  -----------------------------------------------*/

  // TODO: Replace with a single init function that loads from YAML
  Vector5f calib; calib << 0.0, 0.0, 0.0, 0.0, 0.0 ;

  Eigen::Matrix<float,4,4> T_imu_2_cam;
  T_imu_2_cam << 0.0,  0.0,  -1.0,  -0.062,
                -1.0,  0.0,   0.0,   0.04,
                 0.0,  1.0,   0.0,  -0.008,
                 0.,   0.,    0.,    1.       ;
  VectorNcorrect gamma_values;
  gamma_values << 1.0/1.9, 1.0/1.5, 1.0/1.2, 1.0, 1.2, 1.5, 1.9;

  controller.add_camera("left",740,520,1.2,865.909537,865.909537,calib);
  controller.add_camera("right",740,520,1.2,865.909537,865.909537,calib);
  controller.add_imu("imu",T_imu_2_cam);

  controller.set_gamma_values(gamma_values);
  controller.init();

  ros::spin();

  return 0;

} // end main()

