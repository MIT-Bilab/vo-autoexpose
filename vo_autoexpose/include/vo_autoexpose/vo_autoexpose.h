/*--------------------------------------------------------------------
 * Generic header file for active gain/exposure controller
 * (This is not a ROS node).
 *------------------------------------------------------------------*/
#ifndef MBA_AEG_H // header guard
#define MBA_AEG_H

#include <iostream>
#include <eigen3/Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/video/tracking.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

const int _n_corrections = 7;
const int _n_order_polyfit = 6;
const int _n_coeffs_polyfit = _n_order_polyfit+1;

const int GRADINFO_SCORE = 0;
const int STEREOMATCHES_SCORE = 1;

const int GAMMA_CORRECTIONS = 0;
const int EV_CORRECTIONS = 1;

const int IMU_ESTIM = 0;
const int OPTIFLOW_ESTIM = 1;

// Defining custom Eigen variable type
typedef Eigen::Matrix<float, 5, 1> Vector5f;
typedef Eigen::Matrix<float, _n_corrections, 1> VectorNcorrect;
typedef Eigen::Matrix<float, _n_order_polyfit+1, 1> VectorNorderFit;

using namespace std;

class StereoMatcher{
  public:
    int n_matches; // Latest number of matches
    void match(cv::Mat img1, cv::Mat img2); // Compute matches

  private:
    const cv::Ptr<cv::ORB> orb_detector=cv::ORB::create(1200, 1.2f, 8, 31, 0, 2, cv::ORB::FAST_SCORE, 31, 20); // orb detector
    const cv::Ptr<cv::BFMatcher> bf_matcher=cv::BFMatcher::create(cv::BFMatcher::BRUTEFORCE_HAMMING,false); // brute-force matcher
    float Th = 1.0; // Max. threshold on the stereo equipolar error
    const float ratio_thresh = 0.9f; // Lowe's ratio test threshold    
};

struct ExposureParameters{
  public:
    float  exposure;      // Exposure time in seconds
    float  gain;          // Camera gain in dB
};

struct ControlVariables{
  // Metrics and other useful variables used/produced by the controller
  public:
    VectorNcorrect scores; // score/metric associated to each gamma-corrected image (e.g. avg. gradient info or number of stereo matches)
    VectorNorderFit polyfit_coeffs;   // coefficients for the polynomial fitted to the gamma-corrected image metrics
    float     gamma_optim;   // optimal gamma determined based on simulated gamma corrections
    float     deltaEV_optim; // optimal change in exposure value based on simulated changes in exposure values 
    float     next_frame_optim_score; // Hand-tuned score balancing noise and motion blur for exposure-gain balance
    float     exposure_value_next; // predicted exposure value for the next frame
    float     exposure_next; // next exposure time determined
    float     gain_next; // next gain determined
}; // end of ControlVariables structure

struct ControlParameters{
  public:
    int n_corrections = _n_corrections;
    int n_order_polyfit = _n_order_polyfit;
    int n_coeffs_polyfit = _n_coeffs_polyfit;

    VectorNcorrect gamma_values; // gamma values used for prediction
    Eigen::Matrix<float, _n_corrections, _n_order_polyfit+1> gamma_powers_matrix; // pre-computed matrix of powers of gamma for curve fitting
    Eigen::ColPivHouseholderQR<Eigen::Matrix<float, _n_corrections, _n_order_polyfit+1>> gamma_powers_matrix_decomposed; // QR decomposition of the above
    vector<cv::Mat> gamma_tables; // Lookup tables for gamma
    
    VectorNcorrect delta_EVs; // change in exposure value used for prediction (alternative to gamma_values which relies on photometric response)
    Eigen::Matrix<float, _n_corrections, _n_order_polyfit+1> delta_EVs_powers_matrix; // pre-computed matrix of powers of delta_EVs for curve fitting
    Eigen::ColPivHouseholderQR<Eigen::Matrix<float, _n_corrections, _n_order_polyfit+1>> delta_EVs_powers_matrix_decomposed; // QR decomposition of the above
    vector<cv::Mat> delta_EV_tables; // Lookup tables for imposed change in exposure value (relies on photometric response)

    // Min/Max illumination (as mean pixel value)
    double Imin = 10;
    double Imax = 245;

    // Downsampling size
    int down_size_h=270;
    int down_size_w=360;

    // Control parameters based on Shim 2018 and Mehta 2020.
    double kp=1.6; //0.8
    double d=0.1;
    double R;

    // Parameters for computing Shim gradient metric
    double met_act_thresh = 0.3;
    cv::Mat metric_table; // Lookup tables for gradient info metric (Shim 2014)
    double sigma = 255.0 * met_act_thresh;
    double lambda = 1000.0;

    // Parameters for balancing gain and exposure
    double w = 0.02;//0.1; //0.1; // weight on cost associated to gain (wrt to average blur length in pixels) -> higher value = use lower gain

}; // end of Control Parameters structure

struct Frame{
  
  public:
    // Basic input variables
    int    seq = 0;       // Image number
    float  timestamp;     // Camera timestamp
    cv::Mat img;          // actual image
    float  exposure;      // Exposure time in seconds
    float  gain;          // Camera gain in dB

    // (Optional)
    Eigen::Matrix<float, 6, 1> twist; // instantaneous twist estimate in camera optical ref. frame (v_x,v_y,v_z,w_x,w_y,w_z)

    // Derived variables
    float  exposure_value; // Exposure value computed according to Mehta 2020.
    float avg_pixel_speed = 0.0; // estimated average pixel speed 
    struct ControlVariables  control_vars; // Keep track of quality metrics associated to each frame

}; // end of Frame structure

struct Camera{
  public:
    string camid;
    Vector5f photocalib; // Photometric calibration (necessary for some methods)
    // See Bergmann 2018 "Online Photometric Calibration of Auto Exposure Video for Realtime Visual Odometry and SLAM"
    // Info: https://vision.in.tum.de/research/vslam/photometric-calibration
    // Code: https://github.com/tum-vision/online_photometric_calibration                        
    // Coefficients of a fifth-order polynomial fit.
    // X: Exposure, I: Intensity
    // ln(X) = p[0] + p[1]*I + p[2]*I^2 + p[3]*I^3 + p[4]*I^4 + p[5]*I^5

    float fx; // # of horizontal pixels/m * focal length
    float fy; // # of vectical pixels/m * focal length
    float fov_x; // horizontal field of view (rads)
    float fov_y; // vectical field of view (rads)
    float px_per_rad_x; // number of pixels per rad, horizontal
    float px_per_rad_y; // number of pixels per rad, vectical

    float gain_min = 0.1; // minimum gain factor in dB (note: do not set to >=0, otherwise FLIR camera switched to auto-gain)
    float gain_max = 40.0; // maximum gain factor in dB
    float exposure_min = 0.000105; // minimum exposure time in seconds (note: do not set to >=0, otherwise FLIR camera switched to auto-expose)
    float exposure_max = 0.015000; // maximum exposure time in seconds

    float F = 1.2; // camera F-number

    float exposure_value_min = 2*log2(F)-log2(exposure_max)-log2(10)/20*gain_max; // minimum exposure value
    float exposure_value_max = 2*log2(F)-log2(exposure_min)-log2(10)/20*gain_min; // maximum possible exposure value

    vector<Frame>   frames;           // Buffered camera frames
    int        height;     // Images height in px
    int        width;      // Images width in px

}; // end of Camera structure

struct ImuMeasurement{
  public:
    Eigen::Matrix<float,3,1> lin_accel; // linear acceleration (m/s^2), [a_x, a_y, a_z]
    Eigen::Matrix<float,3,1> ang_vel;   // rotational velocity (rad/s), [w_x, w_y, w_z]
    float timestamp; // timestamp in seconds
};

struct Imu{
  public:
    string imuid;
    vector<ImuMeasurement> measures;
    Eigen::Matrix<float,4,4> T_imu_2_cam; // Transform from IMU frame to camera frame 
};

class  ExpGainController{
  public:            
    ControlParameters control_params; // Control parameters
    int             n_cameras;        // Number of cameras considered by the controller 
    vector<Camera>  cameras;          // Cameras considered by the controller
    vector<Imu>     imus;             // Imus considered by the controller
    StereoMatcher stereo_matcher;     // Stereo matcher

  // ExpGainController(string _controller_name): controller_name{_controller_name}{};

  // Initialization functions
  void add_camera(string id,int h,int w,float F, float fx, float fy, Vector5f calib);
  void add_imu(string id, Eigen::Matrix<float,4,4> T_imu_2_cam);
  void set_gamma_values(VectorNcorrect values);

  void init();
  void compute_gamma_LUTs();
  void compute_metric_LUT();
  void compute_delta_EV_LUTs();

  // Data acquisition
  void add_frame(int camid,cv::Mat img, float exposure_time, float gain, int seq, float timestamp);
  void add_imu_meas(int imuid, Eigen::Matrix<float,3,1> lin_accel, Eigen::Matrix<float,3,1> ang_vel, float timestamp);

  // Optional (depends on implementation)
  // void grab_imu();
  // void get_optical_flow();
  // void get_local_features();
  // void get_twist();
  ExposureParameters update_params(int score=GRADINFO_SCORE,int correction=GAMMA_CORRECTIONS,int blur_estim=IMU_ESTIM);

  private:

    float gain_dB_scale = log2(10.0)/20.0 ; // Precompute scale applied to gain (in dB) to compute exposure value. It is very close to 1/6
    
    // Internal utility functions
    float find_exposure_value(float _F,float _exposure_time, float _gain){
      // Compute exposure value based on exposure time and gain 
      // F: camera F number
      // exposure_time: exposure time in seconds
      // gain: image gain in dB
      return 2.0*log2(_F) - log2(_exposure_time) - gain_dB_scale*_gain;
    };

    float factor_2_dB(float x_not_dB){
      return 20*log10(x_not_dB);
    }

    float dB_2_factor(float x_dB){
      return pow(10,x_dB/20);
    }

}; // end of ExpGainController class

#endif // end of header guard