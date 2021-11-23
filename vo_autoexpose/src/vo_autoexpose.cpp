/*--------------------------------------------------------------------
 * vo-autoexpose main library
 * (This is not a ROS node).
 *------------------------------------------------------------------*/

#include <vo_autoexpose/vo_autoexpose.h>
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;



//-----------------------------------------------------------------------------
// Format current time (calculated as an offset in current day) in this form:
//
//     "hh:mm:ss.SSS" (where "SSS" are milliseconds)
//-----------------------------------------------------------------------------
std::string now_str()
{
    // Get current time from the clock, using microseconds resolution
    const boost::posix_time::ptime now = 
        boost::posix_time::microsec_clock::local_time();

    // Get the time offset in current day
    const boost::posix_time::time_duration td = now.time_of_day();

    //
    // Extract hours, minutes, seconds and milliseconds.
    //
    // Since there is no direct accessor ".milliseconds()",
    // milliseconds are computed _by difference_ between total milliseconds
    // (for which there is an accessor), and the hours/minutes/seconds
    // values previously fetched.
    //
    const long hours        = td.hours();
    const long minutes      = td.minutes();
    const long seconds      = td.seconds();
    const long milliseconds = td.total_milliseconds() -
                              ((hours * 3600 + minutes * 60 + seconds) * 1000);

    //
    // Format like this:
    //
    //      hh:mm:ss.SSS
    //
    // e.g. 02:15:40:321
    //
    //      ^          ^
    //      |          |
    //      123456789*12
    //      ---------10-     --> 12 chars + \0 --> 13 chars should suffice
    //  
    // 
    char buf[40];
    sprintf(buf, "%02ld:%02ld:%02ld.%03ld", 
        hours, minutes, seconds, milliseconds);

    return buf;
}


// Stereo matcher
void StereoMatcher::match(cv::Mat img1, cv::Mat img2){
    std::vector<std::vector<cv::DMatch>> knn_matches; // note the vector of vectors (that's because I return first and second best matches for each kp)
    std::vector<cv::KeyPoint> kp1; // keypoints of image 1
    std::vector<cv::KeyPoint> kp2; // keypoints of image 2
    cv::Mat des1; // descriptors for image 1
    cv::Mat des2; // descriptors of image 2

    // Detect with FAST, describe with ORB
    orb_detector->detectAndCompute(img1,cv::noArray(),kp1,des1,false);
    orb_detector->detectAndCompute(img2,cv::noArray(),kp2,des2,false);

    // Match using brute force (return 2 best matches for Lowe's ratio test)
    bf_matcher->knnMatch(des1,des2,knn_matches,2,cv::noArray(),false);

    // Compute good matches
    std::vector<cv::DMatch> good_matches;
    for (int i=0; i<knn_matches.size();i++){
        // Remove matches with too large equipolar error
        if ( abs(kp1[knn_matches[i][0].queryIdx].pt.y - kp2[knn_matches[i][0].trainIdx].pt.y) < Th ){
            // Lowe's ratio test
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance){
                good_matches.push_back(knn_matches[i][0]);
            }
        }
    }
    n_matches = good_matches.size();

    // Debug: show images with matches
    // cv::Mat img_with_matches;
    // cv::drawMatches(img1,kp1,img2,kp2,good_matches,img_with_matches);
    // cv::imshow("Matches",img_with_matches);
    // cv::waitKey(0);
};

// Utility functions
float sign(float x){
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

// Controller

void ExpGainController::init(){
    // Precomputes usefull parameters for the controller.
    compute_gamma_LUTs();
    compute_delta_EV_LUTs();
    compute_metric_LUT();
    compute_delta_EV_LUTs();
};

void ExpGainController::set_gamma_values(VectorNcorrect values){
    control_params.gamma_values = values;
};

void ExpGainController::compute_gamma_LUTs(){
    for(int i_correct=0; i_correct<control_params.n_corrections; i_correct++)
    {
        float gamma = control_params.gamma_values[i_correct];
        float gamma_inv = 1.0/gamma;

        cv::Mat table(1, 256, CV_8U);
        uchar *p = table.ptr();

        for (int i_int = 0; i_int < 256; i_int++)
        {
            p[i_int] = (uchar) (pow(i_int / 255.0, gamma_inv) * 255.0);
        }
        ExpGainController::control_params.gamma_tables.push_back(table);

        // Also compute powers of gamma for curve fitting
        for (int i_order = 0; i_order < control_params.n_coeffs_polyfit; i_order++){
            control_params.gamma_powers_matrix(i_correct,i_order) = pow(gamma,i_order);
        }
        control_params.gamma_powers_matrix_decomposed = control_params.gamma_powers_matrix.colPivHouseholderQr();
    }
};

void ExpGainController::compute_delta_EV_LUTs(){
    // Note that the tables are not really computed here.
    // Precomputed tables based on delta_EVs = -2.0,-1.0,-0.5,0.0,0.5,1.0,2.0
    control_params.delta_EVs << -2.0,-1.0,-0.5,0.0,0.5,1.0,2.0; 

    std::vector<Eigen::Matrix<uint8_t,1,256>> tables_eigen;
    Eigen::Matrix<uint8_t,1,256> table_eigen;
    table_eigen <<  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,
                    57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,
                    57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,  57,
                    57,  58,  58,  60,  61,  62,  63,  64,  65,  67,  69,  71,  73,
                    75,  76,  78,  79,  81,  83,  84,  86,  87,  89,  91,  93,  94,
                    97,  99, 101, 102, 103, 105, 106, 107, 109, 110, 112, 113, 115,
                    116, 118, 120, 121, 123, 125, 126, 128, 130, 132, 133, 135, 137,
                    139, 140, 142, 143, 144, 146, 148, 149, 151, 152, 154, 155, 157,
                    158, 160, 161, 163, 164, 166, 167, 169, 171, 172, 173, 175, 176,
                    177, 179, 180, 182, 183, 184, 186, 187, 189, 190, 192, 193, 194,
                    196, 197, 199, 201, 202, 204, 205, 207, 208, 210, 211, 213, 214,
                    216, 217, 219, 221, 222, 224, 226, 227, 229, 230, 232, 234, 235,
                    236, 237, 238, 240, 241, 242, 243, 245, 246, 247, 248, 250, 251,
                    252, 253, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                    255, 255, 255, 255, 255, 255, 255, 255, 255;
    tables_eigen.push_back(table_eigen);
    table_eigen <<  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,
                    48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,
                    48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,
                    48,  48,  49,  50,  51,  52,  53,  54,  55,  57,  58,  59,  60,
                    62,  63,  63,  65,  66,  67,  68,  70,  71,  73,  75,  76,  78,
                    79,  81,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,
                    94,  96,  97,  98,  99, 101, 102, 103, 105, 106, 107, 109, 110,
                    112, 113, 114, 115, 117, 118, 119, 121, 122, 123, 125, 126, 127,
                    128, 130, 131, 132, 133, 135, 136, 137, 139, 140, 141, 142, 143,
                    144, 145, 147, 148, 149, 150, 151, 152, 154, 155, 156, 157, 158,
                    160, 161, 162, 163, 164, 166, 167, 168, 169, 171, 172, 173, 174,
                    175, 176, 178, 179, 180, 181, 183, 184, 185, 186, 187, 189, 190,
                    191, 192, 193, 195, 196, 197, 198, 200, 201, 202, 203, 205, 206,
                    207, 208, 210, 211, 212, 214, 215, 216, 218, 219, 220, 222, 223,
                    224, 226, 227, 228, 230, 231, 232, 234, 235, 236, 237, 238, 238,
                    240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 250, 251, 252,
                    253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                    255, 255, 255, 255, 255, 255, 255, 255, 255;
    tables_eigen.push_back(table_eigen);
    table_eigen <<  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,
                    44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,
                    44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,  44,
                    44,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,
                    57,  58,  58,  60,  61,  62,  63,  63,  65,  66,  67,  68,  70,
                    71,  73,  75,  76,  77,  78,  79,  80,  81,  82,  83,  84,  85,
                    86,  87,  88,  89,  90,  91,  92,  93,  94,  96,  97,  98,  99,
                    101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 112, 113, 114,
                    115, 116, 117, 119, 120, 121, 122, 123, 125, 126, 127, 128, 129,
                    130, 131, 132, 133, 134, 135, 136, 137, 139, 140, 141, 142, 143,
                    144, 145, 146, 147, 148, 149, 150, 151, 152, 154, 155, 156, 157,
                    158, 159, 160, 161, 163, 164, 165, 166, 167, 168, 169, 171, 172,
                    173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185,
                    186, 187, 189, 190, 191, 192, 193, 194, 196, 197, 198, 199, 200,
                    201, 203, 204, 205, 206, 207, 208, 210, 211, 212, 213, 214, 215,
                    216, 218, 219, 220, 221, 222, 223, 224, 226, 227, 228, 229, 230,
                    231, 232, 234, 234, 235, 236, 237, 238, 239, 240, 241, 241, 242,
                    243, 244, 245, 246, 247, 248, 248, 249, 250, 251, 252, 253, 254,
                    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
                    255, 255, 255, 255, 255, 255, 255, 255, 255;
    tables_eigen.push_back(table_eigen);
    table_eigen <<  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  40,  40,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,
                    52,  53,  53,  55,  56,  57,  58,  58,  60,  61,  62,  63,  63,
                    65,  66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76,  77,
                    78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,
                    91,  92,  93,  94,  94,  96,  97,  98,  99,  99, 101, 101, 103,
                    104, 105, 105, 107, 108, 109, 109, 111, 112, 113, 114, 114, 115,
                    116, 117, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129,
                    130, 130, 132, 133, 134, 135, 136, 137, 137, 139, 140, 141, 142,
                    143, 143, 144, 145, 147, 148, 149, 150, 151, 152, 153, 154, 155,
                    156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168,
                    169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181,
                    182, 183, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 193,
                    195, 196, 197, 198, 199, 200, 201, 202, 203, 203, 205, 206, 207,
                    208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
                    221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233,
                    234, 235, 236, 237, 238, 238, 240, 241, 242, 243, 244, 245, 246,
                    247, 248, 248, 250, 251, 252, 253, 254, 255;
    tables_eigen.push_back(table_eigen);
    table_eigen <<  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  40,  41,  42,  44,  45,  46,  47,
                    48,  48,  49,  50,  51,  52,  53,  53,  55,  56,  57,  58,  58,
                    60,  61,  62,  62,  63,  63,  64,  65,  65,  66,  67,  68,  69,
                    70,  71,  72,  73,  74,  75,  76,  77,  78,  79,  80,  81,  82,
                    83,  83,  84,  85,  86,  87,  87,  88,  89,  90,  91,  91,  92,
                    93,  94,  95,  96,  97,  98,  99, 100, 101, 101, 102, 103, 104,
                    105, 106, 106, 107, 108, 109, 110, 111, 112, 112, 113, 114, 115,
                    116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 125, 126, 127,
                    128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 139,
                    140, 141, 142, 143, 144, 145, 146, 146, 147, 148, 149, 150, 151,
                    152, 153, 154, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163,
                    164, 165, 166, 167, 168, 169, 170, 171, 171, 172, 173, 174, 175,
                    176, 177, 177, 178, 179, 180, 181, 182, 183, 183, 184, 185, 186,
                    187, 188, 189, 189, 190, 191, 192, 193, 194, 195, 196, 196, 197,
                    198, 199, 200, 201, 202, 203, 203, 204, 205, 206, 207, 208, 209,
                    210, 211, 212, 213, 214, 215, 216, 218, 219, 220, 221, 222, 223,
                    224, 226, 227, 228, 229, 230, 231, 232, 244; // last value: somewhere between 234 and 255
    tables_eigen.push_back(table_eigen);
    table_eigen <<  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  40,  41,  42,
                    44,  44,  45,  46,  47,  48,  48,  49,  50,  51,  52,  53,  53,
                    55,  56,  57,  57,  58,  58,  59,  60,  60,  61,  62,  62,  63,
                    63,  64,  65,  65,  66,  67,  68,  69,  70,  71,  72,  73,  74,
                    75,  75,  76,  77,  78,  79,  79,  80,  81,  82,  83,  83,  84,
                    85,  85,  86,  87,  88,  88,  89,  90,  91,  91,  92,  93,  94,
                    94,  95,  96,  97,  97,  98,  99, 100, 101, 101, 102, 103, 104,
                    104, 105, 106, 107, 108, 108, 109, 110, 111, 112, 112, 113, 114,
                    115, 116, 117, 118, 118, 119, 120, 121, 122, 123, 124, 125, 125,
                    126, 127, 128, 129, 129, 130, 131, 132, 133, 134, 134, 135, 136,
                    137, 138, 139, 139, 140, 141, 142, 143, 144, 144, 145, 146, 147,
                    148, 148, 149, 150, 151, 152, 153, 154, 154, 155, 156, 157, 158,
                    158, 159, 160, 161, 162, 162, 163, 164, 165, 166, 166, 167, 168,
                    169, 170, 171, 171, 172, 173, 174, 174, 175, 176, 177, 177, 178,
                    179, 180, 180, 181, 182, 183, 183, 184, 185, 186, 186, 187, 188,
                    189, 190, 191, 192, 193, 193, 195, 196, 197, 198, 199, 200, 201,
                    202, 203, 203, 205, 206, 207, 208, 209, 220; // last value: somewhere between 210 and 255
    tables_eigen.push_back(table_eigen);
    table_eigen <<  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,  39,
                    39,  39,  39,  39,  39,  39,  40,  40,  42,  43,  44,  44,  45,
                    46,  47,  48,  48,  49,  49,  50,  50,  51,  51,  52,  52,  53,
                    53,  54,  55,  55,  56,  57,  57,  58,  58,  59,  60,  60,  61,
                    62,  62,  63,  63,  63,  64,  65,  65,  66,  66,  67,  67,  68,
                    69,  69,  70,  71,  72,  72,  73,  74,  75,  75,  76,  76,  77,
                    78,  78,  79,  79,  80,  81,  81,  82,  83,  83,  84,  84,  85,
                    85,  86,  87,  87,  88,  88,  89,  89,  90,  91,  91,  92,  93,
                    93,  94,  94,  95,  96,  97,  97,  98,  99,  99, 100, 101, 101,
                    102, 102, 103, 104, 104, 105, 106, 106, 107, 108, 108, 109, 110,
                    110, 111, 112, 112, 113, 114, 114, 115, 116, 117, 117, 118, 119,
                    119, 120, 121, 122, 122, 123, 124, 125, 125, 126, 127, 127, 128,
                    129, 129, 130, 131, 131, 132, 132, 133, 134, 135, 135, 136, 137,
                    137, 138, 139, 139, 140, 140, 141, 142, 142, 143, 144, 144, 145,
                    145, 146, 147, 147, 148, 148, 149, 150, 150, 151, 152, 152, 153,
                    154, 154, 155, 156, 157, 158, 158, 159, 160, 161, 162, 162, 163,
                    164, 165, 166, 166, 167, 168, 169, 170, 181; // last value: somewhere between 171 and 255
    tables_eigen.push_back(table_eigen);

    for(int i_correct=0; i_correct<control_params.n_corrections; i_correct++)
    {
        float delta_EV = control_params.delta_EVs[i_correct];

        cv::Mat table(1, 256, CV_8U);
        cv::eigen2cv(tables_eigen[i_correct],table);
        ExpGainController::control_params.delta_EV_tables.push_back(table);
        // std::cout << table << std::endl;

        // Also compute powers of delta_EV for curve fitting
        for (int i_order = 0; i_order < control_params.n_coeffs_polyfit; i_order++){
            control_params.delta_EVs_powers_matrix(i_correct,i_order) = pow(delta_EV,i_order);
        }
        control_params.delta_EVs_powers_matrix_decomposed = control_params.delta_EVs_powers_matrix.colPivHouseholderQr();
    }
}

void ExpGainController::compute_metric_LUT(){
    cv::Mat table(1, 256, CV_8U);
    uchar* q = table.ptr();
    for (int i = 0; i < 256; i++)
        {
            if (i >= control_params.sigma){
                    q[i] = 255 * ( ( log10( control_params.lambda * ((i-control_params.sigma)/255.0) + 1) ) / ( log10( control_params.lambda * ((255.0-control_params.sigma)/255.0) + 1) ) );
                    }
                else{
                    q[i] = 0;
                    }
        }
    ExpGainController::control_params.metric_table = table;

};

void ExpGainController::add_imu(string id, Eigen::Matrix<float,4,4> T_imu_2_cam){
    Imu imu_temp;
    imu_temp.imuid = id;
    imu_temp.T_imu_2_cam = T_imu_2_cam;
    imus.push_back(imu_temp);
}

void ExpGainController::add_camera(string id,int h,int w,float F, float fx, float fy,Vector5f calib){
    Camera cam_temp;
    cam_temp.camid = id;
    cam_temp.height = h;
    cam_temp.width = w;
    cam_temp.photocalib = calib;
    cam_temp.F = F;
    cam_temp.fx = fx;
    cam_temp.fy = fy;

    cam_temp.fov_x = 2 * atan2( w, (2*fx) ) ; 
    cam_temp.fov_y = 2 * atan2( h, (2*fy) ) ;

    cam_temp.px_per_rad_x = w/cam_temp.fov_x;
    cam_temp.px_per_rad_y = h/cam_temp.fov_y;

    ExpGainController::cameras.push_back(cam_temp);
};

void ExpGainController::add_frame(int camid,cv::Mat img, float exposure_time, float gain, int seq, float timestamp){
    // camid: Camera identifier
    // img: image data
    // exposure_time: exposure time in seconds
    // gain: gain in dB
    // seq: sequence number of the frame
    // timestamp: image timestamp in seconds
    
    Frame new_frame;
    new_frame.exposure = exposure_time;
    new_frame.gain = gain;
    new_frame.seq = seq;
    new_frame.timestamp = timestamp;

    // Apply median blur filter
    // cv::medianBlur(img,new_frame.img,7);

    // Apply CLAHE
    // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    // clahe->apply(img,new_frame.img);

    // Downsize the image
    cv::Size size(control_params.down_size_w,control_params.down_size_h);
    cv::resize(img , img , size); 

    // Apply median blur filter
    cv::medianBlur(img,new_frame.img,3); // about equivalent to kernel size 7 at full size

    new_frame.img = img;
    new_frame.exposure_value = find_exposure_value(cameras[camid].F,exposure_time,gain);
    cameras[camid].frames.push_back(new_frame);
}

void ExpGainController::add_imu_meas(int imuid, Eigen::Matrix<float,3,1> lin_accel, Eigen::Matrix<float,3,1> ang_vel, float timestamp){
    ImuMeasurement imu_meas;
    imu_meas.lin_accel = lin_accel;
    imu_meas.ang_vel = ang_vel;
    imu_meas.timestamp = timestamp;

    // Make sure I only keep a certain number of measurements
    if(imus[imuid].measures.size()>20){
        imus[imuid].measures.erase(imus[imuid].measures.begin());
    }

    imus[imuid].measures.push_back(imu_meas);
}; 

ExposureParameters ExpGainController::update_params(int score,int corrections,int blur_estim){
    auto t1 = high_resolution_clock::now();
    // Debug
    // cv::imshow("",cameras[0].frames[last].img);
    // cv::waitKey(0);

    int last = cameras[0].frames.size()-1; // last frame id
    
    // Gamma correction and metric computation
    cv::Mat img_tmp_left; 
    cv::Mat img_tmp_right;

    for(int ii=0;ii<control_params.n_corrections;ii++)
    {
        // Debug
        // cv::Scalar img_mean_I;
        // img_mean_I = cv::mean(cameras[0].frames[last].img);

        if(score==GRADINFO_SCORE){
            // Predict future images at different camera settings based on gamma or EV corrections
            // Note: EV corrections require to identify the camera response function first
            if(corrections==GAMMA_CORRECTIONS){
                // Adjust gamma
                cv::LUT(cameras[0].frames[last].img,control_params.gamma_tables[ii],img_tmp_left);
            }
            else if(corrections==EV_CORRECTIONS){
                // Adjust EV
                cv::LUT(cameras[0].frames[last].img,control_params.delta_EV_tables[ii],img_tmp_left);
            }

            //  Debug
            // cv::Scalar img_tmp_mean_I;
            // img_tmp_mean_I = cv::mean(img_tmp_left);
            // std::cout << img_tmp_mean_I[0]-img_mean_I[0] << std::endl;
            
            // Debug: View gamma-corrected or EV-corrected frames
            // cv::imshow("Correction no. "+std::to_string(ii),img_tmp_left);
            // cv::waitKey(0);
            // cv::destroyAllWindows();

            // Compute gradient
            cv::Mat grad_x, grad_y, grad, grad_info;
            cv::Sobel(img_tmp_left, grad_x, CV_64F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
            cv::Sobel(img_tmp_left, grad_y, CV_64F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
            cv::sqrt(grad_x.mul(grad_x) + grad_y.mul(grad_y),grad);
            grad.convertTo(grad,CV_8UC1); // scale and convert

            // Debug
            // double maxVal;
            // double minVal;
            // cv::Point minLoc;
            // cv::Point maxLoc;
            // cv::minMaxLoc( img_tmp_left, &minVal, &maxVal, &minLoc, &maxLoc );
            // std::cout << "Max of image: " << maxVal << std::endl;

            // Compute metric
            cv::LUT(grad, control_params.metric_table, grad_info);

            // Debug
            // std::cout << "Gradient: " << cv::sum(grad_info) << std::endl;

            // Debug
            // Experimenting with alternative metric (see Shin 2019 with binned grad info metric)
            // cameras[0].frames[last].control_vars.scores[ii] = cv::sum(grad_info)[0]/100000;
            // cv::Mat grad_binned;
            // cv::resize(grad,grad_binned,cv::Size(10,10));
            // cv::Scalar grad_mean_cv;
            // cv::Scalar grad_std_cv;
            // cv::meanStdDev(grad,grad_mean_cv,grad_std_cv);
            // double grad_mean = grad_mean_cv[0];
            // double grad_std = grad_std_cv[0];
            // std::cout << "Std of binned img grad info: " << grad_std << std::endl;
            // cameras[0].frames[last].control_vars.scores[ii] = 100*grad_mean/grad_std;

            cameras[0].frames[last].control_vars.scores[ii] = cv::sum(grad_info)[0]/100000;

            // Debug: Visualization of gradient
            // cv::imshow("",grad_info);
            // cv::waitKey(10);
            // if(ii==3){
            //     std::cout << "Gradient metric: " << cameras[0].frames[last].control_vars.scores[ii] << std::endl;
            // }
        }
        else if(score==STEREOMATCHES_SCORE){
            if(corrections==GAMMA_CORRECTIONS){
                // Adjust gamma
                cv::LUT(cameras[0].frames[last].img,control_params.gamma_tables[ii],img_tmp_left);
                cv::LUT(cameras[1].frames[last].img,control_params.gamma_tables[ii],img_tmp_right);
            }
            else if(corrections==EV_CORRECTIONS){
                // Adjust EV
                cv::LUT(cameras[0].frames[last].img,control_params.delta_EV_tables[ii],img_tmp_left);
                cv::LUT(cameras[1].frames[last].img,control_params.delta_EV_tables[ii],img_tmp_right);
            }

            // Compute stereo matches
            stereo_matcher.match(img_tmp_left,img_tmp_right);
            // if(ii==3){
            //     std::cout << "Number of stereo matches: " << stereo_matcher.n_matches << std::endl;
            // }

            cameras[0].frames[last].control_vars.scores[ii] = float(stereo_matcher.n_matches); // Note we only associate score with one of 2 cameras
            // cameras[1].frames[last].control_vars.scores[ii] = float(stereo_matcher.n_matches);
        }
    } // end of for loop through gamma-corrections

    // cv::imshow("",img_tmp_left);
    // cv::waitKey(500);
    
    // Perform polynomial fit of metric vs. gamma OR vs. exposure value (through least-square)
    if (corrections==GAMMA_CORRECTIONS){
        cameras[0].frames[last].control_vars.polyfit_coeffs = control_params.gamma_powers_matrix_decomposed.solve(cameras[0].frames[last].control_vars.scores);
    }
    else if (corrections==EV_CORRECTIONS){
        cameras[0].frames[last].control_vars.polyfit_coeffs = control_params.delta_EVs_powers_matrix_decomposed.solve(cameras[0].frames[last].control_vars.scores);
    }

    // Debug
    // std::cout << "Grad. info. metrics: "<< std::endl << cameras[0].frames[last].control_vars.scores << std::endl;
    // std::cout<< "Predicted metrics (gamma corrections): "<< std::endl  << control_params.gamma_powers_matrix*cameras[0].frames[last].control_vars.polyfit_coeffs << std::endl;
    // std::cout<< "Predicted metrics (EV corrections): "<< std::endl  << control_params.delta_EVs_powers_matrix*cameras[0].frames[last].control_vars.polyfit_coeffs << std::endl;
    // std::cout << "Error on fitted scores: "<< std::endl << control_params.delta_EVs_powers_matrix*cameras[0].frames[last].control_vars.polyfit_coeffs-cameras[0].frames[last].control_vars.scores << std::endl;

    // Take the first and second derivatives
    Eigen::Matrix<float, _n_coeffs_polyfit-1, 1> polyfit_1stderiv_coeffs;
    Eigen::Matrix<float, _n_coeffs_polyfit-2, 1> polyfit_2ndderiv_coeffs;

    for(int i_coeff=1; i_coeff<_n_coeffs_polyfit; i_coeff++){
        polyfit_1stderiv_coeffs[i_coeff-1] = float(i_coeff)*cameras[0].frames[last].control_vars.polyfit_coeffs[i_coeff];
        if(i_coeff>1){
            polyfit_2ndderiv_coeffs[i_coeff-2] = float(i_coeff)*float(i_coeff-1)*cameras[0].frames[last].control_vars.polyfit_coeffs[i_coeff];
        }
    }

    // Debug
    // std::cout << "Polynomial coefficients: "<< std::endl << cameras[0].frames[last].control_vars.polyfit_coeffs << std::endl;
    // std::cout << "Polynomial 1st deriv. coefficients: "<< std::endl << polyfit_1stderiv_coeffs << std::endl;
    // std::cout << "Polynomial 2nd deriv. coefficients: "<< std::endl << polyfit_2ndderiv_coeffs << std::endl;
    // std::cout << "1st derivative at gamma=1.0: " << polyfit_1stderiv_coeffs.sum() << std::endl;
    // std::cout << "2nd derivative at gamma=1.0: " << polyfit_2ndderiv_coeffs.sum() << std::endl;

    cv::Scalar Imean_cv = cv::mean( cameras[0].frames[last].img );
    double Imean = Imean_cv.val[0]; // mean image illumination
    // std::cout << Imean << std::endl;

    // cv::Mat SaturatedPixels;
    // cv::compare(cameras[0].frames[last].img,250,SaturatedPixels,cv::CmpTypes::CMP_GE);
    // SaturatedPixels = SaturatedPixels;
    // cv::imshow("",SaturatedPixels);
    // cv::waitKey(10);

    // Find optimal gamma correction or optimal change in exposure value directly (based on photometric response function)
    // (Note we use Newton optim. instead of finding roots)

    float value_optim;
    if(corrections==GAMMA_CORRECTIONS){
        value_optim = 1.0; // init, gamma of 1.0 means no change
    }
    else if(corrections==EV_CORRECTIONS){
        value_optim = 0.0; // init, delta EV of 0.0 means no change
    }

    // Compute curve at the initial value for optimization (if positive, don't bother to try to find max.)
    float curve_at_value_optim_init = 0.0;
    for(int order=0; order<polyfit_2ndderiv_coeffs.size();order++){
        curve_at_value_optim_init += polyfit_2ndderiv_coeffs[order]*pow(value_optim,order);
    }

    
    if(Imean<control_params.Imin){
        std::cout << "Image is too dark !" << std::endl;
        // Iff too dark, select min exposure value OR max gamma (make darker)
        if(corrections==GAMMA_CORRECTIONS)   value_optim = control_params.gamma_values[control_params.gamma_values.size()-1];
        else if(corrections==EV_CORRECTIONS) value_optim = control_params.delta_EVs[0];
    }
    else if(Imean>control_params.Imax){
        std::cout << "Image is too bright !" << std::endl;
        // If too bright, select max exposure value OR min gamma (make darker)
        if(corrections==GAMMA_CORRECTIONS)   value_optim = control_params.gamma_values[0];
        else if(corrections==EV_CORRECTIONS) value_optim = control_params.delta_EVs[control_params.delta_EVs.size()-1];
    }
    else{ // Do the actual optimization 
        if(curve_at_value_optim_init >= 0){
            // If curve is positive, then just select the value associated with the max score.
            VectorNcorrect::Index max_index;
            cameras[0].frames[last].control_vars.scores.maxCoeff(&max_index);
            if(corrections==GAMMA_CORRECTIONS)   value_optim = control_params.gamma_values[max_index];
            else if(corrections==EV_CORRECTIONS) value_optim = control_params.delta_EVs[max_index];
        }
        else{
            bool  is_optim_done = false;
            int   n_iter_max = 10;
            float desired_res = pow(10.0,-5);
            int i_iter = 0;
            while(!is_optim_done){
                i_iter += 1;
                float eval_1stderiv = 0.0;
                float eval_2ndderiv = 0.0;
                for(int i_coeff=0; i_coeff<_n_coeffs_polyfit-1; i_coeff++)
                    {
                    eval_1stderiv += polyfit_1stderiv_coeffs[i_coeff]*pow(value_optim,i_coeff);
                    }
                for(int i_coeff=0; i_coeff<_n_coeffs_polyfit-2; i_coeff++)
                    {
                    eval_2ndderiv += polyfit_2ndderiv_coeffs[i_coeff]*pow(value_optim,i_coeff);
                    }
                float delta = eval_1stderiv/eval_2ndderiv;
                value_optim -= delta;

                if(abs(delta)<desired_res){is_optim_done=true;}
                else if(i_iter>=n_iter_max){
                    is_optim_done=true;
                    // std::cout << "Error: Optimization failed. Latest optimal gamma / deltaEV: "<< value_optim << std::endl;
                    // Optimization has failed. Use the best score instead.
                    VectorNcorrect::Index max_index;
                    cameras[0].frames[last].control_vars.scores.maxCoeff(&max_index);
                    if(corrections==GAMMA_CORRECTIONS)   value_optim = control_params.gamma_values[max_index];
                    else if(corrections==EV_CORRECTIONS) value_optim = control_params.delta_EVs[max_index];
                }
            }
        }

    } // end of the optimization
    

    // Debug
    // std::cout << "Optimization - New optim. value: " << value_optim << std::endl;
    
    if(corrections==GAMMA_CORRECTIONS)   cameras[0].frames[last].control_vars.gamma_optim = value_optim;
    else if(corrections==EV_CORRECTIONS) cameras[0].frames[last].control_vars.deltaEV_optim = value_optim;

    // Predicted optimal score at next frame
    Eigen::Matrix<float, 1, _n_order_polyfit+1> value_optim_powers_matrix;
    for (int i_order = 0; i_order < control_params.n_coeffs_polyfit; i_order++){
            value_optim_powers_matrix(0,i_order) = pow(value_optim,i_order);
        }
    cameras[0].frames[last].control_vars.next_frame_optim_score = value_optim_powers_matrix*cameras[0].frames[last].control_vars.polyfit_coeffs;
    
    // Get current exposure value
    cameras[0].frames[last].exposure_value = find_exposure_value(cameras[0].F,cameras[0].frames[last].exposure,cameras[0].frames[last].gain);
    // std::cout << "Current exposure value: "<< cameras[0].frames[last].exposure_value << std::endl;

    // Update exposure value
    if(corrections==GAMMA_CORRECTIONS){
        // Update according to Shim's 2018
        float R = control_params.d * tan( (2 - value_optim) * atan2(1,control_params.d) - atan2(1,control_params.d) ) + 1;
        cameras[0].frames[last].control_vars.exposure_value_next = (1 + control_params.kp * (R-1)) *cameras[0].frames[last].exposure_value; // Nonlinear update
        
        // Debug
        // cameras[0].frames[last].control_vars.exposure_value_next = (1 + control_params.kp * (1-value_optim)) *cameras[0].frames[last].exposure_value; // Linear update
        // std::cout << "Next exposure value: " << cameras[0].frames[last].control_vars.exposure_value_next << std::endl;
    }
    else if(corrections==EV_CORRECTIONS){
        // For EV corrections, we note the tendancy of the predictions to UNDERESTIMATE the amount of information contained in the saturated regions.
        // To compensate, we include a feedback mechanism which scales deltaEV_optim according to how much we actually DID improve the image over the last frame vs. 
        // how much we expected to improve it.
        float feedback_gain;
        if(last > 1){ 
            float predicted_delta_score = ExpGainController::cameras[0].frames[last-2].control_vars.next_frame_optim_score - ExpGainController::cameras[0].frames[last-2].control_vars.scores[3] ;
            float actual_delta_score = ExpGainController::cameras[0].frames[last].control_vars.scores[3]- ExpGainController::cameras[0].frames[last-2].control_vars.scores[3];
            float surprise_ratio = actual_delta_score/(predicted_delta_score+0.1);
            // std::cout << "Predicted change in optim score: " << predicted_delta_score << " Actual change in optim score: " << actual_delta_score << std::endl;
            // std::cout << "Ratio actual:predicted improvement " << surprise_ratio << std::endl;
            // if (predicted_delta_score<0.0){std::cout << "Predicted delta score is negative ! : " << predicted_delta_score << std::endl;}
            if (surprise_ratio>1.1 & predicted_delta_score>0.0){feedback_gain = 1.5;}
            else{feedback_gain = 1.0;}
            // feedback_gain = 1.0;
            // std::cout << "Warning, disabled saturation fb." << std::endl;
        }
        else{
            feedback_gain = 1.0;
        }
        cameras[0].frames[last].control_vars.exposure_value_next = cameras[0].frames[last].exposure_value + feedback_gain*cameras[0].frames[last].control_vars.deltaEV_optim;
    }
    
    
    // Estimate average pixel speed (in pixels/s) as proxy of motion-blur impact
    if (blur_estim==IMU_ESTIM){
        // Simple Method: Use pitch and yaw only based on IMU rotation (ignore translation and roll)
        if (imus.size()>0){
            if (imus[0].measures.size()>0){
                float mean_x_ang_speed = 0.0;
                float mean_y_ang_speed = 0.0;
                int n_meas = imus[0].measures.size();
                float factor = 1.0/float(n_meas);

                for (int i_meas = 0; i_meas < n_meas ; i_meas++){
                    mean_x_ang_speed += -factor*imus[0].measures[i_meas].ang_vel[1]; // TODO: Actually use IMU 2 cam transform
                    mean_y_ang_speed +=  factor*imus[0].measures[i_meas].ang_vel[2];
                }

                cameras[0].frames[last].twist[3] = mean_x_ang_speed;
                cameras[0].frames[last].twist[4] = mean_y_ang_speed;
                cameras[0].frames[last].avg_pixel_speed = sqrt(pow(cameras[0].frames[last].twist[3]*cameras[0].px_per_rad_y,2) + pow(cameras[0].frames[last].twist[4]*cameras[0].px_per_rad_x,2));
                
                // imus[0].measures.clear(); // clear imu buffer
            }
        }
    }
    else if(blur_estim==OPTIFLOW_ESTIM & cameras[0].frames.size()>1){
        cv::Mat flow;
        cv::Mat current_frame_lowres, last_frame_lowres;
        int factor_lowres = 4;
        cv::Size size_lowres(control_params.down_size_w/factor_lowres,control_params.down_size_h/factor_lowres);
        cv::resize(cameras[0].frames[last-1].img,last_frame_lowres,size_lowres);
        cv::resize(cameras[0].frames[last].img,current_frame_lowres,size_lowres);
        cv::calcOpticalFlowFarneback(last_frame_lowres,current_frame_lowres,flow,0.5,1,5,3,5,1.1,0);

        std::vector<cv::Mat> flow_channels(2);
        cv::Mat flow_abs;
        cv::split(flow, flow_channels);
        // Rescale according to original image size
        flow_channels[0] = flow_channels[0]*cameras[0].width/control_params.down_size_w*factor_lowres;
        flow_channels[1] = flow_channels[1]*cameras[0].height/control_params.down_size_h*factor_lowres;
        // Combine x and y displacements
        cv::sqrt(flow_channels[0].mul(flow_channels[0]) + flow_channels[1].mul(flow_channels[1]),flow_abs);
        
        cv::Scalar flow_abs_mean;
        flow_abs_mean =  cv::mean(flow_abs);

        cameras[0].frames[last].avg_pixel_speed = flow_abs_mean[0]*flow_abs_mean[0];

        // Debug
        // std::cout << "Flow mean value: " << flow_abs_mean << std::endl;
        // cv::Mat flow_abs_scaled;
        // flow_abs.convertTo(flow_abs_scaled,CV_8UC1); // scale and convert
        // cv::imshow("",flow_abs_scaled);
        // cv::waitKey(4);
    }
    
    // std::cout << "Average motion blur: "<< cameras[0].frames[last].avg_pixel_speed << std::endl;
    
    // Update exposure time and gain (through finding optimal balance)
    if (cameras[0].frames[last].control_vars.exposure_value_next > cameras[0].exposure_value_max){
        std::cout << "Warning: Maximum exposure value reached ! Setting exposure time and gain to LOWER limits." << std::endl;
        std::cout << "Current exposure value: " << cameras[0].frames[last].exposure_value << std::endl;
        std::cout << "Required exposure value: " << cameras[0].frames[last].control_vars.exposure_value_next << std::endl;
        std::cout << "Maximum exposure value: " << cameras[0].exposure_value_max << std::endl;
        cameras[0].frames[last].control_vars.exposure_next = cameras[0].exposure_min;
        cameras[0].frames[last].control_vars.gain_next     = cameras[0].gain_min; 
    }
    else if(cameras[0].frames[last].control_vars.exposure_value_next < cameras[0].exposure_value_min){
        std::cout << "Warning: Minimum exposure value reached ! Setting exposure time and gain to UPPPER limits." << std::endl;
        std::cout << "Current exposure value: " << cameras[0].frames[last].exposure_value << std::endl;
        std::cout << "Required exposure value: " << cameras[0].frames[last].control_vars.exposure_value_next << std::endl;
        std::cout << "Minimum exposure value: " << cameras[0].exposure_value_min << std::endl;
        cameras[0].frames[last].control_vars.exposure_next = cameras[0].exposure_max;
        cameras[0].frames[last].control_vars.gain_next     = cameras[0].gain_max; 
    }
    else{
        // Consider the cost function: cost = avg_pixel_speed*exp_time + w(gain-1)
        // where gain>1.0 is a factor (NOT in dB), w>0 is wheight, avg_pixel_speed>0, exp_time > 0 
        // s.t. exp_time*gain = c (where c = F^2/(2^exposure_value) is a constant determined by the exposure value)
        // --> The solution is gain* = sqrt(avg_pixel_speed*c/w) and exp_time* = c/gain*
        // --> In case gain* > gain_max or gain*<1.0, set gain* to limit and compute exp_time as exp_time* = c/gain*.
        // --> If exp_time* > exp_time
        float g_optim_offset = 0*20.0; // dB_2_factor(5.0); // explicit offset
        float offset = 8.0;
        float c = pow(cameras[0].F,2)*pow(2,-cameras[0].frames[last].control_vars.exposure_value_next);
        float gain_not_dB_tmp = sqrt((cameras[0].frames[last].avg_pixel_speed*c+offset)/control_params.w) + g_optim_offset;
        float gain_dB_tmp = factor_2_dB(gain_not_dB_tmp);

        // Now make sure that gain and exposure are within bounds
        if(gain_dB_tmp<cameras[0].gain_min){
            gain_dB_tmp = cameras[0].gain_min;
        }
        else if(gain_dB_tmp>cameras[0].gain_max){
            gain_dB_tmp = cameras[0].gain_max;
        }

        gain_not_dB_tmp = dB_2_factor(gain_dB_tmp);

        float exposure_tmp = c/gain_not_dB_tmp;
        if(exposure_tmp<cameras[0].exposure_min){
            exposure_tmp = cameras[0].exposure_min;
        }
        else if(exposure_tmp>cameras[0].exposure_max){
            exposure_tmp = cameras[0].exposure_max;
        }

        gain_not_dB_tmp = c/exposure_tmp;
        gain_dB_tmp = factor_2_dB(gain_not_dB_tmp);
        
        

        // Make sure that the exposure/gain parameters are not updated too quickly
        // float delta_gain_max = 0.5;
        // float delta_exp_max = 0.001;

        // bool is_gain_update_too_quick = abs(gain_dB_tmp-cameras[0].frames[last].gain) > delta_gain_max;

        // if(is_gain_update_too_quick){
        //     gain_dB_tmp = cameras[0].frames[last].gain+sign(gain_dB_tmp-cameras[0].frames[last].gain)*delta_gain_max;
        //     gain_not_dB_tmp = dB_2_factor(gain_dB_tmp);
        //     exposure_tmp = c/gain_not_dB_tmp;
        // }

        // bool is_exposure_update_too_quick = abs(exposure_tmp-cameras[0].frames[last].exposure) > delta_exp_max;

        // if(is_exposure_update_too_quick){
        //     exposure_tmp = cameras[0].frames[last].exposure+sign(exposure_tmp-cameras[0].frames[last].exposure)*delta_exp_max;
        //     gain_not_dB_tmp = c/exposure_tmp;
        //     gain_dB_tmp = factor_2_dB(gain_not_dB_tmp);
        // }




        // Save the new parameters in the control structure (for log)
        cameras[0].frames[last].control_vars.gain_next = gain_dB_tmp;
        cameras[0].frames[last].control_vars.exposure_next = exposure_tmp;

        // Timing
        auto t2 = high_resolution_clock::now();
        auto ms_int = duration_cast<milliseconds>(t2 - t1);
        // std::cout << "Time spent updating: " << ms_int.count() << "ms" << std::endl;

        
    } // end update exposure time and gain

    // Return the exposure parameters
    ExposureParameters new_exposure_params;
    new_exposure_params.exposure = cameras[0].frames[last].control_vars.exposure_next;
    new_exposure_params.gain = cameras[0].frames[last].control_vars.gain_next;

        

    // Debug
    // std::cout << "-------------------------------------------------" << std::endl;
    // std::cout << "Old gain [dB]: "<< cameras[0].frames[last].gain << ", New gain [dB]: "<< cameras[0].frames[last].control_vars.gain_next << std::endl;
    // std::cout << "Old exposure time [ms]: "<< cameras[0].frames[last].exposure*1000 << ", New exposure time [ms]: "<< cameras[0].frames[last].control_vars.exposure_next*1000 << std::endl;
    // std::cout << "Old exposure value: " << cameras[0].frames[last].exposure_value << ", New exposure value: " << cameras[0].frames[last].control_vars.exposure_value_next << std::endl;
    // std::cout << cameras[0].frames.size() << std::endl;
    
    
    // Preperation for next iteration
    if(last > 1){ 
        // Debug: Printing out current params, last commands, timestamp
        // std::cout << 1000*ExpGainController::cameras[0].frames[last-2].control_vars.exposure_next << " / " << 1000*ExpGainController::cameras[0].frames[last-1].control_vars.exposure_next << " : " << 1000*ExpGainController::cameras[0].frames[last].exposure << std::endl;
        // std::cout << ExpGainController::cameras[0].frames[last-2].control_vars.gain_next << " / " << ExpGainController::cameras[0].frames[last-1].control_vars.gain_next << " : " << ExpGainController::cameras[0].frames[last].gain << std::endl;
        // std::cout << now_str() << '\n';

        ExpGainController::cameras[0].frames.erase(ExpGainController::cameras[0].frames.begin()); // Make sure we keep 2 frames max. in the buffer
    }

    return new_exposure_params;
} // End of update params

