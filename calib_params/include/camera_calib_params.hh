#ifndef __CAMERA_CALIB_PARAMS_HH__
#define __CAMERA_CALIB_PARAMS_HH__

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <calib_params/CameraCalibMsg.h>
#include <opencv2/opencv.hpp>

using namespace std;

class CameraCalibParams{
  private:

  public:
    // Path to the Yaml file from which the class has been populated.
    string yaml_file_path;
    // Device model for informative reasons: e.g. Matrix Vision Bluefox ???
    string device_model;
    // Name, identifier of the platform/robot on which the sensor is
    // mounted.
    string robot_conf;
    // Unique ID of the parameter set
    string unique_id;
    // This tells whether the camera can supply color image
    // and does not necesarily reflect current mode of the device
    bool is_color;
    // This is the path to the black-white image which defines
    // the dead and usable regions of the frames.
    string mask_path;
    // Image mask is mostly used in OpenCV function hence loaded
    // an OpenCV matrix. 
    cv::Mat image_mask;
    // Similary the following parameters are loaded as OpenCV
    // matrices. 
    cv::Mat dist_coeffs;		// n-by-1 double vector
    cv::Mat camera_matrix;	// 3-by-3 double matrix
    // Distortion model of the camera. Depending on this value, 
    // different undistortion funtions can be called (fisheye and 
    // narrow FOV lenses has to be treated differently in some case)
    string dist_model;
    // 'image_(width/height)' define the resolution at which the
    // camera calibration has been carried.
    int image_width, image_height;
    // Pose of the camera relative to the platform it is mounted on.
    // This maps vectors in camera frame to platform frame.
    Eigen::Matrix4d relative_pose;
    // This function gets the path to the Yaml file and populates
    // the class. It returns false if the path does not exist or
    // Yaml file file has wrong format.
    bool load(const string &path);
    // This function saves the parameters in Yaml format. It returns
    // true if Yaml file can be initialized. If overwrite is false
    // and a file with the same name exists, then the function
    // returns false.
    bool save(const string &path, bool overwrite = false);
    // Convert from corresponding ROS message type
    bool from_message(const calib_params::CameraCalibMsg &msg);
    // Convert to corresponding ROS message type
    bool to_message(calib_params::CameraCalibMsg &msg);
    // Print the paramters on the std. output
    bool print();

};

#endif
