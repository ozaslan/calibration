#ifndef __RGBD_CALIB_PARAMS_HH__
#define __RGBD_CALIB_PARAMS_HH__

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
// #include <RGBDCalibMsg.h>

using namespace std;

class RGBDCalibParams{
private:
	
public:
	// Path to the Yaml file from which the class has been populated.
	string yaml_file_path;
	// Device model for informative reasons: e.g. Hokuyo UST-20LX
	string device_model;
	// Name, identifier of the platform/robot on which the sensor is
	// mounted.
	string robot_conf;
	// This is the path to the black-white image which defines
	// the dead and usable regions of the frames.
	string mask_path;
	// Image mask is mostly used in OpenCV function hence loaded
	// an OpenCV matrix. 
	cv::Mat image_mask;
	// Maximum and minimum ranges measurable by the device. This
	// information should be from the device manual. Range is in
	// meters.
	double	min_range,
			max_range;
	// ### I have to add noise model as well. Sth. similar to lidar maybe.
	// Similary the following parameters are loaded as OpenCV
	// matrices. 
	cv::Mat dist_coeff;		// n-by-1 double vector
	cv::Mat camera_matrix;	// 3-by-3 double matrix
	// 'image_(width/height)' define the resolution at which the
	// camera calibration has been carried.
	int image_width, image_height;
	// Relative pose of the sensor w.r.t. the platform on which the
	// sensor is mounted. Application of the matrix maps vectors in
	// sensor frame to platform frame.
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
	// Convert to corresponding ROS message type 
	// ### bool to_message(calib_params &LidarCalibMsg &msg);
	// Convert from corresponding ROS message type
	// ### bool from_message(const calib_params &LidarCalibMsg &msg);	
};

#endif
