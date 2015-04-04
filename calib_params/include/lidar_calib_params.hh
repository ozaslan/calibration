#ifndef __LIDAR_CALIB_PARAMS_HH__
#define __LIDAR_CALIB_PARAMS_HH__

#include <vector>
#include <string>
#include <Eigen/Dense>
#include "calib_params/LidarCalibMsg.h"

using namespace std;

class LidarCalibParams{
private:
	
public:
	// Path to the Yaml file from which the class has been populated.
	string yaml_file_path;
	// Device model for informative reasons: e.g. Hokuyo UST-20LX
	string device_model;
	// Name, identifier of the platform/robot on which the sensor is
	// mounted.
	string robot_conf;
	// Identifier string for the device.
	string unique_id;
	// Maximum and minimum ranges measurable by the device. This
	// information should be from the device manual. Range is in
	// meters.
	double	min_range,
			max_range;
	// Noise model parameters of a single beam. 'knee_range' is the
	// range before which the noise variance is constant 'var' and
	// after which variance is 'perc * range'.
	double	knee_range,
			var, perc;
	// Angular spans along which the rays are useless possibly due to 
	// mechanical occlusion due to cabling, mount design etc.
	// The even numbered elements should be interpreted as start and
	// end angles of such regions. Angles are in radians.
	vector<double> dead_regions;
	// Relative pose of the sensor w.r.t. the platform on which the
	// sensor is mounted. Application of the matrix maps vectors in
	// sensor frame to platform frame.
	Eigen::Matrix4d relative_pose;
	// Regions similar to 'dead_regions' which designate angles
	// in between which rays are reflected to -z/+z directions
	// in sensor frame.
	vector<double>   upwards_mirror_regions,
				   downwards_mirror_regions;
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
	bool to_message(calib_params::LidarCalibMsg &msg);
	// Convert from corresponding ROS message type
	bool from_message(const calib_params::LidarCalibMsg &msg);	
	// Print the paramters on the std. output
	bool print();

};

#endif
