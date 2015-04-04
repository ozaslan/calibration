#include "lidar_calib_params.hh"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>

using namespace std;

bool LidarCalibParams::load(const string &path){

	//cout << "path : " << path << endl;
	YAML::Node node = YAML::LoadFile(path.c_str());

	yaml_file_path = path;

	device_model = node["device_model"].as<std::string>();
	//cout << "device_mode : " << device_model << endl;
	robot_conf	 = node["robot_conf"].as<std::string>();
	//cout << "robot_conf : " << robot_conf << endl;
  unique_id	 = node["unique_id"].as<std::string>();

	min_range	 = node["min_range"].as<double>();
	//cout << "min_range : " << min_range << endl;
	max_range    = node["max_range"].as<double>();
	//cout << "max_range : " << max_range << endl;


	knee_range	 = node["noise_model"]["knee_range"].as<double>();
	var			 = node["noise_model"]["var"].as<double>();
	perc		 = node["noise_model"]["perc"].as<double>();

	//cout << "knee_range : " << knee_range << endl;
	//cout << "var : " << var << endl;
	//cout << "perc : " << perc << endl;

	// "relative_pose" matrix has to be 3-by-4
	relative_pose = Eigen::Matrix4d::Zero();
	for(int r = 0 ; r < 3 ; r++)
		for(int c = 0 ; c < 4 ; c++){
			relative_pose(r, c) = node["relative_pose"]["data"][r * 4 + c].as<double>();
			//cout << "relative_pose(" << r << ", " << c << ") = " << relative_pose(r, c) << endl;
		}

	cout << "node[dead_regions].size() : " << (int)node["dead_regions"].size() << endl;   
	for(int i = 0 ; i < (int)node["dead_regions"].size() ; i++)
		dead_regions.push_back(node["dead_regions"][i].as<double>());
	cout << "dead_regions.size() : " << dead_regions.size() << endl;


	for(int i = 0 ; i < (int)node["upwards_mirror_regions"].size() ; i++)
		dead_regions.push_back(node["upwards_mirror_regions"][i].as<double>());
	for(int i = 0 ; i < (int)node["downwards_mirror_regions"].size() ; i++)
		dead_regions.push_back(node["downward_mirror_regions"][i].as<double>());

	return true;
}

bool LidarCalibParams::save(const string &path, bool overwrite){
	return true;
}
	
bool LidarCalibParams::from_message(const calib_params::LidarCalibMsg &msg){
	return true;
}

bool LidarCalibParams::to_message(calib_params::LidarCalibMsg &msg){
	msg.device_model = device_model;
	msg.robot_conf   = robot_conf;
	msg.min_range	 = min_range;
	msg.max_range    = max_range;
	msg.knee_range	 = knee_range;
	msg.var			 = var;
	msg.perc		 = perc;

	msg.dead_regions = dead_regions;
	msg.upwards_mirror_regions   = upwards_mirror_regions;
	msg.downwards_mirror_regions = downwards_mirror_regions;

	for(int r = 0 ; r < 3 ; r++)
		for(int c = 0 ; c < 4 ; c++)
			msg.relative_pose[r * 4 + c] = relative_pose(r, c);
	
	return true;
}

bool LidarCalibParams::print(){
	cout << "-----------------------------------------------------" << endl;
	cout << "yaml_file_path : "	<< yaml_file_path << endl;
	cout << "device_model   : " << device_model << endl;
	cout << "robot_conf     : " << robot_conf << endl;
	cout << "unique_id      : " << unique_id << endl;
	cout << "relative_pose  : \n" << relative_pose << endl;
	cout << "knee_range     : " << knee_range << endl;
	cout << "[var, perc]    : " << "[" << var << ", " << perc << "]" << endl;
	for(int i = 0 ; i < (int)dead_regions.size() ; i+=2)
		cout << "dead_regions[" << i/2 << "] : [" << dead_regions[i] << ", " << dead_regions[i+1] << "]" << endl;
	for(int i = 0 ; i < (int)upwards_mirror_regions.size() ; i+=2)
		cout << "upwards_mirror_regions[" << i/2 << "] : [" << upwards_mirror_regions[i] << ", " << upwards_mirror_regions[i+1] << "]" << endl;
	for(int i = 0 ; i < (int)downwards_mirror_regions.size() ; i+=2)
		cout << "downwards_mirror_regions[" << i/2 << "] : [" << downwards_mirror_regions[i] << ", " << downwards_mirror_regions[i+1] << "]" << endl;

	cout << "-----------------------------------------------------" << endl;
	return true;
}

