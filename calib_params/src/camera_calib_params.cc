#include "camera_calib_params.hh"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>

using namespace std;

bool CameraCalibParams::load(const string &path){

  YAML::Node node = YAML::LoadFile(path.c_str());

  yaml_file_path = path;
  device_model = node["device_model"].as<std::string>();
  robot_conf	 = node["robot_conf"].as<std::string>();
  is_color	 = node["is_color"].as<bool>();
  mask_path	 = node["dead_regions"].as<std::string>();
    unique_id = node["unique_id"].as<std::string>();

  int slash_idx = path.find_last_of("/\\");

  image_mask	 = cv::imread(path.substr(0, slash_idx + 1) + mask_path, CV_LOAD_IMAGE_GRAYSCALE);

  image_width	 = node["image_width"].as<int>();
  image_height = node["image_height"].as<int>();

  // "relative_pose" matrix has to be 3-by-4
  relative_pose = Eigen::Matrix4d::Identity();
  for(int r = 0 ; r < 3 ; r++)
    for(int c = 0 ; c < 4 ; c++)
      relative_pose(r, c) = node["relative_pose"]["data"][r * 4 + c].as<double>();

  dist_model = node["distortion_model"].as<std::string>();

  // "distortion_coefficients" has to be a vector. Its size is dependent
  // on the camera calibration toolbox and type of the lens.
  int num_dist_coeffs =	node["distortion_coefficients"]["rows"].as<int>() *
    node["distortion_coefficients"]["cols"].as<int>();
  dist_coeffs = cv::Mat::zeros(num_dist_coeffs, 1, CV_64F);
  for(int r = 0 ; r < num_dist_coeffs ; r++)
    dist_coeffs.at<double>(r, 0) = node["distortion_coefficients"]["data"][r].as<double>();

  // "camera_matrix" has to be 3-by-3
  camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
  for(int r = 0 ; r < 3 ; r++)
    for(int c = 0 ; c < 3 ; c++)
      camera_matrix.at<double>(r, c) = node["camera_matrix"]["data"][r * 3 + c].as<double>();

  projection_matrix = cv::Mat::zeros(3, 4, CV_64F);
  for(int r = 0 ; r < 3 ; r++)
    for(int c = 0 ; c < 4 ; c++)
      projection_matrix.at<double>(r, c) = node["projection_matrix"]["data"][r * 4 + c].as<double>();
  return true;
}

bool CameraCalibParams::save(const string &path, bool overwrite){
  return true;
}

bool CameraCalibParams::from_message(const calib_params::CameraCalibMsg &msg){
  return true;
}

bool CameraCalibParams::to_message(calib_params::CameraCalibMsg &msg){
  msg.device_model = device_model;
  msg.robot_conf   = robot_conf;
  msg.dist_model   = dist_model;
  msg.dead_regions = mask_path;
  msg.is_color     = is_color;
  msg.image_height = image_height;
  msg.image_width  = image_width;

  for(int r = 0 ; r < 3 ; r++)
    for(int c = 0 ; c < 4 ; c++)
      msg.relative_pose[r * 4 + c] = relative_pose(r, c);

  for(int r = 0 ; r < 3 ; r++)
    for(int c = 0 ; c < 3 ; c++)
      msg.camera_matrix[r * 3 + c] = camera_matrix.at<double>(r, c);

  for(int r = 0 ; r < 3 ; r++)
    for(int c = 0 ; c < 4 ; c++)
      msg.projection_matrix[r * 4 + c] = projection_matrix.at<double>(r, c);

  for(int i = 0 ; i < 9 ; i++)
    msg.dist_coeffs[i] = 0;
  for(int i = 0 ; i < dist_coeffs.rows ; i++)
    msg.dist_coeffs[i] = dist_coeffs.at<double>(i, 0);

  return true;
}

bool CameraCalibParams::print(){
  cout << "-----------------------------------------------------" << endl;
  cout << "yaml_file_path : "	<< yaml_file_path << endl;
  cout << "device_model   : " << device_model << endl;
  cout << "robot_conf     : " << robot_conf << endl;
  cout << "unique_id      : " << unique_id << endl;
  cout << "is_color       : " << (is_color ? "TRUE" : "FALSE") << endl;
  cout << "mask_path      : " << mask_path << endl;
  cout << "dist_coeffs    : " << dist_coeffs << endl;
  cout << "camera_matrix  :\n>>" << camera_matrix << endl;
  cout << "proj_matrix    :\n>>" << projection_matrix << endl;
  cout << "dist_model     : " << dist_model << endl;
  cout << "image_[w, h]   : "	<< "[" << image_width << ", " << image_height << "]" << endl;
  cout << "relative_pose  :\n>>"	<< relative_pose << endl;
  cout << "-----------------------------------------------------" << endl;
  return true;
}

