//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include <ctime>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <torch/script.h>
#include "iostream"
#include <cmath>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;
torch::jit::script::Module module; 
static int current_line_num_;


DEFINE_double(cp1_distance, 2.5, "Distance to travel for 1D TOC (cp1)");
DEFINE_double(cp1_curvature, 0.5, "Curvature for arc path (cp1)");

DEFINE_double(cp2_curvature, 0.5, "Curvature for arc path (cp2)");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n, const string& joystick_mappings) :
    
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  // MODEL LOADING HERE
  try {
    module = torch::jit::load("/home/amrl_user/traced_ikd_model.pt");
  }
  catch (const c10::Error& e) {
    std::cerr << "Error loading the model: " << e.what() << std::endl;
    // Handle error appropriately
    // sys.exit(-1)
  }
  std::cout << "Model loading finished\n";

  // Read in text file for joystick mappings (changed code, passed by constructor)
  std::ifstream mappings_file(joystick_mappings);
  std::string line;
  // Populate lines with all the text file input.
  if (mappings_file.is_open()) {
    std::string line;
    while (std::getline(mappings_file, line)) {
      lines.push_back(line);
    }
    mappings_file.close();
  } else {
    ROS_ERROR_STREAM("Failed to open joystick mappings file: " << joystick_mappings);
  }
  // Set current line number to 0.
  current_line_num_ = 0;

}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

// this function gets called 20 times a second to form the control loop.
void Navigation::Run() {
    // Clear previous visualizations.
    visualization::ClearVisualizationMsg(local_viz_msg_);
    visualization::ClearVisualizationMsg(global_viz_msg_);

    if (!odom_initialized_) return;

    int current_line_num_copy_ = current_line_num_;
    for (std::list<std::string>::iterator it = lines.begin(); it != lines.end(); ++it) {
      if (current_line_num_copy_  > 0) {
        // Skip the lines that have already been read
        current_line_num_copy_--;
        continue;
      }
      // Access the next line in the list
      std::string line = *it;
      std::string value1 = line.substr(1, line.find(",") - 1);
      std::string value2 = line.substr(line.find(",") + 2, line.length() - line.find(",") - 3);
      float double_value1 = std::stof(value1);
      float double_value2 = std::stof(value2);
      // std::cout << "Value 1: " << double_value1 << ", Value 2: " << double_value2 << std::endl;
      float curvature = 0.0;
      if (double_value1 == 0.0) {
        curvature = 0.0;
      } else {
        curvature = double_value2 / double_value1;
      }
      drive_msg_.velocity = double_value1;
      drive_msg_.curvature = curvature;
      
      // Save the current line number
      current_line_num_++;
      std::cout << current_line_num_ << std::endl;

      // Break out of the loop after reading one line
      break;

    }
    // Custom timestamp and quick publish for drive node
    drive_msg_.header.stamp = ros::Time::now();
    drive_pub_.publish(drive_msg_);
    // Add timestamps to all messages.
    // local_viz_msg_.header.stamp = ros::Time::now();
    // global_viz_msg_.header.stamp = ros::Time::now();
    // drive_msg_.header.stamp = ros::Time::now();

    // Publish messages.
    // viz_pub_.publish(local_viz_msg_);
    // viz_pub_.publish(global_viz_msg_);
    // drive_pub_.publish(drive_msg_);




    // IKD CODE:

    // float curvature = 0.8;
    // float velocity = 2.0;
    // float angular_velocity = velocity * curvature;
    // float input_velocity = velocity;
    // std::vector<float> input = {input_velocity, angular_velocity};
    // at::Tensor tensor = torch::from_blob(input.data(), {1, 2}, torch::kFloat32);
    // std::vector<torch::jit::IValue> inputs;
    // inputs.push_back(tensor);
    // std::cout << tensor << std::endl;
    // // run the model
    // at::Tensor output = module.forward(inputs).toTensor();
    // std::cout << output << std::endl;
    // float out = *output[0].data_ptr<float>();
    // // std::cout << out << std::endl;
    // float corr_curvature;
    // if (velocity != 0) {
    // 	corr_curvature = out / velocity;
    // } else {
    // 	corr_curvature = 0;
    // }
    // drive_msg_.velocity = velocity;
    // drive_msg_.curvature = corr_curvature;
  }
}

