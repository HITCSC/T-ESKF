/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "ROSVisualizerHelper.h"

#include "core/VioManager.h"
#include "sim/Simulator.h"
#include "state/State.h"
#include "state/StateHelper.h"

#include "types/PoseJPL.h"

using namespace ov_msckf;
using namespace std;

#if ROS_AVAILABLE == 1
sensor_msgs::PointCloud2 ROSVisualizerHelper::get_ros_pointcloud(const std::vector<Eigen::Vector3d> &feats) {

  // Declare message and sizes
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "global";
  cloud.header.stamp = ros::Time::now();
  cloud.width = feats.size();
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false; // there may be invalid points

  // Setup pointcloud fields
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(feats.size());

  // Iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

  // Fill our iterators
  for (const auto &pt : feats) {
    *out_x = (float)pt(0);
    ++out_x;
    *out_y = (float)pt(1);
    ++out_y;
    *out_z = (float)pt(2);
    ++out_z;
  }

  return cloud;
}

tf::StampedTransform ROSVisualizerHelper::get_stamped_transform_from_pose(const std::shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans) {

  // Need to flip the transform to the IMU frame
  Eigen::Vector4d q_ItoC = pose->quat();
  Eigen::Vector3d p_CinI = pose->pos();
  if (flip_trans) {
    p_CinI = -pose->Rot().transpose() * pose->pos();
  }

  // publish our transform on TF
  // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
  // NOTE: a rotation from ItoC in JPL has the same xyzw as a CtoI Hamilton rotation
  tf::StampedTransform trans;
  trans.stamp_ = ros::Time::now();
  tf::Quaternion quat(q_ItoC(0), q_ItoC(1), q_ItoC(2), q_ItoC(3));
  trans.setRotation(quat);
  tf::Vector3 orig(p_CinI(0), p_CinI(1), p_CinI(2));
  trans.setOrigin(orig);
  return trans;
}
#endif

#if ROS_AVAILABLE == 2
sensor_msgs::msg::PointCloud2 ROSVisualizerHelper::get_ros_pointcloud(std::shared_ptr<rclcpp::Node> node,
                                                                      const std::vector<Eigen::Vector3d> &feats) {

  // Declare message and sizes
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "global";
  cloud.header.stamp = node->now();
  cloud.width = feats.size();
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false; // there may be invalid points

  // Setup pointcloud fields
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(feats.size());

  // Iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

  // Fill our iterators
  for (const auto &pt : feats) {
    *out_x = (float)pt(0);
    ++out_x;
    *out_y = (float)pt(1);
    ++out_y;
    *out_z = (float)pt(2);
    ++out_z;
  }

  return cloud;
}

geometry_msgs::msg::TransformStamped ROSVisualizerHelper::get_stamped_transform_from_pose(std::shared_ptr<rclcpp::Node> node,
                                                                                          const std::shared_ptr<ov_type::PoseJPL> &pose,
                                                                                          bool flip_trans) {

  // Need to flip the transform to the IMU frame
  Eigen::Vector4d q_ItoC = pose->quat();
  Eigen::Vector3d p_CinI = pose->pos();
  if (flip_trans) {
    p_CinI = -pose->Rot().transpose() * pose->pos();
  }

  // publish our transform on TF
  // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
  // NOTE: a rotation from ItoC in JPL has the same xyzw as a CtoI Hamilton rotation
  geometry_msgs::msg::TransformStamped trans;
  trans.header.stamp = node->now();
  trans.transform.rotation.x = q_ItoC(0);
  trans.transform.rotation.y = q_ItoC(1);
  trans.transform.rotation.z = q_ItoC(2);
  trans.transform.rotation.w = q_ItoC(3);
  trans.transform.translation.x = p_CinI(0);
  trans.transform.translation.y = p_CinI(1);
  trans.transform.translation.z = p_CinI(2);
  return trans;
}
#endif

void ROSVisualizerHelper::sim_save_total_state_to_file(std::shared_ptr<State> state, std::shared_ptr<Simulator> sim,
                                                       std::ofstream &of_state_est, std::ofstream &of_state_std,
                                                       std::ofstream &of_state_gt) {

  // We want to publish in the IMU clock frame
  // The timestamp in the state will be the last camera time
  double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
  double timestamp_inI = state->_timestamp + t_ItoC;

  // If we have our simulator, then save it to our groundtruth file
  if (sim != nullptr) {

    // Note that we get the true time in the IMU clock frame
    // NOTE: we record both the estimate and groundtruth with the same "true" timestamp if we are doing simulation
    Eigen::Matrix<double, 17, 1> state_gt;
    timestamp_inI = state->_timestamp + sim->get_true_parameters().calib_camimu_dt;
    if (sim->get_state(timestamp_inI, state_gt)) {
      // STATE: write current true state
      of_state_gt.precision(5);
      of_state_gt.setf(std::ios::fixed, std::ios::floatfield);
      of_state_gt << state_gt(0) << " ";
      of_state_gt.precision(6);
      of_state_gt << state_gt(1) << " " << state_gt(2) << " " << state_gt(3) << " " << state_gt(4) << " ";
      of_state_gt << state_gt(5) << " " << state_gt(6) << " " << state_gt(7) << " ";
      of_state_gt << state_gt(8) << " " << state_gt(9) << " " << state_gt(10) << std::endl;
    }
  }

  //==========================================================================
  //==========================================================================
  //==========================================================================

  // Get the covariance of the whole system
  Eigen::MatrixXd cov = StateHelper::get_full_covariance(state);

  // if (state->_options.do_tekf) {
  //   Eigen::Matrix<double, 6, 6> T_angle_inv = Eigen::Matrix<double, 6, 6>::Identity();
  //   T_angle_inv.block<3, 3>(0, 0) = state->_imu->Rot();
  //   T_angle_inv.block<3, 3>(3, 0) = -ov_core::skew_x(state->_imu->pos());
  //   cov.block<6, 6>(0, 0) = T_angle_inv * cov.block<6, 6>(0, 0) * T_angle_inv.transpose();
  // }

  // STATE: Write the current state to file
  of_state_est.precision(5);
  of_state_est.setf(std::ios::fixed, std::ios::floatfield);
  of_state_est << timestamp_inI << " ";
  of_state_est.precision(6);
  of_state_est << state->_imu->quat()(0) << " " << state->_imu->quat()(1) << " " << state->_imu->quat()(2) << " " << state->_imu->quat()(3)
               << " ";
  of_state_est << state->_imu->pos()(0) << " " << state->_imu->pos()(1) << " " << state->_imu->pos()(2) << " ";
  of_state_est << state->_imu->vel()(0) << " " << state->_imu->vel()(1) << " " << state->_imu->vel()(2) << " ";

  of_state_est.precision(10);

  for (int i = 0; i < 6; i++) {
    for (int j = i; j < 6; j++) {
      if (i + j < 10)
        of_state_est << cov(i, j) << " ";
      else 
        of_state_est << cov(i, j) << std::endl;
    }
  }
  // of_state_est << cov(0, 0) << " " << cov(0, 1) << " " << cov(0, 2) << " " << cov(1, 1) << " " << cov(1, 2) << " " << cov(2, 2) << " ";
  // of_state_est << cov(3, 3) << " " << cov(3, 4) << " " << cov(3, 5) << " " << cov(4, 4) << " " << cov(4, 5) << " " << cov(5, 5)
  //              << std::endl;
}
