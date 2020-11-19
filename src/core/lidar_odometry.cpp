/*
 * LI_Calib: An Open Platform for LiDAR-IMU Calibration
 * Copyright (C) 2020 Jiajun Lv
 * Copyright (C) 2020 Kewei Hu
 * Copyright (C) 2020 Jinhong Xu
 * Copyright (C) 2020 LI_Calib Contributors
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
#include <core/lidar_odometry.h>
#include <utils/pcl_utils.h>
#include <utils/math_utils.h>

namespace licalib {

LiDAROdometry::LiDAROdometry(double ndt_resolution)
        : map_cloud_(new VPointCloud()) {
  ndt_omp_ = ndtInit(ndt_resolution);
}

pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr
LiDAROdometry::ndtInit(double ndt_resolution) {
  auto ndt_omp = pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr(
          new pclomp::NormalDistributionsTransform<VPoint, VPoint>());
  ndt_omp->setResolution(ndt_resolution);
  ndt_omp->setNumThreads(4);
  ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt_omp->setTransformationEpsilon(1e-3);
  ndt_omp->setStepSize(0.01);
  ndt_omp->setMaximumIterations(50);
  return ndt_omp;
}

/**
 * @brief 每次添加一帧就做一次NDT变换，以第一帧Lidar为参考坐标系
 * 
 * @param timestamp 当前帧点云的时间戳(s)
 * @param cur_scan 当前帧点云
 * @param pose_predict 位姿的初始值(当前帧相对于上一帧)
 * @param update_map 是否更新source点云
 */
void LiDAROdometry::feedScan(double timestamp,
                             VPointCloud::Ptr cur_scan,
                             Eigen::Matrix4d pose_predict,
                             const bool update_map) {
  OdomData odom_cur;
  odom_cur.timestamp = timestamp;
  odom_cur.pose = Eigen::Matrix4d::Identity();

  VPointCloud::Ptr scan_in_target(new VPointCloud());
  if (map_cloud_->empty()) {
    scan_in_target = cur_scan;
  } else {
    Eigen::Matrix4d T_LtoM_predict = odom_data_.back().pose * pose_predict;
    registration(cur_scan, T_LtoM_predict, odom_cur.pose, scan_in_target);
  }
  odom_data_.push_back(odom_cur);

  if (update_map) {
    updateKeyScan(cur_scan, odom_cur);
  }
}

/**
 * @brief NDT点云匹配接口函数，将cur_scan向scan_in_target对齐
 * 
 * @param cur_scan 当前帧点云(输入点云)
 * @param pose_predict 当前帧点云的预测位姿(相对于scan_in_target点云)
 * @param pose_out 点云的位姿结果(输入点云相对于scan_in_targer点云)
 * @param scan_in_target 
 */
void LiDAROdometry::registration(const VPointCloud::Ptr& cur_scan,
                                 const Eigen::Matrix4d& pose_predict,
                                 Eigen::Matrix4d& pose_out,
                                 VPointCloud::Ptr scan_in_target) {
  VPointCloud::Ptr p_filtered_cloud(new VPointCloud());
  downsampleCloud(cur_scan, p_filtered_cloud, 0.5);
  
  // 设定Source点云(the point cloud that we want to align to the target)
  ndt_omp_->setInputSource(p_filtered_cloud);
  // 将Source点云向Target点云对齐
  ndt_omp_->align(*scan_in_target, pose_predict.cast<float>());
  // 获取相对位姿
  pose_out = ndt_omp_->getFinalTransformation().cast<double>();
}

/**
 * @brief 更新NDT配准时的Target点云
 * 
 * @param cur_scan 当前帧点云
 * @param odom_data 当前帧相对于初始帧的位姿
 */
void LiDAROdometry::updateKeyScan(const VPointCloud::Ptr& cur_scan,
                                  const OdomData& odom_data) {
  if (checkKeyScan(odom_data)) {

    VPointCloud::Ptr filtered_cloud(new VPointCloud());
    downsampleCloud(cur_scan, filtered_cloud, 0.1);

    VPointCloud::Ptr scan_in_target(new VPointCloud());
    pcl::transformPointCloud(*filtered_cloud, *scan_in_target, odom_data.pose);

    *map_cloud_ += *scan_in_target;
    ndt_omp_->setInputTarget(map_cloud_);
    key_frame_index_.push_back(odom_data_.size());
  }
}

/**
 * @brief 检查当前帧是否是关键帧(只根据当前帧的位姿确定)
 * 
 * @param odom_data 当前帧的位姿
 * @return true 是关键帧
 * @return false 不是关键帧
 */
bool LiDAROdometry::checkKeyScan(const OdomData& odom_data) {
  static Eigen::Vector3d position_last(0,0,0);
  static Eigen::Vector3d ypr_last(0,0,0);

  Eigen::Vector3d position_now = odom_data.pose.block<3,1>(0,3);
  // 连续两帧之间平移的二范数
  double dist = (position_now - position_last).norm();

  const Eigen::Matrix3d rotation (odom_data.pose.block<3,3> (0,0));
  Eigen::Vector3d ypr = mathutils::R2ypr(rotation);
  Eigen::Vector3d delta_angle = ypr - ypr_last;
  for (size_t i = 0; i < 3; i++)
    delta_angle(i) = normalize_angle(delta_angle(i));
  // Eigen内置了一些对矩阵元素操作的函数，均以cwise开头
  delta_angle = delta_angle.cwiseAbs();

  if (key_frame_index_.size() == 0 || dist > 0.2
     || delta_angle(0) > 5.0 || delta_angle(1) > 5.0 || delta_angle(2) > 5.0) {
    position_last = position_now;
    ypr_last = ypr;
    return true;
  }
  return false;
}

void LiDAROdometry::setTargetMap(VPointCloud::Ptr map_cloud_in) {
  map_cloud_->clear();
  pcl::copyPointCloud(*map_cloud_in, *map_cloud_);
  ndt_omp_->setInputTarget(map_cloud_);
}

void LiDAROdometry::clearOdomData() {
  key_frame_index_.clear();
  odom_data_.clear();
}


}


