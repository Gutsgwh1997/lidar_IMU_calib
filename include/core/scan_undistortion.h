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
#pragma once

#include <utils/dataset_reader.h>
#include <core/trajectory_manager.h>
#include <core/lidar_odometry.h>

namespace licalib {

class ScanUndistortion {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<ScanUndistortion> Ptr;

  explicit ScanUndistortion(TrajectoryManager::Ptr traj_manager,
                            std::shared_ptr<IO::LioDataset> dataset)
                            : traj_manager_(std::move(traj_manager)),
                              dataset_reader_(std::move(dataset)) {
  }

  /**
   * @brief 点云畸变补偿(去除所有dataset_reader_中点云的畸变)
   * 
   * @param correct_position 是否去除平移部分的畸变
   */
  void undistortScan(bool correct_position = false) {
    scan_data_.clear();

    // 带有时间戳的点云数据
    for (const TPointCloud& scan_raw: dataset_reader_->get_scan_data()) {
      Eigen::Quaterniond q_L0_to_G;
      Eigen::Vector3d p_L0_in_G;
      
      // 从B样条中获取lidar点云时刻的姿态
      double scan_timestamp = pcl_conversions::fromPCL(scan_raw.header.stamp).toSec();
      if (!traj_manager_->evaluateLidarPose(scan_timestamp, q_L0_to_G, p_L0_in_G)) {
        std::cout << "[ScanUndistortion] : pass " << scan_timestamp << std::endl;
        continue;
      }
      
      VPointCloud::Ptr scan_in_target(new VPointCloud);
      undistort(q_L0_to_G.conjugate(), p_L0_in_G, scan_raw,
                scan_in_target, correct_position);
      scan_data_.insert({scan_in_target->header.stamp, scan_in_target});
    }
  }

  void undistortScanInMap(bool correct_position = true) {
    scan_data_in_map_.clear();
    map_cloud_ = VPointCloud::Ptr(new VPointCloud);
    Eigen::Quaterniond q_L0_to_G;
    Eigen::Vector3d p_L0_in_G;
    double map_start_time = traj_manager_->get_map_time();
    traj_manager_->evaluateLidarPose(map_start_time, q_L0_to_G, p_L0_in_G);

    for (const TPointCloud& scan_raw: dataset_reader_->get_scan_data()) {
      VPointCloud::Ptr scan_in_target(new VPointCloud);
      undistort(q_L0_to_G.conjugate(), p_L0_in_G, scan_raw,
                scan_in_target, correct_position);
      scan_data_in_map_.insert({scan_in_target->header.stamp, scan_in_target});
      *map_cloud_ += *scan_in_target;
    }
  }

  void undistortScanInMap(const Eigen::aligned_vector<LiDAROdometry::OdomData>& odom_data) {
    scan_data_in_map_.clear();
    map_cloud_ = VPointCloud::Ptr(new VPointCloud);

    for (size_t idx = 0; idx < dataset_reader_->get_scan_data().size(); idx++){
      auto scan_raw = dataset_reader_->get_scan_data().at(idx);
      auto iter = scan_data_.find(scan_raw.header.stamp);
      if (iter == scan_data_.end()) {
        continue;
      }
      VPointCloud::Ptr scan_inMap = VPointCloud::Ptr(new VPointCloud);
      pcl::transformPointCloud(*(iter->second), *scan_inMap, odom_data.at(idx).pose);
      scan_data_in_map_.insert({scan_raw.header.stamp, scan_inMap});
      *map_cloud_ += *scan_inMap;
    }
  }

  const std::map<pcl::uint64_t, VPointCloud::Ptr> &get_scan_data() const {
    return scan_data_;
  }

  const std::map<pcl::uint64_t, VPointCloud::Ptr> &get_scan_data_in_map() const {
    return scan_data_in_map_;
  }

  const VPointCloud::Ptr& get_map_cloud() const {
    return map_cloud_;
  }

private:
  
  /**
   * @brief 真正实现点云畸变去除的函数，去除
   * 
   * @param q_G_to_target 初始时刻到lidar的旋转
   * @param p_target_in_G lidar相对于初始时刻的平移
   * @param scan_raw 原始点云(需要去除畸变的点云)，带有每个激光点的时间戳
   * @param scan_in_target 去除畸变后的点云(指针)，XYZI类型的数据
   * @param correct_position (是否去除平移项的畸变)
   */
  void undistort(const Eigen::Quaterniond& q_G_to_target,
                 const Eigen::Vector3d& p_target_in_G,
                 const TPointCloud& scan_raw,
                 const VPointCloud::Ptr& scan_in_target,
                 bool correct_position = false) const {
    scan_in_target->header = scan_raw.header;
    scan_in_target->height = scan_raw.height;
    scan_in_target->width  = scan_raw.width;
    scan_in_target->resize(scan_raw.height * scan_raw.width);
    scan_in_target->is_dense = false;

    VPoint NanPoint;
    NanPoint.x = NAN; NanPoint.y = NAN; NanPoint.z = NAN;
    for (int h = 0; h < scan_raw.height; h++) {
      for (int w = 0; w < scan_raw.width; w++) {
        VPoint vpoint;
        if (pcl_isnan(scan_raw.at(w,h).x)) {
          vpoint = NanPoint;
          scan_in_target->at(w,h) = vpoint;
          continue;
        }
        double point_timestamp = scan_raw.at(w,h).timestamp;
        Eigen::Quaterniond q_Lk_to_G;
        Eigen::Vector3d p_Lk_in_G;
        if (!traj_manager_->evaluateLidarPose(point_timestamp, q_Lk_to_G, p_Lk_in_G)) {
          continue;
        }
        // 当前激光点相对于点云时间戳处的旋转变换
        Eigen::Quaterniond q_LktoL0 = q_G_to_target * q_Lk_to_G;
        Eigen::Vector3d p_Lk(scan_raw.at(w,h).x, scan_raw.at(w,h).y, scan_raw.at(w,h).z);

        Eigen::Vector3d point_out;
        if (!correct_position) {         /// 只补偿旋转畸变
          point_out = q_LktoL0 * p_Lk;
        } else {                        /// 补偿旋转和平移的畸变
          point_out = q_LktoL0 * p_Lk + q_G_to_target * (p_Lk_in_G - p_target_in_G);
        }

        vpoint.x = point_out(0);
        vpoint.y = point_out(1);
        vpoint.z = point_out(2);
        vpoint.intensity = scan_raw.at(w,h).intensity;
        scan_in_target->at(w,h) = vpoint;
      }
    }
  }

  // 从构造函数中获取的 
  TrajectoryManager::Ptr traj_manager_;
  std::shared_ptr<IO::LioDataset> dataset_reader_;
  
  // 此类构造出来的结果
  std::map<pcl::uint64_t, VPointCloud::Ptr> scan_data_;
  std::map<pcl::uint64_t, VPointCloud::Ptr> scan_data_in_map_;
  VPointCloud::Ptr map_cloud_;
};

}