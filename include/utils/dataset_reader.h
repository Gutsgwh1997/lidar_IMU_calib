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
#ifndef DATASET_READER_H
#define DATASET_READER_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <fstream>
#include <Eigen/Core>
#include <utils/eigen_utils.hpp>
#include <utils/vlp_common.h>
#include <utils/rs_common.h>

namespace licalib {
namespace IO {


struct IMUData {
  double timestamp;
  Eigen::Matrix<double, 3, 1> gyro;
  Eigen::Matrix<double, 3, 1> accel;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PoseData {
  double timestamp;
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


enum LidarModelType {
    VLP_16,
    VLP_16_SIMU,
    RS_LIDAR,
};

class LioDataset {
private:
  std::shared_ptr<LioDataset> data_;

  std::shared_ptr<rosbag::Bag> bag_;

  // 带有时间戳的IMU测量数据
  Eigen::aligned_vector<IMUData> imu_data_;

  // 带有每个激光点时间戳的点云
  Eigen::aligned_vector<TPointCloud> scan_data_;
  std::vector<double> scan_timestamps_;

  double start_time_;
  double end_time_;

  VelodyneCorrection::Ptr p_LidarConvert_;
  RobosenseConverter::Ptr p_RSLidarConvert_;

  LidarModelType lidar_model_;

public:
  LioDataset(LidarModelType lidar_model) : lidar_model_(lidar_model) {
  }

  /**
   * @brief 根据构造函数赋给的状态设定decoder_指针
   */
  void init() {
    switch (lidar_model_) {
      case LidarModelType::VLP_16:
        p_LidarConvert_ = VelodyneCorrection::Ptr(
                new VelodyneCorrection(VelodyneCorrection::ModelType::VLP_16));
        break;
      case LidarModelType::VLP_16_SIMU:
        p_LidarConvert_ = VelodyneCorrection::Ptr(
                new VelodyneCorrection(VelodyneCorrection::ModelType::VLP_16));
        break;
      // 添加对速腾激光雷达的支持
      case LidarModelType::RS_LIDAR:
        p_RSLidarConvert_ = RobosenseConverter::Ptr(
                new RobosenseConverter(RobosenseConverter::ModelType::RS_LIDAR));
        break;
      default:
        std::cout << "LiDAR model " << lidar_model_ << " not support yet." << std::endl;
    }
  }

  /**
   * @brief 读取数据的重要函数，已经适配速腾的激光雷达
   *
   * @param path rosbag数据包的位置
   * @param imu_topic imu的topic
   * @param lidar_topic lidar的topic
   * @param bag_start 自定义的rosbag起点时间偏移(s)
   * @param bag_durr 自定义的rosbag有效数据的时间长度(s)
   *
   * @return 
   */
  bool read(const std::string path, 
            const std::string imu_topic,
            const std::string lidar_topic,
            const double bag_start = -1.0, 
            const double bag_durr = -1.0) {
      data_.reset(new LioDataset(lidar_model_));
      data_->bag_.reset(new rosbag::Bag);
      data_->bag_->open(path, rosbag::bagmode::Read);

      // 定义对应格式的decoder-->即p_LidarConvert_
      init();

      // 获得imu_topic与lidar_topic指定时间段内的query
      rosbag::View view;
      {
          std::vector<std::string> topics;
          topics.push_back(imu_topic);
          topics.push_back(lidar_topic);

          rosbag::View view_full;
          view_full.addQuery(*data_->bag_);
          ros::Time time_init = view_full.getBeginTime();
          ros::Time time_end = view_full.getEndTime();
          time_init += ros::Duration(bag_start);
          ros::Time time_finish = (bag_durr < 0) ? time_end : time_init + ros::Duration(bag_durr);
          if(time_finish > time_end){
              std::cout << "bag duration time is too large, please try to reduce bag_durr or bag_start!" << std::endl;
              exit(-1);
          }
          view.addQuery(*data_->bag_, rosbag::TopicQuery(topics), time_init, time_finish);
      }

      for (rosbag::MessageInstance const m : view) {
          const std::string &topic = m.getTopic();

          if (lidar_topic == topic) {
              TPointCloud pointcloud;
              double timestamp = 0;

              if (m.getDataType() == std::string("velodyne_msgs/VelodyneScan")) {
                  velodyne_msgs::VelodyneScan::ConstPtr vlp_msg = m.instantiate<velodyne_msgs::VelodyneScan>();
                  timestamp = vlp_msg->header.stamp.toSec();
                  p_LidarConvert_->unpack_scan(vlp_msg, pointcloud);
              }
              if (m.getDataType() == std::string("sensor_msgs/PointCloud2")) {
                  if (lidar_model_ == LidarModelType::VLP_16) {
                      sensor_msgs::PointCloud2::ConstPtr scan_msg = m.instantiate<sensor_msgs::PointCloud2>();
                      timestamp = scan_msg->header.stamp.toSec();
                      p_LidarConvert_->unpack_scan(scan_msg, pointcloud);
                  } else if (lidar_model_ == LidarModelType::RS_LIDAR) {
                      sensor_msgs::PointCloud2::Ptr scan_msg = m.instantiate<sensor_msgs::PointCloud2>();
                      timestamp = scan_msg->header.stamp.toSec();
                      p_RSLidarConvert_->unpack_scan(scan_msg, pointcloud);
                      timestamp = pointcloud.at(0, 0).timestamp;
                  }
              }

              data_->scan_data_.emplace_back(pointcloud);
              data_->scan_timestamps_.emplace_back(timestamp);
          }

          if (imu_topic == topic) {
              sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
              double time = imu_msg->header.stamp.toSec();

              data_->imu_data_.emplace_back();
              data_->imu_data_.back().timestamp = time;
              data_->imu_data_.back().accel = Eigen::Vector3d(
                  imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
              data_->imu_data_.back().gyro = Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
                                                             imu_msg->angular_velocity.z);
          }
      }

      std::cout << lidar_topic << ": " << data_->scan_data_.size() << std::endl;
      std::cout << imu_topic << ": " << data_->imu_data_.size() << std::endl;
  }


  /// Select the overlap of the dataset
  ///     ----------------------- ...... -----------------------------> t
  ///     |       |        |                    |         |        |
  ///   scan_0  IMU_0    scan_1               scan_N-1  IMU_N    scan_N
  ///
  /// selected data [scan_0, scan_N-1],[IMU_0, IMU_N]
  /// time  [scan_0.t, scan_N.t)
  void adjustDataset() {
    assert(imu_data.size() > 0 && "No IMU data. Check your bag and imu topic");
    assert(scan_data.size() > 0 && "No scan data. Check your bag and lidar topic");

    // 保证数据有重合
    assert(scan_timestamps.front() < imu_data.back().timestamp
           && scan_timestamps.back() > imu_data.front().timestamp
           && "Unvalid dataset. Check your dataset.. ");

    if (scan_timestamps_.front() > imu_data_.front().timestamp) {
      start_time_ = scan_timestamps_.front();
      while (imu_data_.front().timestamp < start_time_)
        imu_data_.erase(imu_data_.begin());
    } else {
        std::cout << "--------------------GWH觉得不太对的地方，dataset_reader.h的219行--------------------------" << std::endl;
        std::cout << "start_time:" << start_time_ << std::endl;
        while ((*std::next(scan_timestamps_.begin())) < start_time_) {
            scan_data_.erase(scan_data_.begin());
            scan_timestamps_.erase(scan_timestamps_.begin());
        }
        start_time_ = scan_timestamps_.front();
    }

    end_time_ = std::min(scan_timestamps_.back(), imu_data_.back().timestamp);

    while (imu_data_.back().timestamp > end_time_)
      imu_data_.pop_back();

    while (scan_timestamps_.back() >= end_time_) {
      scan_data_.pop_back();
      scan_timestamps_.pop_back();
    }
    std::cout<<"after adjust --> imu size : "<< imu_data_.size() << std::endl;
    std::cout<<"after adjust --> lidar size : "<< scan_data_.size() << std::endl;
  }

  void reset() { data_.reset(); }

  std::shared_ptr<LioDataset> get_data() const {
    // return std::dynamic_pointer_cast<VioDataset>(data);
    return data_;
  }

  double get_start_time() const {
    return start_time_;
  }

  double get_end_time() const {
    return end_time_;
  }

  const std::vector<double> &get_scan_timestamps() const {
    return scan_timestamps_;
  }

  const Eigen::aligned_vector<IMUData> &get_imu_data() const {
    return imu_data_;
  }
  const Eigen::aligned_vector<TPointCloud> &get_scan_data() const {
    return scan_data_;
  }

};

}
}

#endif // DATASET_READER_H
