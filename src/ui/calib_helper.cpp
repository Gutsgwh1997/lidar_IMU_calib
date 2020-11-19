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
#include "utils/dataset_reader.h"
#include <cmath>
#include <ui/calib_helper.h>
#include <core/scan_undistortion.h>
#include <utils/tic_toc.h>

#include <boost/filesystem.hpp>
#include <memory>
#include <sstream>

namespace licalib {

// 系统运行最先调用的函数
CalibrHelper::CalibrHelper(ros::NodeHandle& nh)
        : calib_step_(Start),                        // 只是一个状态机，用来说明当期算法的状态
          iteration_step_(0),
          opt_time_offset_(false),
          plane_lambda_(0.6),                        // 论文中平面拟合的阈值（与二阶矩矩阵的特征值确定）
          ndt_resolution_(0.5),
          associated_radius_(0.05) {
  std::string topic_lidar;
  double bag_start, bag_durr;
  double scan4map;
  double knot_distance;
  double time_offset_padding;

  nh.param<std::string>("path_bag", bag_path_, "V1_01_easy.bag");
  nh.param<std::string>("topic_imu", topic_imu_, "/imu0");
  nh.param<std::string>("topic_lidar", topic_lidar, "/velodyne_packets");
  nh.param<double>("bag_start", bag_start, 0);
  nh.param<double>("bag_durr", bag_durr, -1);
  nh.param<double>("scan4map", scan4map, 15);
  nh.param<double>("ndtResolution", ndt_resolution_, 0.5);
  nh.param<double>("time_offset_padding", time_offset_padding, 0.015);
  nh.param<double>("knot_distance", knot_distance, 0.02);

  //* 1.创建结果保存的目录
  if (!createCacheFolder(bag_path_)) {
    calib_step_ = Error;
    std::cout << "Could not create cache folder!" << std::endl;
    exit(-1);
  }

  //* 2.read dataset and adjustDataset
  {
    std::string lidar_model;
    nh.param<std::string>("LidarModel", lidar_model, "VLP_16");
    IO::LidarModelType lidar_model_type = IO::LidarModelType::VLP_16;
    if (lidar_model == "VLP_16") {
        lidar_model_type = IO::LidarModelType::VLP_16;
    } else if (lidar_model == "RS_LIDAR") {
        lidar_model_type = IO::LidarModelType::RS_LIDAR;
    } else {
        calib_step_ = Error;
        ROS_WARN("LiDAR model %s not support yet.", lidar_model.c_str());
        exit(-1);
    }
    std::cout << "\nLoad dataset from " << bag_path_ << std::endl;
    IO::LioDataset lio_dataset_temp(lidar_model_type);
    lio_dataset_temp.read(bag_path_, topic_imu_, topic_lidar, bag_start, bag_durr);
    dataset_reader_ = lio_dataset_temp.get_data();
    dataset_reader_->adjustDataset();

    // DebugPointCloud();
  }

  //* 3.初始化一些基础功能类
  map_time_ = dataset_reader_->get_start_time();
  scan4map_time_ = map_time_ + scan4map;
  double end_time = dataset_reader_->get_end_time();

  traj_manager_ = std::make_shared<TrajectoryManager>(
          map_time_, end_time, knot_distance, time_offset_padding);

  scan_undistortion_ = std::make_shared<ScanUndistortion>(
          traj_manager_, dataset_reader_);

  lidar_odom_ = std::make_shared<LiDAROdometry>(ndt_resolution_);

  rotation_initializer_ = std::make_shared<InertialInitializer>();

  surfel_association_ = std::make_shared<SurfelAssociation>(
          associated_radius_, plane_lambda_);
}

/**
 * @brief 初始化部分，使用旋转项相同去得到外参中旋转项的初始值
 */
void CalibrHelper::Initialization() {
  if (Start != calib_step_) {
    ROS_WARN("[Initialization] Need status: Start.");
    return;
  }

  // 将IMU数据全部喂给B 样条曲线
  for (const auto& imu_data: dataset_reader_->get_imu_data()) {
    traj_manager_->feedIMUData(imu_data);
  }
  // 将IMU观测添加为约束，B样条插值旋转项的轨迹 
  traj_manager_->initialSO3TrajWithGyro();

  // lidar数据每次喂10帧，直到初始化成功
  for(const TPointCloud& raw_scan: dataset_reader_->get_scan_data()) {
    // 这个cloud是PointXYZI类型的
    VPointCloud::Ptr cloud(new VPointCloud);
    // 将PointXYZIT类型转换为PointXYZI类型
    TPointCloud2VPointCloud(raw_scan.makeShared(), cloud);

    double scan_timestamp = pcl_conversions::fromPCL(raw_scan.header.stamp).toSec();
    // feed数据的过程就会调用NDT配准，获得当前帧点云相对于初始帧的位姿(这一结果保存在lidar_odom_类中)
    lidar_odom_->feedScan(scan_timestamp, cloud);

    // 先喂30帧lidar数据，若rotation_initializer_初始化失败再喂10帧lidar进去
    if (lidar_odom_->get_odom_data().size() < 30 || (lidar_odom_->get_odom_data().size() % 10 != 0))
      continue;
    if (rotation_initializer_->EstimateRotation(traj_manager_, lidar_odom_->get_odom_data())) {
      // 初始化出来的外参旋转是Imu_2_Lidar
      Eigen::Quaterniond qItoLidar = rotation_initializer_->getQ_ItoS();
      // 将估计出来的初始外参写进配置文件类
      traj_manager_->getCalibParamManager()->set_q_LtoI(qItoLidar.conjugate());

      Eigen::Vector3d euler_ItoL = qItoLidar.toRotationMatrix().eulerAngles(0,1,2);
      std::cout << "[Initialization] Done. Euler_ItoL initial degree: "
                << (euler_ItoL*180.0/M_PI).transpose() << std::endl;
      calib_step_ = InitializationDone;
      break;
    }
  }
  if (calib_step_ != InitializationDone)
    ROS_WARN("[Initialization] fails.");
}

/**
 * @brief Lidar数据关联部分
 */
void CalibrHelper::DataAssociation() {
  std::cout << "[Data Association] start ...." << std::endl;
  TicToc timer;
  timer.tic();

  /// set surfel pap
  if (InitializationDone == calib_step_ ) {
    Mapping();
    scan_undistortion_->undistortScanInMap(lidar_odom_->get_odom_data());

    surfel_association_->setSurfelMap(lidar_odom_->getNDTPtr(), map_time_);
  } else if (BatchOptimizationDone == calib_step_ || RefineDone == calib_step_) {
    scan_undistortion_->undistortScanInMap();

    plane_lambda_ = 0.7;
    surfel_association_->setPlaneLambda(plane_lambda_);
    auto ndt_omp = LiDAROdometry::ndtInit(ndt_resolution_);
    ndt_omp->setInputTarget(scan_undistortion_->get_map_cloud());
    surfel_association_->setSurfelMap(ndt_omp, map_time_);
  } else {
      ROS_WARN("[DataAssociation] Please follow the step.");
      return;
  }

  /// get association
  for (auto const &scan_raw : dataset_reader_->get_scan_data()) {
    auto iter = scan_undistortion_->get_scan_data_in_map().find(
            scan_raw.header.stamp);
    if (iter == scan_undistortion_->get_scan_data_in_map().end()) {
      continue;
    }
    surfel_association_->getAssociation(iter->second, scan_raw.makeShared(), 2);
  }
  surfel_association_->averageTimeDownSmaple();
  std::cout << "Surfel point number: "
            << surfel_association_->get_surfel_points().size() << std::endl;
  std::cout<<GREEN<<"[Association] "<<timer.toc()<<" ms"<<RESET<<std::endl;

  if (surfel_association_->get_surfel_points().size() > 10){
    calib_step_ = DataAssociationDone;
  } else {
    ROS_WARN("[DataAssociation] fails.");
  }
}

void CalibrHelper::BatchOptimization() {
  if (DataAssociationDone != calib_step_) {
    ROS_WARN("[BatchOptimization] Need status: DataAssociationDone.");
    return;
  }
  std::cout << "\n================ Iteration " << iteration_step_ << " ==================\n";

  TicToc timer;
  timer.tic();
  traj_manager_->trajInitFromSurfel(surfel_association_, opt_time_offset_);

  calib_step_ = BatchOptimizationDone;
  saveCalibResult(cache_path_ + "/calib_result.csv");
  std::cout<<GREEN<<"[BatchOptimization] "<<timer.toc()<<" ms"<<RESET<<std::endl;
}

void CalibrHelper::Refinement() {
  if (BatchOptimizationDone > calib_step_) {
    ROS_WARN("[Refinement] Need status: BatchOptimizationDone.");
    return;
  }
  iteration_step_++;
  std::cout << "\n================ Iteration " << iteration_step_ << " ==================\n";

  DataAssociation();
  if (DataAssociationDone != calib_step_) {
    ROS_WARN("[Refinement] Need status: DataAssociationDone.");
    return;
  }
  TicToc timer;
  timer.tic();

  traj_manager_->trajInitFromSurfel(surfel_association_, opt_time_offset_);
  calib_step_ = RefineDone;
  saveCalibResult(cache_path_ + "/calib_result.csv");

  std::cout<<GREEN<<"[Refinement] "<<timer.toc()<<" ms"<<RESET<<std::endl;
}

void CalibrHelper::Mapping(bool relocalization) {
  bool update_map = true;
  // 未使用到重定位模式
  if (relocalization) {
    lidar_odom_->clearOdomData();
    update_map = false;
  } else {
    scan_undistortion_->undistortScan();
    lidar_odom_ = std::make_shared<LiDAROdometry>(ndt_resolution_);
  }

  double last_scan_t = 0;
  for (const auto& scan_raw: dataset_reader_->get_scan_data()) {
    double scan_t = pcl_conversions::fromPCL(scan_raw.header.stamp).toSec();
    if (scan_t > scan4map_time_)
      update_map = false;
    auto iter = scan_undistortion_->get_scan_data().find(scan_raw.header.stamp);
    if (iter != scan_undistortion_->get_scan_data().end()) {
      Eigen::Matrix4d pose_predict = Eigen::Matrix4d::Identity();
      Eigen::Quaterniond q_L2toL1 = Eigen::Quaterniond::Identity();
      if (last_scan_t > 0 &&
          traj_manager_->evaluateLidarRelativeRotation(last_scan_t, scan_t, q_L2toL1)) {
        pose_predict.block<3,3>(0,0) = q_L2toL1.toRotationMatrix();
      }
      lidar_odom_->feedScan(scan_t, iter->second, pose_predict, update_map);
      last_scan_t = scan_t;
    }
  }
}


void CalibrHelper::saveCalibResult(const std::string& calib_result_file) const {
  if (!boost::filesystem::exists(calib_result_file)) {
    std::ofstream outfile;
    outfile.open(calib_result_file, std::ios::app);
    outfile << "bag_path" << ","
            << "imu_topic" << "," << "map_time" << "," << "iteration_step" << ","
            << "p_IinL.x" << "," << "p_IinL.y" << "," << "p_IinL.z" << ","
            << "q_ItoL.x" << "," << "q_ItoL.y" << "," << "q_ItoL" << ","
            << "q_ItoL.w" << ","
            << "time_offset" << ","
            << "gravity.x" << "," << "gravity.y" << "," << "gravity.z" << ","
            << "gyro_bias.x" << "," << "gyro_bias.y" << "," <<"gyro_bias.z" << ","
            << "acce_bias.z" << "," << "acce_bias.y" << "," <<"acce_bias.z" << "\n";
    outfile.close();
  }

  std::stringstream ss;
  ss << bag_path_;
  ss << "," << topic_imu_;
  ss << "," << map_time_;
  ss << "," << iteration_step_;
  std::string info;
  ss >> info;

  traj_manager_->getCalibParamManager()->save_result(calib_result_file, info);
}

void CalibrHelper::saveMap() const {
  if (calib_step_ <= Start)
    return;
  std::string NDT_target_map_path = cache_path_ + "/NDT_target_map.pcd";
  lidar_odom_->saveTargetMap(NDT_target_map_path);

  std::string surfel_map_path = cache_path_ + "/surfel_map.pcd";
  surfel_association_->saveSurfelsMap(surfel_map_path);

  if (RefineDone == calib_step_) {
    std::string refined_map_path = cache_path_ + "/refined_map.pcd";
    std::cout << "Save refined map to " << refined_map_path << "; size: "
              << scan_undistortion_->get_map_cloud()->size() << std::endl;
    pcl::io::savePCDFileASCII(refined_map_path, *scan_undistortion_->get_map_cloud());
  }
}

/**
 * @brief 创建存放结果的文件路径
 *
 * @param bag_path bag文件的路径
 *
 * @return 在bag包所在的目录下创建一个与bag包同名的文件夹
 */
bool CalibrHelper::createCacheFolder(const std::string& bag_path) {
  boost::filesystem::path p(bag_path);
  if (p.extension() != ".bag") {
    return false;
  }
  cache_path_ = p.parent_path().string() + "/" + p.stem().string();
  boost::filesystem::create_directory(cache_path_);
  return true;
}

/**
 * @brief 测试点云存储分布的函数
 */
void CalibrHelper::DebugPointCloud() {
    const Eigen::aligned_vector<TPointCloud>& rawPointClouds = dataset_reader_->get_scan_data();
    if (rawPointClouds.size() == 0) {
        std::cout << "还没获取到数据!" << std::endl;
    } else {
        const TPointCloud& rawPointCloud = rawPointClouds[0];
        int width = rawPointCloud.width;
        int height = rawPointCloud.height;
        for (int w = 0; w < width; ++w) {
            for (int h = 0; h < height; ++h) {
                float x = rawPointCloud.at(w, h).x;
                float y = rawPointCloud.at(w, h).y;
                float z = rawPointCloud.at(w, h).z;
                float intensity = rawPointCloud.at(w, h).intensity;

                double time = rawPointCloud.at(w, h).timestamp;

                double xoy = std::sqrt(x * x + y * y);
                double angle = std::atan2(z, xoy) * 180 / M_PI;

                if (h == 0) {
                    std::cout << angle << " ";
                }
            }
        }
    }
    exit(-1);
}

} // namespace licalib
