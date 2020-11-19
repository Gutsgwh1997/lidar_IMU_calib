#ifndef RS_COMMON_HPP
#define RS_COMMON_HPP

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <vector>

#include <utils/pcl_utils.h>

namespace licalib {

class RobosenseConverter {
   public:
    typedef std::shared_ptr<RobosenseConverter> Ptr;

    enum ModelType {
        RS_LIDAR,
    };

    RobosenseConverter(ModelType modelType = RS_LIDAR) : m_modelType_(modelType) {}

    /**
     * @brief 点云格式转换函数，此函数会改变lidarMsg与outPointCloud
     *
     * @param lidarMsg ROS类型的点云消息（原本时间戳在是点云中最后一个点，现在改为点云中第一个点的了）
     * @param outPointCloud PointXYZIT类型的点云
     */
    void unpack_scan(sensor_msgs::PointCloud2::Ptr &lidarMsg, TPointCloud &outPointCloud) const {
        pcl::PointCloud<licalib::PointXYZIRT> cloud;
        pcl::fromROSMsg(*lidarMsg, cloud);

        outPointCloud.clear();
        outPointCloud.height = cloud.height;
        outPointCloud.width = cloud.width;
        outPointCloud.is_dense = false;
        outPointCloud.resize(outPointCloud.height * outPointCloud.width);

        double timebase = lidarMsg->header.stamp.toSec();
        float last_point_time = cloud.at(cloud.width -1, cloud.height -1).timestamp;
        for (int h = 0; h < cloud.height; h++) {
            for (int w = 0; w < cloud.width; w++) {
                TPoint point;
                point.x = cloud.at(w, h).x;
                point.y = cloud.at(w, h).y;
                point.z = cloud.at(w, h).z;
                point.intensity = cloud.at(w, h).intensity;
                float cur_point_time = cloud.at(w, h).timestamp;
                point.timestamp = timebase + (cur_point_time - last_point_time) * 1e-9;
                if(h == 0 && w == 0){
                    lidarMsg->header.stamp = ros::Time().fromSec(point.timestamp);
                    outPointCloud.header = pcl_conversions::toPCL(lidarMsg->header);
                }
                outPointCloud.at(w, h) = point;
            }
        }
    }

   private:
    ModelType m_modelType_;
};
}
#endif
