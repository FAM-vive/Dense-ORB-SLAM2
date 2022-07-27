/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "PointCloude.h"
#include "KeyFrame.h"

#include <vector>
#include <memory>
#include <mutex>
#include <thread>

#include <condition_variable>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace ORB_SLAM2 {
class PointCloudMapping {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef pcl::PointXYZRGBA PointT;
  typedef pcl::PointCloud<PointT> PointCloud;

  PointCloudMapping(double resolution_, double meank_, double thresh_);
  /**
   * @brief 保存点云文件
   */
  void Save();

  /**
   * @brief 将关键帧生成点云并插入到全局点云中
   *
   * @param kf            关键帧
   * @param color         关键帧对应的彩色图
   * @param depth         关键帧对应的深度图
   * @param idk           关键帧索引，不管是否增加关键帧，都计数+1
   */
  void InsertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth, int idk);

  /**
   * @brief 关闭线程
   */
  void Shutdown();

  /**
   * @brief 点云显示
   */
  void Viewer();

  /**
   * @brief 闭环之后更新点云信息
   * @param all_kfs 闭环调整之后的所有关键帧
   */
  void UpdateCloud(vector<KeyFrame*> all_kfs);

  bool cloudbusy;
  bool loopbusy;
  int loopcount = 0;
  mutex cloudBusyMutex;
  mutex loopBusyMutex;

 protected:
  /**
   * @brief 用关键帧生成带颜色信息的点云
   * */
  PointCloud::Ptr GeneratePointCloud(KeyFrame *kf, cv::Mat &color,
                                     cv::Mat &depth);
  shared_ptr<thread> viewerThread;

  mutex globalMapMutex;
  PointCloud::Ptr globalMap;  // 全局点云，由pointcloud得到

  bool shutDownFlag = false;
  mutex shutDownMutex;

  // 关键帧是否更新点云信息
  condition_variable keyFrameUpdated;
  mutex keyFrameUpdateMutex;

  // data to generate point clouds
  vector<KeyFrame *> keyframes;
  mutex keyframeMutex;
  uint16_t lastKeyframeSize = 0;

  // 存储所有关键帧的点云、位姿、id信息，闭环中用到
  mutex pointcloudMutex;
  vector<PointCloude, Eigen::aligned_allocator<PointCloude>> pointcloud;

  // 默认参数，可以从yaml文件输入修改
  // 体素滤波分辨率，0.04m*0.04m*0.04m
  double resolution = 0.04;
  // 每个点分析的临近点的个数设置为50
  double meank = 50;
  // 离群点阈值
  double thresh = 1;
  pcl::VoxelGrid<PointT> voxel;
  // 统计滤波器
  pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
};
}  // namespace ORB_SLAM2

#endif  // POINTCLOUDMAPPING_H
