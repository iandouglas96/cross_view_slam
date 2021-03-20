#ifndef SCAN_RENDERER_H_
#define SCAN_RENDERER_H_

#include <Eigen/Dense>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

class ScanRenderer {
  public:
    ScanRenderer();
    void renderSemanticTopDown(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, 
                               float res, std::vector<Eigen::ArrayXXf> &imgs);
  protected:
    Eigen::VectorXi flatten_lut_;
};

#endif //SCAN_RENDERER_H_
