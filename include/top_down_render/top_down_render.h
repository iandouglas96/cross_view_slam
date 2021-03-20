#ifndef TOP_DOWN_RENDER_H_
#define TOP_DOWN_RENDER_H_

#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <chrono>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "top_down_render/point_xyz_class_normal.h"
#include "top_down_render/top_down_map_polar.h"
#include "top_down_render/particle_filter.h"
#include "top_down_render/scan_renderer_polar.h"

class TopDownRender {
  public:
    TopDownRender(ros::NodeHandle &nh);
    void initialize();
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport *it_;
    tf2_ros::TransformBroadcaster *tf2_broadcaster_;
    ros::Subscriber pc_sub_;
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>> *pc_sync_sub_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *motion_prior_sync_sub_;
    typedef message_filters::sync_policies::ApproximateTime<pcl::PointCloud<pcl::PointXYZRGB>, geometry_msgs::PoseStamped> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> *sync_sub_;

    ros::Subscriber gt_pose_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher scale_pub_;
    image_transport::Publisher map_pub_;
    image_transport::Publisher scan_pub_;

    Eigen::Affine2f gt_pose_;
    Eigen::Affine3f last_prior_pose_;
    cv::Point map_center_;
    cv::Mat color_lut_;
    cv::Mat background_img_;
    TopDownMapPolar *map_;
    ParticleFilter *filter_;
    ScanRendererPolar *renderer_;

    float current_res_ = 4; //m/px range
    bool is_converged_ = false;

    void publishSemanticTopDown(std::vector<Eigen::ArrayXXf> &top_down, std_msgs::Header &header);
    void visualize(std::vector<Eigen::ArrayXXf> &classes, cv::Mat &img);
    cv::Mat visualizeAnalog(Eigen::ArrayXXf &cls, float scale);
    void updateFilter(std::vector<Eigen::ArrayXXf> &top_down, float res, 
                      Eigen::Affine3f &motion_prior, std_msgs::Header &header);
    void pcCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&, 
                    const geometry_msgs::PoseStamped::ConstPtr &motion_prior);
    void gtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
};

#endif //TOP_DOWN_RENDER_H_
