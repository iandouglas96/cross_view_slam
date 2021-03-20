#include "top_down_render/top_down_render.h"

TopDownRender::TopDownRender(ros::NodeHandle &nh) {
  nh_ = nh;
}

void TopDownRender::initialize() {
  last_prior_pose_ = Eigen::Affine3f::Identity();
  bool use_motion_prior;
  nh_.param<bool>("use_motion_prior", use_motion_prior, false);
  if (use_motion_prior) {
    pc_sync_sub_ = new message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>>(nh_, "pc", 50);
    motion_prior_sync_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "motion_prior", 50);
    sync_sub_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(50),
                      *pc_sync_sub_, *motion_prior_sync_sub_);
    sync_sub_->registerCallback(&TopDownRender::pcCallback, this);
  } else {
    pc_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("pc", 10, 
                std::bind(&TopDownRender::pcCallback, this, std::placeholders::_1, 
                          geometry_msgs::PoseStamped::ConstPtr()));
  }

  gt_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("gt_pose", 10, &TopDownRender::gtPoseCallback, this);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_est", 1);
  scale_pub_ = nh_.advertise<std_msgs::Float32>("scale", 1);
  it_ = new image_transport::ImageTransport(nh_);
  map_pub_ = it_->advertise("map", 1);
  scan_pub_ = it_->advertise("scan", 1);

  //This order determines priority as well
  color_lut_ = cv::Mat::ones(256, 1, CV_8UC3)*255;
  color_lut_.at<cv::Vec3b>(0) = cv::Vec3b(255,255,255); //unlabeled
  color_lut_.at<cv::Vec3b>(1) = cv::Vec3b(0,100,0);     //terrain
  color_lut_.at<cv::Vec3b>(2) = cv::Vec3b(255,0,0);     //road
  color_lut_.at<cv::Vec3b>(3) = cv::Vec3b(255,0,255);   //dirt
  color_lut_.at<cv::Vec3b>(4) = cv::Vec3b(0,0,255);     //building
  color_lut_.at<cv::Vec3b>(5) = cv::Vec3b(0,255,0);     //veg
  color_lut_.at<cv::Vec3b>(6) = cv::Vec3b(255,255,0);   //car

  std::string map_path;
  nh_.getParam("map_path", map_path);
  background_img_ = cv::imread(map_path+".png", cv::IMREAD_COLOR);

  float svg_res = -1;
  float raster_res = 1;
  nh_.getParam("raster_res", raster_res);
  bool estimate_scale = true;
  if (nh_.getParam("svg_res", svg_res)) {
    estimate_scale = false; 
  }

  int svg_origin_x, svg_origin_y;
  nh_.param<int>("svg_origin_x", svg_origin_x, 0);
  nh_.param<int>("svg_origin_y", svg_origin_y, 0);
  map_center_ = cv::Point(svg_origin_x, background_img_.size().height-svg_origin_y);

  //Get filter parameters
  FilterParams filter_params;
  int num_particles;
  nh_.param<int>("num_particles", num_particles, 20000);
  nh_.param<float>("filter_pos_cov", filter_params.pos_cov, 0.3);
  nh_.param<float>("filter_theta_cov", filter_params.theta_cov, M_PI/100);
  nh_.param<float>("filter_regularization", filter_params.regularization, 0.15);
  if (!estimate_scale) {
    filter_params.fixed_scale = svg_res;
  } else {
    filter_params.fixed_scale = -1;
  }

  bool use_raster;
  nh_.param<bool>("use_raster", use_raster, false);

  //Publish the map image
  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", background_img_).toImageMsg();
  map_pub_.publish(img_msg);

  if (use_raster) {
    map_ = new TopDownMapPolar(map_path, color_lut_, 6, 6, raster_res);
  } else {
    map_ = new TopDownMapPolar(map_path+".svg", color_lut_, 6, 6, raster_res);
  }
  map_->samplePtsPolar(Eigen::Vector2i(100, 25), 2*M_PI/100);
  filter_ = new ParticleFilter(num_particles, map_, filter_params);
  renderer_ = new ScanRendererPolar();

  //static transform broadcaster for map viz
  tf2_broadcaster_ = new tf2_ros::TransformBroadcaster(); 

  ROS_INFO_STREAM("Setup complete");

}

void TopDownRender::publishSemanticTopDown(std::vector<Eigen::ArrayXXf> &top_down, std_msgs::Header &header) {
  cv::Mat map_color;
  visualize(top_down, map_color);

  //Convert to ROS and publish
  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", map_color).toImageMsg();
  img_msg->header = header;
  scan_pub_.publish(img_msg);
}

cv::Mat TopDownRender::visualizeAnalog(Eigen::ArrayXXf &cls, float scale) {
  cv::Mat map_mat(cls.cols(), cls.rows(), CV_32FC1, (void*)cls.data());
  cv::Mat map_char, map_color;
  map_mat.convertTo(map_char, CV_8UC1, 255./scale);
  cv::cvtColor(map_char, map_color, cv::COLOR_GRAY2BGR);

  return map_color;
}

void TopDownRender::visualize(std::vector<Eigen::ArrayXXf> &classes, cv::Mat &img) {
  cv::Mat map_mat(classes[0].cols(), classes[0].rows(), CV_8UC1, cv::Scalar(0));

  for (int idx=0; idx<map_mat.size().width; idx++) {
    for (int idy=0; idy<map_mat.size().height; idy++) {
      char best_cls = 0;
      float best_cls_score = -std::numeric_limits<float>::infinity();
      float worst_cls_score = std::numeric_limits<float>::infinity();
      char cls_id = 1;
      for (auto cls : classes) {
        if (cls(idx, idy) >= best_cls_score) {
          best_cls_score = cls(idx, idy);
          best_cls = cls_id;
        }
        if (cls(idx, idy) < worst_cls_score) {
          worst_cls_score = cls(idx, idy);
        }
        cls_id++;
      }

      if (best_cls_score == worst_cls_score) {
        //All scores are the same, show white
        map_mat.at<uint8_t>(idy, idx) = 0;
      } else {
        map_mat.at<uint8_t>(idy, idx) = best_cls;
      }
    }
  }

  cv::Mat map_multichannel;
  cv::cvtColor(map_mat, map_multichannel, cv::COLOR_GRAY2BGR);
  cv::LUT(map_multichannel, color_lut_, img);
}

void TopDownRender::updateFilter(std::vector<Eigen::ArrayXXf> &top_down, float res,
                                 Eigen::Affine3f &motion_prior, std_msgs::Header &header) {
  auto start = std::chrono::high_resolution_clock::now();
  filter_->update(top_down, res);
  auto stop = std::chrono::high_resolution_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
  ROS_INFO_STREAM("Filter update " << dur.count() << " ms");

  //Project 3d prior to 2d
  Eigen::Vector2f motion_priort = motion_prior.translation().head<2>();
  Eigen::Vector3f proj_rot = motion_prior.rotation() * Eigen::Vector3f::UnitX();
  float motion_priora = std::atan2(proj_rot[1], proj_rot[0]);
  ROS_INFO_STREAM(motion_priort << ", " << motion_priora);
  filter_->propagate(motion_priort, motion_priora);

  cv::Mat background_copy = background_img_.clone();
  filter_->visualize(background_copy);

  //Draw gt pose
  Eigen::Vector2f front(2,0);
  front = gt_pose_.linear()*front;
  cv::Point img_rot(front[0], -front[1]);
  cv::Point img_pos(gt_pose_.translation()[0], -gt_pose_.translation()[1]);
  cv::arrowedLine(background_copy, map_center_+img_pos-img_rot, map_center_+img_pos+img_rot, 
                  cv::Scalar(0,255,0), 2, CV_AA, 0, 0.3);

  //Publish visualization
  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", background_copy).toImageMsg();
  img_msg->header = header;
  map_pub_.publish(img_msg);
}

void TopDownRender::pcCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                               const geometry_msgs::PoseStamped::ConstPtr& motion_prior) {
  ROS_INFO_STREAM("pc cb");
  auto start = std::chrono::high_resolution_clock::now();

  //Generate top down render and remap
  std::vector<Eigen::ArrayXXf> top_down;
  for (int i=0; i<map_->numClasses(); i++) {
    Eigen::ArrayXXf img(100, 25);
    top_down.push_back(img);
  }

  ROS_INFO_STREAM("Starting render");
  renderer_->renderSemanticTopDown(cloud, current_res_, 2*M_PI/100, top_down);

  //convert pointcloud header to ROS header
  std_msgs::Header img_header;
  publishSemanticTopDown(top_down, img_header);

  pcl_conversions::fromPCL(cloud->header, img_header);

  auto stop = std::chrono::high_resolution_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
  ROS_INFO_STREAM("Render took " << dur.count() << " ms");
  
  //Compute delta motion
  Eigen::Affine3d motion_prior_eig = Eigen::Affine3d::Identity();
  if (motion_prior) {
    tf::poseMsgToEigen(motion_prior->pose, motion_prior_eig);
  }
  Eigen::Affine3f delta_pose = last_prior_pose_.inverse() * motion_prior_eig.cast<float>();
  last_prior_pose_ = motion_prior_eig.cast<float>();

  updateFilter(top_down, current_res_, delta_pose, img_header);
  Eigen::Matrix4f cov;
  filter_->computeMeanCov(cov);

  if (std::max(cov(0,0), cov(1,1)) > 15 && current_res_ < 4) {
    //cov big, expand local region
    current_res_ += 0.05;
  } else if (current_res_ > 0.5) {
    //we gucci, shrink to refine
    current_res_ -= 0.02;
  }

  //get mean likelihood state
  Eigen::Vector4f ml_state;
  filter_->meanLikelihood(ml_state);

  ROS_INFO_STREAM("scale uncertainty: " << cov(3,3));
  if (cov(3,3) < 0.003*ml_state[3]) {
    //freeze scale
    ROS_INFO_STREAM("scale: " << ml_state[3]);
    filter_->freezeScale();
  }

  //Only publish if converged to unimodal dist
  float scale = filter_->scale();
  float scale_2 = scale*scale;
  if (cov(0,0)/scale_2 < 40 && cov(1,1)/scale_2 < 40 && cov(2,2) < 0.5 && filter_->scale() > 0) {
    is_converged_ = true;
  }

  if (is_converged_) {
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header = pcl_conversions::fromPCL(cloud->header);
    pose.header.frame_id = "world";

    std_msgs::Float32 scale_msg;
    scale_msg.data = scale;
    scale_pub_.publish(scale_msg);

    //pose
    pose.pose.pose.position.x = (ml_state[0] - map_center_.x)/scale;
    pose.pose.pose.position.y = (ml_state[1] - (background_img_.size().height-map_center_.y))/scale;
    pose.pose.pose.position.z = 2;
    pose.pose.pose.orientation.x = 0;
    pose.pose.pose.orientation.y = 0;
    pose.pose.pose.orientation.z = sin(ml_state[2]/2);
    pose.pose.pose.orientation.w = cos(ml_state[2]/2);

    //cov
    pose.pose.covariance[0] = cov(0,0)/scale_2;
    pose.pose.covariance[1] = cov(0,1)/scale_2;
    pose.pose.covariance[5] = cov(0,2)/scale;
    pose.pose.covariance[6] = cov(1,0)/scale_2;
    pose.pose.covariance[7] = cov(1,1)/scale_2;
    pose.pose.covariance[11] = cov(1,2)/scale;
    pose.pose.covariance[30] = cov(2,0)/scale;
    pose.pose.covariance[31] = cov(2,1)/scale;
    pose.pose.covariance[35] = cov(2,2);

    pose_pub_.publish(pose);

    geometry_msgs::TransformStamped map_svg_transform;
    map_svg_transform.header.stamp = pcl_conversions::fromPCL(cloud->header.stamp);
    map_svg_transform.header.frame_id = "world";
    map_svg_transform.child_frame_id = "sem_map";
    map_svg_transform.transform.translation.x = (background_img_.size().width/2-map_center_.x)/scale;
    map_svg_transform.transform.translation.y = -(background_img_.size().height/2-map_center_.y)/scale;
    map_svg_transform.transform.translation.z = -2;
    map_svg_transform.transform.rotation.x = 1; //identity rot
    tf2_broadcaster_->sendTransform(map_svg_transform);
  }
}

void TopDownRender::gtPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
  //Convert to Eigen
  Eigen::Affine3f gt_pose3 = Eigen::Affine3f::Identity();
  gt_pose3 *= Eigen::Quaternionf(pose->pose.orientation.w, 
                                 pose->pose.orientation.x, 
                                 pose->pose.orientation.y, 
                                 pose->pose.orientation.z);
  gt_pose3.translation() << pose->pose.position.x, pose->pose.position.y, pose->pose.position.z;

  //Now we have to project into 2D
  Eigen::Vector3f x_axis = gt_pose3.linear()*Eigen::Vector3f::UnitX();
  float theta = atan2(x_axis[1], x_axis[0]);

  gt_pose_.translation() = gt_pose3.translation().head(2);
  gt_pose_.linear() << cos(theta), -sin(theta),
                       sin(theta), cos(theta);
}
