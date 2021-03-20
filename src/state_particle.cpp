#include "top_down_render/state_particle.h"

StateParticle::StateParticle(std::mt19937 *gen, TopDownMapPolar *map, FilterParams &params) {
  params_ = params;
  gen_ = gen;
  std::uniform_real_distribution<float> dist(0.,1.);

  std::vector<int> cls_vec;
  Eigen::Vector2f map_size = map->size().cast<float>() * map->resolution();

  if (params_.fixed_scale < 0) {
    state_.scale = std::pow(10, (dist(*gen)-0.5)*2);
  } else {
    state_.scale = params_.fixed_scale;
  }

  while (true) {
    state_.init_x_px = dist(*gen)*map_size[0];
    state_.init_y_px = dist(*gen)*map_size[1];
    map->getClassesAtPoint(Eigen::Vector2i(state_.init_x_px, state_.init_y_px), cls_vec);
    if (std::find(cls_vec.begin(), cls_vec.end(), 1) != cls_vec.end()) {
      break; //Particle is on the road
    }
  }

  state_.theta = 0;
  state_.have_init = false;

  map_ = map;
  width_ = map_size[0];
  height_ = map_size[1];
  weight_ = 0;

  class_weights_.push_back(0.75); //terrain
  class_weights_.push_back(2);    //road
  class_weights_.push_back(1.5);  //dirt road
  class_weights_.push_back(1.5);  //building
  class_weights_.push_back(0.5);  //trees
  class_weights_.push_back(0);
}

void StateParticle::propagate(Eigen::Vector2f &trans, float omega, bool scale_freeze) {
  Eigen::Vector2f trans_global = Eigen::Rotation2D<float>(state_.theta) * trans;
  state_.dx_m += trans_global[0];
  state_.dy_m += trans_global[1];

  //std::normal_distribution<float> disp_dist{0, 0.5};
  //std::normal_distribution<float> theta_dist{0, M_PI/30};
  std::normal_distribution<float> disp_dist{0, params_.pos_cov};
  std::normal_distribution<float> theta_dist{0, params_.theta_cov};

  state_.theta += theta_dist(*gen_) + omega;
  state_.dx_m += disp_dist(*gen_);
  state_.dy_m += disp_dist(*gen_);

  if (!scale_freeze) {
    float dist = std::sqrt(std::pow(state_.dx_m, 2) + std::pow(state_.dy_m, 2));
    std::normal_distribution<float> scale_dist{1, std::min(2./dist, 0.02)};
    state_.scale *= scale_dist(*gen_);
  }
}

void StateParticle::setState(State s) {
  state_.init_x_px = s.init_x_px;
  state_.init_y_px = s.init_y_px;
  state_.dx_m = s.dx_m;
  state_.dy_m = s.dy_m;
  state_.theta = s.theta;
  state_.scale = s.scale;
  state_.have_init = s.have_init;
}

void StateParticle::setScale(float scale) {
  state_.scale = scale;
}

State StateParticle::state() {
  return state_;
}

Eigen::Vector4f StateParticle::mlState() {
  return Eigen::Vector4f(state_.dx_m*state_.scale + state_.init_x_px, 
                         state_.dy_m*state_.scale + state_.init_y_px, 
                         state_.theta, state_.scale);
}

float StateParticle::weight() {
  return weight_;
}

float StateParticle::getCostForRot(std::vector<Eigen::ArrayXXf> &top_down_scan,
                                   std::vector<Eigen::ArrayXXf> &classes, float rot) {
  //number of bins to shift by
  int num_bins = top_down_scan[0].rows();
  int rot_shift = static_cast<int>(std::round(rot*num_bins/2/M_PI));

  //normalize rotation
  while (rot_shift >= num_bins) rot_shift -= num_bins;
  while (rot_shift < 0) rot_shift += num_bins;

  float cost = 0;
  float normalization = 0;
  for (int i=0; i<map_->numClasses(); i++) {
    //if (i == 4) continue; //ignore trees
    //if (i == 3) continue; //ignore buildings
    //semantic cost
    cost += (top_down_scan[i].topRows(rot_shift) * classes[i].bottomRows(rot_shift)).sum()*0.01*class_weights_[i];
    cost += (top_down_scan[i].bottomRows(num_bins-rot_shift) * classes[i].topRows(num_bins-rot_shift)).sum()*0.01*class_weights_[i];
    normalization += top_down_scan[i].sum();
  }

  if (isnan(cost/normalization)) {
    ROS_WARN("NAN");
    ROS_WARN_STREAM("cost: " << cost << ", norm: " << normalization << ", rot: " << rot);
    for (int i=0; i<map_->numClasses(); i++) {
      ROS_WARN_STREAM("sum: " << top_down_scan[i].sum());
      ROS_WARN_STREAM("weights: " << class_weights_[i]);
    }
  }
  return cost/normalization;
}

void StateParticle::computeWeight(std::vector<Eigen::ArrayXXf> &top_down_scan, float res) {
  auto start = std::chrono::high_resolution_clock::now();

  Eigen::Vector2f center(state_.dx_m*state_.scale + state_.init_x_px, 
                         state_.dy_m*state_.scale + state_.init_y_px);
  if (center[0] < 0 || center[1] < 0 || center[0] > width_ || center[1] > height_ ||
      state_.scale < std::pow(10, -0.1) || state_.scale > std::pow(10, 1)) {
    weight_ = 0;
    return;
  }

  std::vector<Eigen::ArrayXXf> classes;
  for (int i=0; i<map_->numClasses(); i++) {
    classes.push_back(Eigen::ArrayXXf(top_down_scan[0].rows(), top_down_scan[0].cols()));
  }

  map_->getLocalMap(center, state_.scale, res, classes);
  
  auto mid = std::chrono::high_resolution_clock::now();

  float best_cost = 1e10;
  float best_theta = 0;
  if (!state_.have_init) {
    //initialize
    for (float t=0; t<2*M_PI; t+=2*M_PI/40) {
      float cost = getCostForRot(top_down_scan, classes, t);
      if (best_cost > cost) {
        best_cost = cost; 
        best_theta = t;
      } 
    }
    state_.theta = best_theta;
    state_.have_init = true;
  } else {
    best_cost = getCostForRot(top_down_scan, classes, state_.theta);
  }

  weight_ = 1./(best_cost + params_.regularization);

  auto end = std::chrono::high_resolution_clock::now();

  //ROS_INFO_STREAM("overall: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "us");
  //ROS_INFO_STREAM("render: " << std::chrono::duration_cast<std::chrono::microseconds>(mid - start).count() << "us");
  //ROS_INFO_STREAM("cost: " << std::chrono::duration_cast<std::chrono::microseconds>(end - mid).count() << "us");
}
