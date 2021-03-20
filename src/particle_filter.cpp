#include "top_down_render/particle_filter.h"

ParticleFilter::ParticleFilter(int N, TopDownMapPolar *map, FilterParams &params) {
  std::random_device rd;
  gen_ = new std::mt19937(rd());

  num_gaussians_ = 1;
  map_ = map;
  params_ = params;
  max_num_particles_ = N;

  size_t num_at_scale = 1;
  if (params_.fixed_scale < 0) {
    num_at_scale = 10;
  } else {
    scale_frozen_ = true;
  }

  //Weights should be even
  ROS_INFO_STREAM("Initializing particles...");
  for (int i=0; i<N/num_at_scale; i++) {
    StateParticle proto_part(gen_, map, params_);
    for (float scale=0; scale<1; scale+=1./num_at_scale) {
      std::shared_ptr<StateParticle> particle = std::make_shared<StateParticle>(gen_, map, params_);
      if (params_.fixed_scale < 0) {
        particle->setState(proto_part.state());
        particle->setScale(std::pow(10., scale));
      }
      particles_.push_back(particle);

      //Allocate memory for new array too, then we can swap back and forth without allocating
      std::shared_ptr<StateParticle> new_particle = std::make_shared<StateParticle>(gen_, map, params_);
      new_particles_.push_back(new_particle);
    }
  }
  ROS_INFO_STREAM("Particles initialized");
  max_likelihood_particle_ = particles_[0];

  num_particles_ = particles_.size();
  weights_ = Eigen::Matrix<float, 1, Eigen::Dynamic>::Ones(num_particles_)/num_particles_;

  //Initialize Gaussians
  computeGMM();
  gmm_thread_ = new std::thread(std::bind(&ParticleFilter::gmmThread, this));
}

void ParticleFilter::propagate(Eigen::Vector2f &trans, float omega) {
  particle_lock_.lock();
  for (auto p : particles_) {
    p->propagate(trans, omega, scale_frozen_);
  }
  particle_lock_.unlock();
}

void ParticleFilter::update(std::vector<Eigen::ArrayXXf> &top_down_scan, float res) {
  particle_lock_.lock();
  //Recompute weights
  std::for_each(std::execution::par, particles_.begin(), particles_.end(), 
                std::bind(&StateParticle::computeWeight, std::placeholders::_1, top_down_scan, res));

  weights_ = Eigen::Matrix<float, 1, Eigen::Dynamic>::Ones(particles_.size())/particles_.size();
  for (int i; i<particles_.size(); i++) {
    weights_[i] = particles_[i]->weight();
  }
  ROS_INFO_STREAM("particle weights: " << weights_.sum());
  weights_ = weights_/weights_.sum(); //Renormalize

  //Find the maximum likelihood particle
  Eigen::VectorXf::Index max_weight;
  weights_.maxCoeff(&max_weight);
  max_likelihood_particle_ = particles_[max_weight];

  ROS_INFO_STREAM("particle reweighting complete");

  int last_num_particles = num_particles_;
  num_particles_ = 0;
  for (auto cov : covs_) {
    Eigen::Vector2cf eig = cov.block<2,2>(0,0).eigenvalues(); //complex vector
    num_particles_ += static_cast<int>(sqrt(eig[0].real())*sqrt(eig[1].real()))*5; //Approximation of area of cov ellipse
  }
  num_particles_ = std::min(std::max(num_particles_, 3*last_num_particles/4+10), max_num_particles_); //bounds
  ROS_INFO_STREAM(num_particles_ << " particles");

  //resize num_particles_
  if (num_particles_ < new_particles_.size()) {
    new_particles_.resize(num_particles_);
  } else {
    while (new_particles_.size() < num_particles_) {
      new_particles_.push_back(std::make_shared<StateParticle>(gen_, map_, params_));
    }
  }
  
  //Resample
  std::uniform_real_distribution<float> shift_dist(0.,1.);
  float shift = shift_dist(*gen_); //Add a random shift
  for (int i=0; i<num_particles_; i++) {
    float running_sum = 0;
    float sample = (static_cast<float>(i)+shift)/num_particles_;
    int j=0;
    for (; j<particles_.size(); j++) {
      running_sum += weights_[j];
      if (running_sum > sample || j == particles_.size()-1) {
        break; 
      }
    }
    new_particles_[i]->setState(particles_[j]->state());
  }
  particles_.swap(new_particles_);
  particle_lock_.unlock();
}

void ParticleFilter::meanLikelihood(Eigen::Vector4f &mean_state) {
  mean_state = Eigen::Vector4f::Zero();
  float cos_sum = 0;
  float sin_sum = 0;
  for (auto particle : particles_) {
    Eigen::Vector4f state = particle->mlState();
    mean_state += state;
    cos_sum += cos(state[2]);
    sin_sum += sin(state[2]);
  }
  mean_state /= particles_.size();
  mean_state[2] = atan2(sin_sum/particles_.size(), cos_sum/particles_.size());
}

void ParticleFilter::computeMeanCov(Eigen::Matrix4f &cov) {
  cov.setZero();
  Eigen::Vector4f mean_likelihood_state;
  meanLikelihood(mean_likelihood_state);

  for (auto particle : particles_) {
    Eigen::Vector4f state = particle->mlState() - mean_likelihood_state;
    while (state[2] > M_PI) state[2] -= 2*M_PI;
    while (state[2] < -M_PI) state[2] += 2*M_PI;
    cov += state*state.transpose();
  }
  cov /= particles_.size()-1;
}

void ParticleFilter::maxLikelihood(Eigen::Vector4f &state) {
  state = max_likelihood_particle_->mlState();
}

void ParticleFilter::computeCov(Eigen::Matrix4f &cov) {
  cov.setZero();
  Eigen::Vector4f max_likelihood_state = max_likelihood_particle_->mlState();
  for (auto particle : particles_) {
    Eigen::Vector4f state = particle->mlState() - max_likelihood_state;
    while (state[2] > M_PI) state[2] -= 2*M_PI;
    while (state[2] < -M_PI) state[2] += 2*M_PI;
    cov += state*state.transpose();
  }
  cov /= particles_.size()-1;
}

void ParticleFilter::getGMM(std::vector<Eigen::Vector3f> &means, std::vector<Eigen::Matrix3f> &covs) {
  gmm_lock_.lock();
  means = means_;
  covs = covs_;
  gmm_lock_.unlock();
}

void ParticleFilter::gmmThread() {
  while (true) {
    computeGMM();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void ParticleFilter::computeGMM() {
  //We do this recursively: GMM in euclidean space, and then GMM on each cluster in theta space
  cv::Ptr<cv::ml::EM> em = cv::ml::EM::create();
  em->setCovarianceMatrixType(cv::ml::EM::COV_MAT_GENERIC);

  //Build sample array
  particle_lock_.lock();
  num_gaussians_ = std::min(static_cast<int>(particles_.size()/20)+1, num_gaussians_);
  em->setClustersNumber(num_gaussians_);

  int num_samples = std::min(1000, static_cast<int>(particles_.size()));
  cv::Mat samples = cv::Mat(num_samples, 4, CV_64F);
  for (int i=0; i<num_samples; i+=1) {
    Eigen::Vector3f state = particles_[i*particles_.size()/num_samples]->mlState().head<3>();
    samples.at<double>(i, 0) = state[0];
    samples.at<double>(i, 1) = state[1];
    samples.at<double>(i, 2) = 50*cos(state[2]);
    samples.at<double>(i, 3) = 50*sin(state[2]);
  }
  particle_lock_.unlock();

  cv::Mat likelihoods, labels, cluster_means;
  em->trainEM(samples, likelihoods, labels);
  float likelihood_mean = cv::mean(likelihoods)[0];

  int dir = 0;
  //try increasing
  if (num_gaussians_*50 < num_particles_) {
    em->setClustersNumber(num_gaussians_ + 1);
    em->trainEM(samples, likelihoods, labels);
    if (likelihood_mean + 0.3 < cv::mean(likelihoods)[0]) {
      dir = 1;
    }
  }
  //try decr
  if (num_gaussians_ > 1) {
    em->setClustersNumber(num_gaussians_ - 1);
    em->trainEM(samples, likelihoods, labels);
    if (likelihood_mean - 0.3 < cv::mean(likelihoods)[0]) {
      dir = -1;
    }
  }

  num_gaussians_ += dir;
  em->setClustersNumber(num_gaussians_);
  em->trainEM(samples, likelihoods, labels);
  cluster_means = em->getMeans();
  std::vector<cv::Mat> cluster_covs;
  em->getCovs(cluster_covs);

  //Convert to Eigen by manually copying elements
  gmm_lock_.lock();
  means_.clear();
  covs_.clear();
  for (int i=0; i<cluster_covs.size(); i++) {
    means_.push_back(Eigen::Vector3f(cluster_means.at<double>(i,0), cluster_means.at<double>(i,1),
                     atan2(cluster_means.at<double>(i,3), cluster_means.at<double>(i,2))));
    Eigen::Matrix3f cov;
    cov << cluster_covs[i].at<double>(0,0), cluster_covs[i].at<double>(0,1), 0,
           cluster_covs[i].at<double>(1,0), cluster_covs[i].at<double>(1,1), 0,
           0, 0, 1;
    covs_.push_back(cov);
  }
  gmm_lock_.unlock();
}

void ParticleFilter::freezeScale() {
  if (!scale_frozen_) {
    float geo_mean = 1;
    for (const auto& p : particles_) {
      geo_mean *= std::pow(p->state().scale, 1./particles_.size());
    }

    for (auto& p : particles_) {
      p->setScale(geo_mean);
    }
    scale_frozen_ = true;

    ROS_INFO_STREAM("scale converged and locked to " << geo_mean);
  }
}

float ParticleFilter::scale() const {
  if (scale_frozen_) {
    return particles_[0]->state().scale;
  }
  return -1;
}

void ParticleFilter::visualize(cv::Mat &img) {
  //Particle dist
  for (auto p : particles_) {
    Eigen::Vector3f state = p->mlState().head<3>();
    cv::Point pt(state[0], 
                 img.size().height-state[1]);
    cv::Point dir(cos(state[2])*5, -sin(state[2])*5);
    cv::arrowedLine(img, pt-dir, pt+dir, cv::Scalar(0,0,255), 2, CV_AA, 0, 0.3);
  }
  //GMM
  gmm_lock_.lock();
  for (int i=0; i<means_.size(); i++) {
    Eigen::Matrix2f pos_cov = covs_[i].block<2,2>(0,0);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigensolver(pos_cov);
    if (eigensolver.info() != Eigen::Success) break;
    Eigen::Vector2f evals = eigensolver.eigenvalues();
    Eigen::Vector2f maj_axis = eigensolver.eigenvectors().col(0);
    if (evals[0] < 0 || evals[1] < 0) break; //We better be PSD

    cv::Size size(sqrt(evals[0]), sqrt(evals[1]));
    float angle = atan2(-maj_axis[1], maj_axis[0]);
    cv::Point center(means_[i][0], 
                     img.size().height-means_[i][1]);
    cv::ellipse(img, center, size*2, angle*180/M_PI, 0, 360, cv::Scalar(255,0,0), 2);

    cv::Point dir(cos(means_[i][2])*5, -sin(means_[i][2])*5);
    cv::arrowedLine(img, center-dir, center+dir, cv::Scalar(255,0,0), 2, CV_AA, 0, 0.3);
  }
  gmm_lock_.unlock();
  //Max likelihood
  Eigen::Vector3f ml_state = max_likelihood_particle_->mlState().head<3>();
  cv::Point pt(ml_state[0], 
               img.size().height-ml_state[1]);
  cv::Point dir(cos(ml_state[2])*5, -sin(ml_state[2])*5);
  cv::arrowedLine(img, pt-dir, pt+dir, cv::Scalar(255,0,0), 2, CV_AA, 0, 0.3);
}
