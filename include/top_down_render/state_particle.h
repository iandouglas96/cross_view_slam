#ifndef STATE_PARTICLE_H_
#define STATE_PARTICLE_H_

#include "top_down_render/top_down_map_polar.h"

#include <random>
#include <chrono>

typedef struct State {
  float init_x_px;
  float init_y_px;
  float dx_m = 0;
  float dy_m = 0;
  float theta;
  float scale; //px/m
  bool have_init;
} State;

typedef struct FilterParams {
  float pos_cov;
  float theta_cov;
  float regularization;
  float fixed_scale = -1;
} FilterParams;

class StateParticle {
  public:
    StateParticle(std::mt19937 *gen, TopDownMapPolar *map, FilterParams &params);
    
    void propagate(Eigen::Vector2f &trans, float omega, bool scale_freeze=false);
    State state();
    Eigen::Vector4f mlState();
    void setState(State s);
    void computeWeight(std::vector<Eigen::ArrayXXf> &top_down_scan, float res);
    float weight();
    void setScale(float scale);
  private:
    //State
    State state_;
    float width_;
    float height_;
    float weight_;
    TopDownMapPolar *map_;
    std::mt19937 *gen_;

    FilterParams params_;
    std::vector<float> class_weights_;

    float getCostForRot(std::vector<Eigen::ArrayXXf> &top_down_scan,
                        std::vector<Eigen::ArrayXXf> &classes, float rot);
};

#endif //STATE_PARTICLE_H_
