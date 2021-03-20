#ifndef TOP_DOWN_MAP_POLAR_H_
#define TOP_DOWN_MAP_POLAR_H_

#include "top_down_render/top_down_map.h"

class TopDownMapPolar : public TopDownMap {
  public:
    TopDownMapPolar(std::string path, cv::Mat& color_lut, int num_classes, int num_ex, float res);

    //Overriden versions for polar map gen
    void getLocalMap(Eigen::Vector2f center, float scale, float res, std::vector<Eigen::ArrayXXf> &dists);
    void getLocalMap(Eigen::Vector2f center, float res, std::vector<Eigen::ArrayXXf> &dists);
    void samplePtsPolar(Eigen::Vector2i shape, float ang_res);

  protected:
    Eigen::Array2Xf ang_sample_pts_;
};

#endif //TOP_DOWN_MAP_POLAR_H_
