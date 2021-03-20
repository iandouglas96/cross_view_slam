#include "top_down_render/scan_renderer.h"

ScanRenderer::ScanRenderer() {
  flatten_lut_ = Eigen::VectorXi::Zero(256);
  flatten_lut_[100] = 2; //road
  flatten_lut_[101] = 3; //dirt
  flatten_lut_[102] = 1; //grass
  flatten_lut_[2] = 4;   //building
  flatten_lut_[3] = 4;   //wall

  flatten_lut_[7] = 5;   //vegetation
  flatten_lut_[8] = 1;   //terrain
  flatten_lut_[13] = 2;  //car
  flatten_lut_[14] = 2;  //truck
  flatten_lut_[15] = 2;  //bus
}

void ScanRenderer::renderSemanticTopDown(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, 
                                         float res, std::vector<Eigen::ArrayXXf> &imgs) {
  if (imgs.size() < 1) return;
  Eigen::Vector2i img_size(imgs[0].cols(), imgs[0].rows());

  for (int i=0; i<imgs.size(); i++) {
    imgs[i].setZero();
  }

  //Generate bins of points
  for (size_t idx=0; idx<cloud->height*cloud->width; idx++) {
    auto pt = cloud->points[idx];
    if (pt.x == 0 && pt.y == 0) continue;

    int x_ind = std::round(pt.x/res)+img_size[0]/2;
    int y_ind = std::round(pt.y/res)+img_size[1]/2;
    if (x_ind >= 0 && x_ind < img_size[0] && y_ind >= 0 && y_ind < img_size[1]) {
      int pt_class = *reinterpret_cast<const int*>(&cloud->points[idx].rgb) & 0xff;
      imgs[flatten_lut_[pt_class]-1](y_ind, x_ind)++;
    }
  }
}

