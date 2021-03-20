#include "top_down_render/top_down_map.h"

//For some reason the implementation is in a separate #ifdef
//and not within the include guards
#define NANOSVG_IMPLEMENTATION
#include "top_down_render/nanosvg.h"

TopDownMap::TopDownMap(std::string path, cv::Mat& color_lut, int num_classes, int num_ex, float res) {
  resolution_ = res;
  num_classes_ = num_classes;
  num_exclusive_classes_ = num_ex;

  if (loadCacheMetaData(path)) {
    loadCachedMaps();
    ROS_INFO_STREAM("Loaded cached maps");
  } else {
    if (path.substr(path.size()-4) == ".svg") { 
      //parse svg
      NSVGimage* map;

      ROS_INFO_STREAM("No cache found, Loading vector map " << path);
      map = nsvgParseFromFile(path.c_str(), "px", 96);

      if (map == NULL) {
        ROS_ERROR("Map loading failed");
        return;
      }

      for (int cls=1; cls<=num_classes; cls++) {
        std::vector<std::vector<Eigen::Vector2f>> class_poly;
        cv::Vec3b color = color_lut.at<cv::Vec3b>(cls);
        int color_compressed = color[0] << 16 | color[1] << 8 | color[2];

        //iterate through shapes
        for (NSVGshape *shape = map->shapes; shape != NULL; shape = shape->next) {
          if ((shape->fill.color & 0xFFFFFF) == color_compressed) {

            //iterate through paths (assume 1 path per shape, really)
            for (NSVGpath *path = shape->paths; path != NULL; path = path->next) {
              std::vector<Eigen::Vector2f> eig_path;
              ROS_DEBUG_STREAM("path");

              for (int i=0; i<path->npts-1; i+=3) {
                float* p = &path->pts[i*2];
                eig_path.push_back(Eigen::Vector2f(p[0], map->height-p[1]));
                ROS_DEBUG_STREAM(p[0] << ", " << map->height-p[1]);
              }
              class_poly.push_back(eig_path);
            }
          }
        }
        poly_.push_back(class_poly);
      }

      ROS_INFO("Map loaded.");
      ROS_INFO_STREAM("Size: " << map->width << " x " << map->height);

      //Generate full rasterized map
      ROS_INFO_STREAM("Rasterizing map...");
      for (int i=0; i<num_classes; i++) {
        Eigen::ArrayXXf class_map(static_cast<int>(map->height/resolution_), 
                                  static_cast<int>(map->width/resolution_)); //0 inside obstacles, 1 elsewhere
        class_maps_.push_back(class_map);
      }

      ROS_INFO_STREAM("Rasterized map size: " << class_maps_[0].cols() << " x " << class_maps_[0].rows());
      getRasterMap(Eigen::Vector2f(map->width/2, map->height/2), 0, resolution_, class_maps_);

      saveRasterizedMaps(path.substr(0, path.size()-4));
    } else {
      ROS_INFO_STREAM("No cache found, loading raster map");
      loadRasterizedMaps(path);
    }

    //Do this after so we can reuse maps
    computeDists(class_maps_);
    ROS_INFO_STREAM("Rasterization complete");

    saveCachedMaps(path);
  }
}

void TopDownMap::getClassesAtPoint(const Eigen::Vector2i &center_ind, std::vector<int> &classes) {
  Eigen::Vector2i center = (center_ind.cast<float>() / resolution_).cast<int>();
  classes.clear();
  for (int cls=0; cls<num_classes_; cls++) {
    if (center[0] < class_maps_[cls].cols() && center[1] < class_maps_[cls].rows() &&
        center[0] >= 0 && center[1] >= 0) {
      if (class_maps_[cls](center[1], center[0]) < 1) {
        classes.push_back(cls);
      }
    }
  }
}

void TopDownMap::getClassesAtPoint(const Eigen::Vector2f &center, std::vector<int> &classes) {
  Eigen::Vector2i center_ind = (center/resolution_).cast<int>();
  getClassesAtPoint(center_ind, classes);
}

int TopDownMap::numClasses() const {
  return num_classes_;
}

Eigen::Vector2i TopDownMap::size() const {
  return Eigen::Vector2i(class_maps_[0].cols(), class_maps_[0].rows());
}

float TopDownMap::resolution() const {
  return resolution_;
}

void TopDownMap::saveRasterizedMaps(const std::string &path) {
  mkdir(path.c_str(), S_IRWXU);

  size_t ind = 0;
  cv::Mat cv_map, cv_map_scaled;
  for (const auto &map : class_maps_) {
    //eigen2cv doesn't like arrays
    Eigen::MatrixXf mat_map = map.array();
    cv::eigen2cv(mat_map, cv_map);
    cv_map.convertTo(cv_map_scaled, CV_8UC1, 255);
    cv::imwrite(path+"/class"+std::to_string(ind++)+".png", cv_map_scaled);
  }
}

void TopDownMap::loadRasterizedMaps(const std::string &path) {
  cv::Mat cv_map, cv_map_float;
  for (size_t i=0; i<num_classes_; i++) {
    cv_map = cv::imread(path+"/class"+std::to_string(i)+".png", cv::IMREAD_GRAYSCALE);
    cv_map.convertTo(cv_map_float, CV_32FC1, 1./255);
    Eigen::MatrixXf mat_map;
    cv::cv2eigen(cv_map_float, mat_map);
    class_maps_.push_back(mat_map.array());
  }
}

bool TopDownMap::loadCacheMetaData(const std::string &path) {
  std::ifstream data_file(std::string(getenv("HOME")) + "/.ros/cached_data.txt");
  if (!data_file) {
    return false;
  } 

  ROS_INFO_STREAM("Found cache, checking if parameters have changed");
  
  //Check that metadata agrees
  std::string line;
  std::getline(data_file, line);
  if (line != path) return false;
  std::getline(data_file, line);
  if (std::stoi(line) != num_classes_) return false;
  std::getline(data_file, line);
  if (std::stoi(line) != num_exclusive_classes_) return false;
  std::getline(data_file, line);
  if (std::abs(std::stof(line) - resolution_) > 0.01) return false;

  return true;
}

void TopDownMap::loadCachedMaps() {
  for (int cls=0; cls<num_classes_; cls++) {
    Eigen::ArrayXXf class_map;
    std::string name = std::string(getenv("HOME")) + "/.ros/class_map" + std::to_string(cls) + ".eig";
    read_binary(name, class_map);
    class_maps_.push_back(class_map);
  }
}

void TopDownMap::saveCachedMaps(const std::string &path) {
  std::ofstream data_file(std::string(getenv("HOME")) + "/.ros/cached_data.txt", 
                          std::ofstream::out | std::ofstream::trunc);
  data_file << path << std::endl;
  data_file << num_classes_ << std::endl;
  data_file << num_exclusive_classes_ << std::endl;
  data_file << resolution_ << std::endl;

  for (int cls=0; cls<num_classes_; cls++) {
    std::string name = std::string(getenv("HOME")) + "/.ros/class_map" + std::to_string(cls) + ".eig";
    write_binary(name, class_maps_[cls]);
  }
}

//For each cell, compute the distance to other cells
void TopDownMap::computeDists(std::vector<Eigen::ArrayXXf> &classes) {
  ROS_INFO_STREAM("Computing distance maps...");
  //generate the sample grid
  Eigen::Array2Xf grid(2, classes[0].rows()*2*classes[0].cols()*2);
  samplePts(Eigen::Vector2f(0, 0), 0, grid, classes[0].cols()*2, classes[0].rows()*2, 1);
  ROS_INFO_STREAM("Sample grid generated");

  //This is really inefficient, but it's precomputed so who cares
  for (int cls_id=0; cls_id<classes.size(); cls_id++) {
    for (int row=0; row<classes[cls_id].rows(); row++) {
      for (int col=0; col<classes[cls_id].cols(); col++) {
        if (classes[cls_id](row, col) == 0) continue;

        float closest_dist = 50; //max distance allowed

        //Iterate through all pts
        for (int comp_row=-closest_dist; comp_row<=closest_dist; comp_row++) {
          for (int comp_col=-closest_dist; comp_col<=closest_dist; comp_col++) {
            if (comp_row+row >= classes[cls_id].rows() || comp_col+col >= classes[cls_id].cols() ||
                comp_row+row < 0 || comp_col+col < 0) continue;
            //First approximation
            if (abs(comp_row) > closest_dist || abs(comp_col) > closest_dist) continue;
            //Verify that pt is of the appropriate class
            if (classes[cls_id](comp_row+row, comp_col+col) > 0) continue;

            int ind = comp_row+classes[cls_id].rows() + (comp_col+classes[cls_id].cols())*2*classes[cls_id].rows();

            float dist = grid.matrix().col(ind).norm();
            if (dist < closest_dist) {
              closest_dist = dist;
            }
          }
        }
        classes[cls_id](row, col) = closest_dist;
      }
    }
    classes[cls_id] *= resolution_;
    ROS_INFO_STREAM("class " << cls_id << " complete");
  }
}

void TopDownMap::getClasses(Eigen::Ref<Eigen::Array2Xf> pts, std::vector<Eigen::ArrayXXf> &classes) {
  if (classes.size() < poly_.size()) return;
  //Algorithm modified from https://en.wikipedia.org/wiki/Even-odd_rule
  int cls_id = 0;
  for (auto cls : poly_) {
    Eigen::Map<Eigen::Array1Xf> class_fills(classes[cls_id].data(), 1, classes[cls_id].size());
    Eigen::Array1Xf class_fills_buf(classes[cls_id].size());
    class_fills = -1;
    for (auto poly : cls) {
      int j = poly.size()-1;
      class_fills_buf = -1;
      for (int i=0; i<poly.size(); i++) {
        class_fills_buf *= -2*((pts.row(0) < poly[i][1]).cwiseNotEqual(pts.row(0) < poly[j][1]) * 
                              (pts.row(1) < (poly[i][0] + ((poly[j][0]-poly[i][0]) * (pts.row(0)-poly[i][1]) / 
                              (poly[j][1]-poly[i][1]))))).cast<float>() + 1;
        j = i;
      }
      class_fills = class_fills.max(class_fills_buf);
    }
    class_fills *= -1; //invert
    class_fills += 1;
    class_fills /= 2;
    cls_id++;
  }

  //Only one ground type per cell
  for (int under_cls_id=0; under_cls_id<num_exclusive_classes_; under_cls_id++) {
    for (int cls_id=under_cls_id+1; cls_id<num_exclusive_classes_; cls_id++) {
      classes[under_cls_id] += 1-classes[cls_id];
    }
    classes[under_cls_id] = classes[under_cls_id].cwiseMin(1);
  }
}

void TopDownMap::samplePts(Eigen::Vector2f center, float rot, Eigen::Array2Xf &pts, int cols, int rows, float res) {
  //Generate the sampling coordinates
  Eigen::Map<Eigen::ArrayXXf, 0, Eigen::Stride<2, Eigen::Dynamic>> x_vals(
      pts.data(), cols, rows, 
      Eigen::Stride<2, Eigen::Dynamic>(2, rows*2));
  Eigen::Map<Eigen::ArrayXXf, 0, Eigen::Stride<Eigen::Dynamic, 2>> y_vals(
      pts.row(1).data(), rows, cols, 
      Eigen::Stride<Eigen::Dynamic, 2>(rows*2, 2));

  ROS_INFO_STREAM("maps generated");

  x_vals = Eigen::RowVectorXf::LinSpaced(rows, 
      -res*(rows-1)/2., res*(rows-1)/2.).replicate(cols, 1);

  y_vals = Eigen::RowVectorXf::LinSpaced(cols, 
      -res*(cols-1)/2., res*(cols-1)/2.).replicate(rows, 1);

  ROS_INFO_STREAM("transforming coord");

  //Transform coordinates
  Eigen::Matrix2f rotm;
  rotm << cos(rot), -sin(rot),
          sin(rot), cos(rot);
  pts = rotm*pts.matrix();

  ROS_INFO_STREAM("coord rotated");

  x_vals += center[1];
  y_vals += center[0];
}

void TopDownMap::getRasterMap(Eigen::Vector2f center, float rot, float res, std::vector<Eigen::ArrayXXf> &classes) {
  if (classes.size() < 1) return;

  Eigen::Array2Xf pts(2, classes[0].rows()*classes[0].cols());
  samplePts(center, rot, pts, classes[0].cols(), classes[0].rows(), res);

  getClasses(pts, classes);
}

void TopDownMap::getLocalMap(Eigen::Vector2f center, float rot, float res, std::vector<Eigen::ArrayXXf> &dists) {
  if (dists.size() < 1) return;
  Eigen::Array2Xf pts(2, dists[0].rows()*dists[0].cols());
  samplePts(center/resolution_, rot, pts, dists[0].cols(), dists[0].rows(), res/resolution_);

  //Generate list of indices
  Eigen::Array2Xi pts_int = pts.round().cast<int>();

  for (int cls=0; cls<dists.size(); cls++) {
    for (int idx=0; idx<dists[0].rows()*dists[0].cols(); idx++) {
      if (pts_int(0, idx) >= 0 && pts_int(0, idx) < class_maps_[cls].rows() &&
          pts_int(1, idx) >= 0 && pts_int(1, idx) < class_maps_[cls].cols()) {
        dists[cls](idx) = class_maps_[cls](pts_int(0, idx), pts_int(1, idx));
      } else {
        dists[cls](idx) = 100;
      }
    }
  }
}
