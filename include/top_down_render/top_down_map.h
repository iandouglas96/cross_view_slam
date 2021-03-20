#ifndef TOP_DOWN_MAP_H_
#define TOP_DOWN_MAP_H_

#include <fstream>
#include <sys/stat.h>

#include <ros/ros.h> //Just for prints
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#define NANOSVG_CPLUSPLUS
#include "top_down_render/nanosvg.h"

namespace Eigen {
  typedef Array<bool, 1, Dynamic> Array1Xb;
  typedef Array<uint8_t, 1, Dynamic> Array1Xc;
  typedef Array<float, 1, Dynamic> Array1Xf;
  typedef Array<uint8_t, Dynamic, Dynamic> ArrayXXc;
  typedef Array<float, Dynamic, Dynamic> ArrayXXf;
}

//Save/load Eigen matrices to file.
//Modified from https://stackoverflow.com/questions/25389480/how-to-write-read-an-eigen-matrix-from-binary-file
template<class Matrix>
void write_binary(std::string &filename, const Matrix& matrix){
  std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
  typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
	//write size
  out.write((char*) (&rows), sizeof(typename Matrix::Index));
  out.write((char*) (&cols), sizeof(typename Matrix::Index));
	//write contents
  out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
  out.close();
}

template<class Matrix>
void read_binary(std::string &filename, Matrix& matrix){
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  typename Matrix::Index rows=0, cols=0;
  in.read((char*) (&rows), sizeof(typename Matrix::Index));
  in.read((char*) (&cols), sizeof(typename Matrix::Index));
  matrix.resize(rows, cols);
  in.read((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
  in.close();
}

class TopDownMap {
  public:
    TopDownMap(std::string path, cv::Mat& color_lut, int num_classes, int num_ex, float res);

    void getClassesAtPoint(const Eigen::Vector2f &center, std::vector<int> &classes);
    void getClassesAtPoint(const Eigen::Vector2i &center_ind, std::vector<int> &classes);
    void getLocalMap(Eigen::Vector2f center, float rot, float res, std::vector<Eigen::ArrayXXf> &dists);
    Eigen::Vector2i size() const;
    int numClasses() const;
    float resolution() const;
  protected:
    std::vector<std::vector<std::vector<Eigen::Vector2f>>> poly_;
    std::vector<Eigen::ArrayXXf> class_maps_;
    float resolution_; //pixels for svg per per pixel for rasterized map
    int num_classes_;
    int num_exclusive_classes_;

    void saveRasterizedMaps(const std::string &path);
    void loadRasterizedMaps(const std::string &path);
    bool loadCacheMetaData(const std::string &path);
    void loadCachedMaps();
    void saveCachedMaps(const std::string &path);
    void getRasterMap(Eigen::Vector2f center, float rot, float res, std::vector<Eigen::ArrayXXf> &classes);
    void computeDists(std::vector<Eigen::ArrayXXf> &classes);
    void getClasses(Eigen::Ref<Eigen::Array2Xf> pts, std::vector<Eigen::ArrayXXf> &classes);
    void samplePts(Eigen::Vector2f center, float rot, Eigen::Array2Xf &pts, int cols, int rows, float res);
};

#endif
