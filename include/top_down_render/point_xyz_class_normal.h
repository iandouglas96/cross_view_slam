#ifndef POINT_XYZ_CLASS_NORMAL_
#define POINT_XYZ_CLASS_NORMAL_

#include <pcl_ros/point_cloud.h>

class PointXYZClassNormal {
  public:
    pcl::PointXYZRGB pt_xyz;
    pcl::Normal pt_normal;

    PointXYZClassNormal(const pcl::PointXYZRGB &xyz, const pcl::Normal &norm){
      pt_xyz = xyz;
      pt_normal = norm;
    }

    bool operator<(const PointXYZClassNormal &other) const {
      return pt_xyz.z<other.pt_xyz.z;
    }

    bool operator>(const PointXYZClassNormal &other) const {
      return pt_xyz.z>other.pt_xyz.z;
    }
};

#endif //POINT_XYZ_CLASS_NORMAL_
