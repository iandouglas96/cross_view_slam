#include <ros/ros.h>
#include "top_down_render/top_down_render.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "cross_view_slam");
  ros::NodeHandle nh("~");

  TopDownRender node(nh);
  node.initialize();
  ros::spin();

  return 0;
}
