#include <vector>
#include <array>
#include <map>
#include <mutex>
#include <functional>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Vector3.h>
#include <tf2_ros/transform_listener.h>

struct Map
{
    // cv::Mat1b land;
    // cv::Mat1b maxLand;
    cv::Mat1b wall;
    // cv::Mat1b unknown;
    cv::Point2d base;
    // cv::Mat_<int8_t> mapRaw;
    double resolution = 0;
    std::mutex mtx;
};

struct SubmapId {
  SubmapId(int trajectory_id, int submap_index)
      : trajectory_id(trajectory_id), submap_index(submap_index) {}

  int trajectory_id;
  int submap_index;

  bool operator==(const SubmapId& other) const {
    return std::forward_as_tuple(trajectory_id, submap_index) ==
           std::forward_as_tuple(other.trajectory_id, other.submap_index);
  }

  bool operator!=(const SubmapId& other) const { return !operator==(other); }

  bool operator<(const SubmapId& other) const {
    return std::forward_as_tuple(trajectory_id, submap_index) <
           std::forward_as_tuple(other.trajectory_id, other.submap_index);
  }
};

typedef std::vector<std::array<double, 2>> FrontierArray;
typedef std::map<SubmapId, FrontierArray> FrontierArrayMap;

FrontierArrayMap process(const FrontierArrayMap &frontiers, Map &globalMap, const tf2_ros::Buffer &tfBuffer, const std::string &base_frame);
