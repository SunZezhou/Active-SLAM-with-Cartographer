#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_FRONTIER_DETECTION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_FRONTIER_DETECTION_H

#include <cartographer/io/submap_painter.h>
#include <cartographer/mapping/2d/submap_2d.h>
#include <cartographer/mapping/2d/probability_grid.h>
#include <cartographer/mapping/id.h>
#include <cartographer/mapping/pose_graph.h>
#include <cartographer/transform/rigid_transform.h>
#include <cartographer_ros_msgs/SubmapList.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace frontier {

// const double kFreeProbability = 0.56;
// const double kOccupiedProbability = 0.5;

// cartographer::mapping::ProbabilityToValue(kFreeProbability) 18841
// cartographer::mapping::ProbabilityToValue(kOccupiedProbability) 16384
// constexpr uint16_t kFreeProbabilityValue = 18841;           // dashgo
// constexpr uint16_t kOccupiedProbabilityValue = 16384;       // dashgo
constexpr uint16_t kFreeProbabilityValue = 17000;              // turtlebot
constexpr uint16_t kOccupiedProbabilityValue = 16500;          // turtlebot


namespace bg = boost::geometry;
namespace bgi = bg::index;

inline Eigen::Isometry2d rigid3d_to_isometry2d(
    const cartographer::transform::Rigid3d& rigid) {
  return Eigen::Translation2d(rigid.translation().head<2>()) *
         Eigen::Rotation2Dd(
             rigid.rotation().toRotationMatrix().block<2, 2>(0, 0));
}

class LambdaWorker {
 public:
  LambdaWorker() : finished_(false) {}

  ~LambdaWorker() {
    finish();
  }

  void finish() {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      finished_ = true;
    }
    condition_.notify_one();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void PushIntoWorkQueue(std::function<void(void)> task) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!finished_) {
      work_queue_.push_back(std::move(task));
      condition_.notify_one();
    }
  }

  int TaskCount() {
    std::unique_lock<std::mutex> lock(mutex_);
    return static_cast<int>(work_queue_.size());
  }

  void start() {
    thread_ = std::thread([this]() {
      while (true) {
        int last_queue_size;
        do {
          std::function<void(void)> top_of_queue;
          {
            std::unique_lock<std::mutex> lock(mutex_);
            if (finished_) return;
            last_queue_size = work_queue_.size();
            if (last_queue_size > 0) {
              top_of_queue = std::move(work_queue_.front());
              work_queue_.pop_front();
            }
          }
          if (last_queue_size > 0) top_of_queue();
        } while (last_queue_size > 1);
        std::unique_lock<std::mutex> lock(mutex_);
        if (work_queue_.size() == 0 && !finished_) {
          condition_.wait(lock, []() { return true; });
        }
      }
    });
  }

 private:
  std::deque<std::function<void(void)>> work_queue_;
  std::thread thread_;

  std::condition_variable condition_;
  std::mutex mutex_;
  bool finished_;
};

class Detector {
 public:
  Detector(cartographer::mapping::PoseGraph* pose_graph);

  // Performs local frontier edge detection.
  void HandleSubmapUpdates(
      const std::vector<cartographer::mapping::SubmapId>& submap_ids);

  void NotifyEnd();
  std::vector<double> isUpdateFrontier(const cartographer::mapping::SubmapId& id);

 private:
  void InitPublisher();
  void PublishAllSubmaps();
  void PublishSubmaps(
      const std::vector<cartographer::mapping::SubmapId>& submap_ids,
      const std::vector<cartographer::mapping::SubmapId>& additional_submaps);

  bool CheckForOptimizationEvent();
  void CheckOptimizationEventsPeriodicallyWhenIdle(const ::ros::WallTimerEvent&);

  std::vector<cartographer::mapping::SubmapId> GetIntersectingFinishedSubmaps(
      const cartographer::mapping::SubmapId& id_i);

  using Point = bg::model::point<double, 2, bg::cs::cartesian>;
  using Box = bg::model::box<Point>;
  using Value = std::pair<Box, cartographer::mapping::SubmapId>;
  using RTree = bgi::rtree<Value, bgi::quadratic<16, 4>>;

  RTree rt_;

  std::mutex mutex_;

  std::map<cartographer::mapping::SubmapId,
           std::pair<
               Eigen::Matrix3Xd,
               std::vector<cartographer::mapping::SubmapId> /* submap_hints */>>
      submap_frontier_points_;

  struct BoundingBoxInfo {
    std::pair<Eigen::Vector3d, Eigen::Vector3d> local_box;
    Box last_global_box;
  };
  std::map<cartographer::mapping::SubmapId, BoundingBoxInfo> bounding_boxes_;

  std::vector<cartographer::mapping::SubmapId> active_submaps_;

  std::map<cartographer::mapping::SubmapId, std::vector<double>> opt_submap_color_;

  // cartographer_ros_msgs::SubmapList::ConstPtr last_submap_list_;
  ros::Publisher frontier_publisher_;
  bool publisher_initialized_;
  int last_optimizations_performed_;

  cartographer::mapping::PoseGraph* pose_graph_;
  LambdaWorker lambda_worker_;
  ros::WallTimer optimization_timer_;

  struct Submap {
    Submap(const cartographer::mapping::SubmapId& id,
           const cartographer::mapping::PoseGraphInterface::SubmapData&
               submap_data,
           std::unique_ptr<cartographer::mapping::ProbabilityGrid> grid_copy)
        : id(id),
          grid_original(*static_cast<const cartographer::mapping::Submap2D*>(
                             submap_data.submap.get())
                             ->grid()),
          local_pose_inverse(submap_data.submap->local_pose().inverse()),
          finished(grid_copy ? false : submap_data.submap->insertion_finished()),
          needUpdate(false) {
      SetGlobalPose(submap_data.pose);
      frontier_marker.header.frame_id = "map";
      frontier_marker.pose.orientation.w = 1.0;
      frontier_marker.type = visualization_msgs::Marker::POINTS;
      frontier_marker.scale.x = 0.075;
      frontier_marker.scale.y = 0.075;
      frontier_marker.color.r = 1.0;
      frontier_marker.color.a = 1.0;
      std::ostringstream ss;
      ss << "Trajectory " << id.trajectory_id << ", submap " << id.submap_index;
      frontier_marker.ns = ss.str();

      if (grid_copy) {
        is_copy = true;
        Submap::grid_copy = std::move(grid_copy);
      } else {
        is_copy = false;
      }
    }

    const cartographer::mapping::SubmapId id;
    // TODO use a shared pointer with the aliasing constructor instead
    // of evil reference member
    const cartographer::mapping::Grid2D& grid_original;
    std::unique_ptr<cartographer::mapping::Grid2D> grid_copy;
    bool is_copy;
    cartographer::transform::Rigid3d old_pose;
    cartographer::transform::Rigid3d pose;
    const cartographer::transform::Rigid3d local_pose_inverse;
    Eigen::Isometry2d to_global_position;
    Eigen::Isometry2d to_local_submap_position;
    Eigen::Matrix2Xd cached_frontier_marker_points_global;
    visualization_msgs::Marker frontier_marker;
    const bool finished;

    bool needUpdate;

    const cartographer::mapping::Grid2D& grid() const {
      return is_copy ? *grid_copy : grid_original;
    }
    const cartographer::mapping::MapLimits& limits() const {
      return grid().limits();
    }

    void SetGlobalPose(const cartographer::transform::Rigid3d& global_pose) {
      pose = global_pose;
      to_global_position = rigid3d_to_isometry2d(pose * local_pose_inverse);
      to_local_submap_position = to_global_position.inverse();

      Eigen::Vector3d delta = old_pose.translation() - pose.translation();
      if(std::abs(delta[0]) < 0.05 && std::abs(delta[1]) < 0.05) return;
      old_pose = pose;
      needUpdate = true;
    }

                       int ToFlatIndex(const Eigen::Array2i& cell_index) const {
      return limits().cell_limits().num_x_cells * cell_index.y() +
             cell_index.x();
    }

    bool is_in_limits(const Eigen::Array2i& xy_index) const {
      return limits().Contains(xy_index);
    }

    bool is_free(const Eigen::Array2i& xy_index) const {
      return is_in_limits(xy_index) &&
             grid().correspondence_cost_cells_[ToFlatIndex(xy_index)] >=
                 kFreeProbabilityValue;
    }

    bool is_unknown(const Eigen::Array2i& xy_index) const {
      if (!is_in_limits(xy_index)) return true;
      const int& value =
          grid().correspondence_cost_cells_[ToFlatIndex(xy_index)];
      return value == 0 ||
             (value > kOccupiedProbabilityValue && !is_free(xy_index));
    }

    bool is_known(const Eigen::Vector3d& local_position_in_submap) const {
      return !is_unknown(limits().GetCellIndex(
          local_position_in_submap.head<2>().cast<float>()));
    }

    bool checkNeedUpdate() {
      bool ret = needUpdate;
      needUpdate = false;
      return ret;
    }
  };

  class SubmapCache {
   public:
    SubmapCache(const cartographer::mapping::PoseGraph* const pose_graph)
        : pose_graph_(pose_graph) {}
    Detector::Submap& operator()(
        const cartographer::mapping::SubmapId& id) const {
      return *IfExists(id);
    }

    void UpdateCacheWithCopy(
        const cartographer::mapping::SubmapId& id,
        const cartographer::mapping::PoseGraphInterface::SubmapData&
            submap_data,
        std::unique_ptr<cartographer::mapping::ProbabilityGrid> grid_copy) {
      auto iter = submaps_.lower_bound(id);
      if (iter == submaps_.end() || iter->first != id) {
        submaps_.emplace_hint(
            iter,
            std::make_pair(id, absl::make_unique<Detector::Submap>(
                                   id, submap_data, std::move(grid_copy))));
      } else {
        iter->second = absl::make_unique<Detector::Submap>(
            Detector::Submap(id, submap_data, std::move(grid_copy)));
      }
    }

    Submap* IfExists(const cartographer::mapping::SubmapId& id) const {
      auto iter = submaps_.lower_bound(id);
      if (iter != submaps_.end() && iter->first == id)
        return iter->second.get();
      auto submap_data = pose_graph_->GetSubmapData(id);
      if (submap_data.submap != nullptr) {
        // LOG(INFO) << "Cache has seen a new submap " << id;
        return submaps_
            .emplace_hint(
                iter, std::make_pair(id, absl::make_unique<Detector::Submap>(
                                             id, submap_data, nullptr)))
            ->second.get();
      } else {
        return nullptr;
      }
    }

    void Invalidate() {
      const auto all_submap_data = pose_graph_->GetAllSubmapData();
      for (auto& submap : submaps_) {
        submap.second->SetGlobalPose(all_submap_data.at(submap.first).pose);
      }
    }

    std::map<cartographer::mapping::SubmapId,
             std::unique_ptr<Detector::Submap>>& submaps() const { return submaps_; }

   private:
    using SubmapPair =
        std::pair<cartographer::mapping::SubmapId, Detector::Submap>;
    mutable std::map<cartographer::mapping::SubmapId,
                     std::unique_ptr<Detector::Submap>>
        submaps_;
    const cartographer::mapping::PoseGraph* const pose_graph_;
  };

  SubmapCache submaps_;

  size_t colorIndex; // only for debug

  // Goes through previously edge-detected local frontier points in submaps,
  // checks if they really are frontier points by looking in other submaps,
  // and creates a marker containing the appropriate frontier points.
  visualization_msgs::Marker& CreateMarkerForSubmap(
      const cartographer::mapping::SubmapId& id_i,
      const std::vector<cartographer::mapping::SubmapId>* updated_submaps,
      bool check_against_active);

  Box CalculateBoundingBox(const Submap& submap) {
    auto& bounding_box_info = bounding_boxes_[submap.id];
    const Eigen::Vector3d p1_global =
        submap.pose * bounding_box_info.local_box.first;
    const Eigen::Vector3d p2_global =
        submap.pose * bounding_box_info.local_box.second;
    const Eigen::Vector3d p3_global =
        submap.pose * Eigen::Vector3d{bounding_box_info.local_box.first.x(),
                                      bounding_box_info.local_box.second.y(),
                                      0.};
    const Eigen::Vector3d p4_global =
        submap.pose * Eigen::Vector3d{bounding_box_info.local_box.second.x(),
                                      bounding_box_info.local_box.first.y(),
                                      0.};

    const auto minmax_x = std::minmax(
        {p1_global.x(), p2_global.x(), p3_global.x(), p4_global.x()});
    const auto minmax_y = std::minmax(
        {p1_global.y(), p2_global.y(), p3_global.y(), p4_global.y()});

    /*visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    std::ostringstream ss;
    ss << "BBOX Trajectory " <<submap.id.trajectory_id << ", submap " <<
    submap.id.submap_index; marker.ns = ss.str();

    geometry_msgs::Point point;
    point.x = minmax_x.first;
    point.y = minmax_y.first;
    marker.points.push_back(point);
    point.x = minmax_x.second;
    point.y = minmax_y.first;
    marker.points.push_back(point);
    marker.points.push_back(point);
    point.x = minmax_x.second;
    point.y = minmax_y.second;
    marker.points.push_back(point);
    marker.points.push_back(point);
    point.x = minmax_x.first;
    point.y = minmax_y.second;
    marker.points.push_back(point);
    marker.points.push_back(point);
    point.x = minmax_x.first;
    point.y = minmax_y.first;
    marker.points.push_back(point);

    visualization_msgs::MarkerArray frontier_markers;
    frontier_markers.markers.push_back(marker);
    frontier_publisher_.publish(frontier_markers);*/

    return Box{Point(minmax_x.first, minmax_y.first),
               Point(minmax_x.second, minmax_y.second)};
  }

  void RebuildTree();
};

}  // namespace frontier

#endif
