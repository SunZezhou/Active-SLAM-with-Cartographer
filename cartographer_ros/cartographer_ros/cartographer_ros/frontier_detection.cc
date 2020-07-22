// clang-format off
#pragma GCC target ("arch=broadwell")
// clang-format on

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

// #define COLORFUL
// #define TYPES_4
// #define COUNT_DATA_LOG

// #define BFS_LOOPY_SEARCH

#ifndef BFS_LOOPY_SEARCH
#define BFS_DIRECTLY_SEARCH
#endif

#define INFLATE_MAP

#define DEFAULT_COLOR {1.0, 1.0, 1.0, 0.1}
#define OPT_COLOR {1.0, 0.0, 0.0, 1.0}
#define NEWLY_COLOR {0.0, 0.0, 0.0, 0.0}

#include <absl/synchronization/mutex.h>
#include <cartographer/mapping/2d/map_limits.h>
#include <cartographer_ros/frontier_detection.h>
#include <cartographer_ros/msg_conversion.h>

int total_submap_updates = 0;
int optimization_events = 0;
int skipped_updates = 0;

std::vector<std::array<uint8_t,3>> colormap = {{{255, 0, 0}}};/*{
  {{0, 0, 255}},
  {{0, 96, 255}},
  {{0, 192, 255}},
  {{0, 255, 222}},
  {{0, 255, 126}},
  {{0, 255, 30}},
  {{66, 255, 0}},
  {{162, 255, 0}},
  {{250, 247, 0}},
  {{255, 156, 0}},
  {{255, 60, 0}},
  {{255, 0, 36}},
  {{255, 0, 132}},
  {{255, 0, 228}},
  {{186, 0, 255}},
  {{90, 0, 255}}
};*/

const int core_radius = 8; // for inflate

std::vector<std::pair<int, int>> inflateIt;

namespace frontier {

Detector::Detector(cartographer::mapping::PoseGraph* const pose_graph)
    : publisher_initialized_(false),
      last_optimizations_performed_(-1),
      pose_graph_(pose_graph),
      submaps_(pose_graph_) {
  lambda_worker_.start();
  optimization_timer_ = ros::NodeHandle().createWallTimer(
      ::ros::WallDuration(0.2), &Detector::CheckOptimizationEventsPeriodicallyWhenIdle, this);
  for(int r = -core_radius; r <= core_radius; r++) {
    for(int c = -core_radius; c <= core_radius; c++) {
      inflateIt.push_back({r, c});
    }
  }
#ifdef COUNT_DATA_LOG
  ROS_ERROR_STREAM("count-log");
#endif
#ifdef TYPES_4
  ROS_ERROR_STREAM("type-4");
#endif
#ifdef BFS_LOOPY_SEARCH
  ROS_ERROR_STREAM("bfs");
#endif
#ifndef BFS_LOOPY_SEARCH
#ifdef BFS_DIRECTLY_SEARCH
  ROS_ERROR_STREAM("direct");
#else
  ROS_ERROR_STREAM("naive");
#endif
#endif
}

void Detector::NotifyEnd() {
  optimization_timer_.stop();
  lambda_worker_.finish();
}

std::vector<double> Detector::isUpdateFrontier(const cartographer::mapping::SubmapId& id) {
#ifdef COLORFUL
  auto ret = opt_submap_color_.find(id);
  if(ret == opt_submap_color_.end()) {
    return NEWLY_COLOR;
  }
  return ret->second;
#else
  return DEFAULT_COLOR;
#endif
}

void Detector::InitPublisher() {
  frontier_publisher_ =
      ros::NodeHandle().advertise<visualization_msgs::MarkerArray>(
          "frontier_marker", 3, true);
#ifdef FRONTIER_COLORFUL
  frontier_colorful = ros::NodeHandle().advertise<visualization_msgs::Marker>("frontier_colorful", 3, true);
#endif
  publisher_initialized_ = true;
}

void Detector::PublishAllSubmaps() {
  if (!publisher_initialized_) InitPublisher();
  visualization_msgs::MarkerArray frontier_markers;

  std::unique_lock<std::mutex> lock(mutex_);

  // TODO: add a constraint condition to reduce updating speed.
  std::set<cartographer::mapping::SubmapId> update_ids;
#ifdef BFS_LOOPY_SEARCH
  std::queue<cartographer::mapping::SubmapId> bfs_ids;
  for(const auto& id : active_submaps_) {
    bfs_ids.push(id);
  }
  while(!bfs_ids.empty()) {
    auto current = bfs_ids.front();
    bfs_ids.pop();
    update_ids.insert(current);
    auto& bounding_box = bounding_boxes_.at(current);
    std::vector<Value> intersecting_submaps;
    intersecting_submaps.reserve(32);
    rt_.query(bgi::intersects(bounding_box.last_global_box),
              std::back_inserter(intersecting_submaps));
    for(const Value& v : intersecting_submaps) {
      if(update_ids.find(v.second) != update_ids.end())
        continue;
      if(!submaps_(v.second).checkNeedUpdate()) {
        update_ids.insert(v.second);
        continue;
      }
      bfs_ids.push(v.second);
    }
  }
#elif defined BFS_DIRECTLY_SEARCH
  for (const auto& submap_i : submap_frontier_points_) {
    if(submaps_(submap_i.first).checkNeedUpdate()) {
      update_ids.insert(submap_i.first);
      
      auto& bounding_box = bounding_boxes_.at(submap_i.first);
      std::vector<Value> intersecting_submaps;
      intersecting_submaps.reserve(32);
      rt_.query(bgi::intersects(bounding_box.last_global_box),
                std::back_inserter(intersecting_submaps));
      for(const Value& v : intersecting_submaps) {
        update_ids.insert(v.second);
      }
    }
  }
#else
  for (const auto& submap_i : submap_frontier_points_) {
    update_ids.insert(submap_i.first);
  }
#endif // search submaps
#ifdef COLORFUL
  for(const auto& submap_i : submap_frontier_points_) {
    opt_submap_color_[submap_i.first] = DEFAULT_COLOR;
  }
  for(const auto& id : update_ids) {
    opt_submap_color_[id] = OPT_COLOR;
  }
#endif

#ifdef COUNT_DATA_LOG
  size_t check_count = 0;
  size_t frontier_count = 0;
  size_t total_count = 0;
#endif
  auto t = ros::Time::now();
  for (const auto& submap_i : update_ids) { // Algo3.L5
    size_t once_total_count = 0;
    frontier_markers.markers.push_back(
        CreateMarkerForSubmap(submap_i, nullptr /* updated_submap_ids */,
                              true /* check_against_active */, &once_total_count)); // Algo3.L8
#ifdef COUNT_DATA_LOG
    check_count += once_total_count;
#endif
  }
  double delta_t = (ros::Time::now() - t).toSec();
#ifdef FRONTIER_COLORFUL
  visualization_msgs::Marker frontier_colorful_marker;
  frontier_colorful_marker.header.frame_id = "map";
  frontier_colorful_marker.pose.orientation.w = 1.0;
  frontier_colorful_marker.type = visualization_msgs::Marker::POINTS;
  frontier_colorful_marker.scale.x = 0.075;
  frontier_colorful_marker.scale.y = 0.075;
  frontier_colorful_marker.color.r = 1.0;
  frontier_colorful_marker.color.a = 1.0;
  frontier_colorful_marker.ns = "colorful";
  for(const auto& m : frontier_markers.markers) {
    frontier_colorful_marker.points.insert(frontier_colorful_marker.points.end(), m.points.begin(), m.points.end());
  }
  frontier_colorful.publish(frontier_colorful_marker);
#endif
#ifdef COUNT_DATA_LOG
  for(const auto& submap_i : submap_frontier_points_) {
    total_count += submap_i.second.second.size();
    for(const auto& hint : submap_i.second.second) {
      if(hint == cartographer::mapping::SubmapId{-1, -1})
        ++frontier_count;
    }
  }
  ROS_INFO_STREAM(", " << delta_t << ", " << total_count << ", " << frontier_count << "," << check_count << ";");
#endif

#ifdef TYPES_4
  size_t TP = 0, FP = 0, TN = 0, FN = 0;
  for(const auto& submap_i : submap_frontier_points_) {
    const Submap& s_i(submaps_(submap_i.first));
    const auto& submap_frontier_points = submap_i.second;
    const auto& bounding_box = bounding_boxes_.at(submap_i.first);
    std::vector<Value> intersecting_submaps;
    intersecting_submaps.reserve(32);
    rt_.query(bgi::intersects(bounding_box.last_global_box),
              std::back_inserter(intersecting_submaps));
    Eigen::Matrix3Xd submap_frontier_points_global =
        (s_i.to_global_position * submap_frontier_points.first); // transform to global g
    for (int i = 0; i < submap_frontier_points_global.cols(); i++) {
      const auto global_position = submap_frontier_points_global.col(i);
      bool ok = true;
      const auto& submap_hint = submap_frontier_points.second.at(i);
      if (submap_hint != cartographer::mapping::SubmapId{-1, -1}) { // first test hint
        Submap* s_j = submaps_.IfExists(submap_hint);
        if (s_j != nullptr) {
          if (s_j->is_known(s_j->to_local_submap_position * global_position)) {
            ++TN;
            ok = false;
          }
        }
      }
      if (ok) { // then test active submaps
        for (const auto& active_submap : active_submaps_) {
          const cartographer::mapping::SubmapId& id_j = active_submap;
          if (id_j == s_i.id ||
              (id_j.trajectory_id == s_i.id.trajectory_id &&
               (abs(id_j.submap_index - s_i.id.submap_index) <= 2)))
            continue;
          if (!bg::intersects(
                  bounding_box.last_global_box,
                  bounding_boxes_.at(active_submap).last_global_box)) {
            continue;
          }
          Submap* s_j = submaps_.IfExists(id_j);
          if (s_j == nullptr) {
            continue;
          }
          if (s_j->is_known(s_j->to_local_submap_position * global_position)) {
            ++FN;
            ok = false;
            break;
          }
        }
      }
      if (ok) { // finally test all intersecting submaps
        for (const auto& intersecting_submap : intersecting_submaps) {
          const cartographer::mapping::SubmapId& id_j =
              intersecting_submap.second;
          if (id_j == s_i.id ||
              (id_j.trajectory_id == s_i.id.trajectory_id &&
               (abs(id_j.submap_index - s_i.id.submap_index) <= 2))) {
            continue;
          }
          Submap* s_j = submaps_.IfExists(id_j);
          if (s_j == nullptr) {
            continue;
          }
          if (s_j->is_known(s_j->to_local_submap_position * global_position)) {
            ++FN;
            ok = false;
            break;
          }
        }
      }
      if (ok) {
        if(submap_hint != cartographer::mapping::SubmapId{-1, -1}) ++FP;
        else ++TP;
      }
    }
  }
  ROS_INFO_STREAM(", " << TP << ", " << FP << ", " << TN << ", " << FN);
#endif

  frontier_publisher_.publish(frontier_markers);
  colorIndex++;
}

void Detector::PublishSubmaps(
    const std::vector<cartographer::mapping::SubmapId>& submap_ids,
    const std::vector<cartographer::mapping::SubmapId>& additional_submaps) {
  if (!publisher_initialized_) InitPublisher();

  visualization_msgs::MarkerArray frontier_markers;

  // std::set<cartographer::mapping::SubmapId> update_submaps = update_submaps_;
  for (const auto& id_i : submap_ids) {
    frontier_markers.markers.push_back(
        CreateMarkerForSubmap(id_i, nullptr /* updated_submap_ids */,
                              true /* check_against_active */)); // Algo1.L10
// #ifdef COLORFUL
//     opt_submap_color_[id_i] = NEWLY_COLOR;
// #endif
  }

  for (const auto& id_additional : additional_submaps) {
    frontier_markers.markers.push_back(CreateMarkerForSubmap(
        id_additional, &submap_ids /* updated_submap_ids */,
        true /* check_against_active */)); // Algo3.L13-15
// #ifdef COLORFUL
//     opt_submap_color_[id_additional] = NEWLY_COLOR;
// #endif
  }
  // update_submaps_.swap(update_submaps);

  frontier_publisher_.publish(frontier_markers);
  colorIndex++;
}

bool Detector::CheckForOptimizationEvent() {
  int actual_optimizations_performed = pose_graph_->optimizations_performed();
  if (actual_optimizations_performed != last_optimizations_performed_) {
    last_optimizations_performed_ = actual_optimizations_performed;
  } else
    return false;
  optimization_events++;
  submaps_.Invalidate();
  RebuildTree();
  PublishAllSubmaps();
  return true;
}

void Detector::CheckOptimizationEventsPeriodicallyWhenIdle(
    const ::ros::WallTimerEvent&) {
  if (lambda_worker_.TaskCount() == 0) {
    lambda_worker_.PushIntoWorkQueue([this]() { CheckForOptimizationEvent(); });
  }
}

visualization_msgs::Marker& Detector::CreateMarkerForSubmap(
    const cartographer::mapping::SubmapId& id_i,
    const std::vector<cartographer::mapping::SubmapId>* const
        updated_submap_ids,
    const bool check_against_active,
    size_t* all_count,
    size_t* delta_count) {
  Submap& s_i(submaps_(id_i));

  s_i.frontier_marker.points.clear();

  std::vector<double> updated_frontier_marker_points_global;

  if (updated_submap_ids == nullptr) {
    // delta_count vars
    size_t old_hint_count = 0;
    size_t new_fail_count = 0;
    size_t hint_fail_again = 0;
    // When recomputing all submaps, or when computing only updated submaps.
    auto& submap_frontier_points = submap_frontier_points_.at(s_i.id);
    auto& bounding_box = bounding_boxes_.at(s_i.id);

    std::vector<Value> intersecting_submaps;
    intersecting_submaps.reserve(32);
    rt_.query(bgi::intersects(bounding_box.last_global_box),
              std::back_inserter(intersecting_submaps)); // Alog2.L3

    updated_frontier_marker_points_global.reserve(
        static_cast<size_t>(submap_frontier_points.first.cols()) * 2);

    Eigen::Matrix3Xd submap_frontier_points_global =
        (s_i.to_global_position * submap_frontier_points.first); // transform to global g
    for (int i = 0; i < submap_frontier_points_global.cols(); i++) {
      const auto global_position = submap_frontier_points_global.col(i);

      bool ok = true;
      auto& submap_hint = submap_frontier_points.second.at(i);

      if (submap_hint != cartographer::mapping::SubmapId{-1, -1}) { // first test hint
        ++old_hint_count;
        Submap* s_j = submaps_.IfExists(submap_hint);
        if (s_j == nullptr) {
          submap_hint = {-1, -1};
        } else {
          if (s_j->is_known(s_j->to_local_submap_position * global_position)) {
            ++hint_fail_again;
            ok = false;
          }
        }
        if (ok) submap_hint = {-1, -1};
      }

      if (ok && check_against_active) { // then test active submaps
        for (const auto& active_submap : active_submaps_) {
          const cartographer::mapping::SubmapId& id_j = active_submap;
          if (id_j == s_i.id ||
              (id_j.trajectory_id == s_i.id.trajectory_id &&
               (abs(id_j.submap_index - s_i.id.submap_index) <= 2)))
            continue;
          if (!bg::intersects(
                  bounding_box.last_global_box,
                  bounding_boxes_.at(active_submap).last_global_box)) {
            continue;
          }

          Submap* s_j = submaps_.IfExists(id_j);
          if (s_j == nullptr) {
            continue;
          }
          if (s_j->is_known(s_j->to_local_submap_position * global_position)) {
            ++new_fail_count;
            ok = false;
            submap_hint = s_j->id;
            break;
          }
        }
      }

      if (ok) { // finally test all intersecting submaps
        for (const auto& intersecting_submap : intersecting_submaps) {
          const cartographer::mapping::SubmapId& id_j =
              intersecting_submap.second;
          if (id_j == s_i.id ||
              (id_j.trajectory_id == s_i.id.trajectory_id &&
               (abs(id_j.submap_index - s_i.id.submap_index) <= 2))) {
            continue;
          }

          Submap* s_j = submaps_.IfExists(id_j);
          if (s_j == nullptr) {
            rt_.remove(
                std::make_pair(bounding_boxes_.at(id_j).last_global_box, id_j));
            continue;
          }

          if (s_j->is_known(s_j->to_local_submap_position * global_position)) {
            ++new_fail_count;
            ok = false;
            submap_hint = id_j;
            break;
          }
        }
      }

      if (ok) { // Algo2.L4
        updated_frontier_marker_points_global.insert(
            updated_frontier_marker_points_global.end(),
            {global_position.x(), global_position.y()});
        geometry_msgs::Point frontier_point;
        frontier_point.x = global_position.x();
        frontier_point.y = global_position.y();
        frontier_point.z = 0.;  // global_position.z();
        s_i.frontier_marker.points.push_back(frontier_point);
        // submap_hint = {-1, -1};
      }
    }
    if(all_count != nullptr) *all_count = submap_frontier_points_global.cols();
    if(delta_count != nullptr) *delta_count = old_hint_count + new_fail_count - hint_fail_again;
  } else {
    // Computing the intersecting submaps. Check if frontier points were
    // covered by an active submap update.
    updated_frontier_marker_points_global.reserve(
        static_cast<size_t>(s_i.cached_frontier_marker_points_global.cols()) *
        2);

    std::vector<Eigen::Array2Xi> indices_in_updated_submaps;
    indices_in_updated_submaps.reserve(updated_submap_ids->size());

    std::vector<Submap*> updated_submaps;
    updated_submaps.reserve(updated_submap_ids->size());
    for (int i = 0; i < static_cast<int>(updated_submap_ids->size()); i++) {
      updated_submaps.push_back(&submaps_((*updated_submap_ids)[i]));
      indices_in_updated_submaps.push_back(
          (((updated_submaps[i]->to_local_submap_position *
             s_i.cached_frontier_marker_points_global)
                .array()
                .colwise() -
            updated_submaps[i]->limits().max().array()) /
               (-updated_submaps[i]->limits().resolution()) -
           0.5)
              .round()
              .cast<int>());
    }

    for (int j = 0; j < s_i.cached_frontier_marker_points_global.cols(); j++) {
      const auto global_position =
          s_i.cached_frontier_marker_points_global.col(j);

      bool ok = true;

      for (int i = 0; i < static_cast<int>(updated_submap_ids->size()); i++) { // Algo1.L13
        ok &= updated_submaps[i]->is_unknown(
            {indices_in_updated_submaps[i](1, j),
             indices_in_updated_submaps[i](0, j)});
      }

      if (ok) { // Algo1.L15
        updated_frontier_marker_points_global.insert(
            updated_frontier_marker_points_global.end(),
            {global_position.x(), global_position.y()});
        geometry_msgs::Point frontier_point;
        frontier_point.x = global_position.x();
        frontier_point.y = global_position.y();
        frontier_point.z = 0.;  // global_position.z();
        s_i.frontier_marker.points.push_back(frontier_point);
        // submap_hint = {-1, -1};
      }
    }
  }
  s_i.cached_frontier_marker_points_global = Eigen::Map<Eigen::Matrix2Xd>(
      updated_frontier_marker_points_global.data(), 2,
      updated_frontier_marker_points_global.size() / 2);

  s_i.frontier_marker.color.r = colormap[colorIndex % colormap.size()][0] / 255.0;
  s_i.frontier_marker.color.g = colormap[colorIndex % colormap.size()][1] / 255.0;
  s_i.frontier_marker.color.b = colormap[colorIndex % colormap.size()][2] / 255.0;
  return s_i.frontier_marker;
}

std::vector<cartographer::mapping::SubmapId>
Detector::GetIntersectingFinishedSubmaps(
    const cartographer::mapping::SubmapId& id_i) {
  std::vector<Value> intersecting_submaps;
  rt_.query(bgi::intersects(bounding_boxes_.at(id_i).last_global_box),
            std::back_inserter(intersecting_submaps));
  std::vector<cartographer::mapping::SubmapId> result;
  result.reserve(intersecting_submaps.size());
  for (const auto& intersecting_submap : intersecting_submaps) {
    result.push_back(intersecting_submap.second);
  }
  return result;
}

void Detector::HandleSubmapUpdates(
    const std::vector<cartographer::mapping::SubmapId>& submap_ids) {
  bool do_not_skip = false;
  total_submap_updates++;
  std::vector<cartographer::mapping::PoseGraphInterface::SubmapData>
      submap_data(submap_ids.size());
  for (int i = 0; i < static_cast<int>(submap_ids.size()); i++) {
    const auto& id_i = submap_ids[i];
    submap_data[i] = pose_graph_->GetSubmapData(id_i);
    if (submap_data[i].submap->insertion_finished()) {
      do_not_skip = true;
    }
  }

  if (!do_not_skip && lambda_worker_.TaskCount() > 10) {
    skipped_updates++;
    return;
  }

  auto* submap_copies_ptr = new std::vector<
      std::pair<cartographer::mapping::PoseGraphInterface::SubmapData,
                std::unique_ptr<cartographer::mapping::ProbabilityGrid>>>(
      submap_ids.size());
  for (int i = 0; i < static_cast<int>(submap_ids.size()); i++) {
    (*submap_copies_ptr)[i] = std::make_pair(
        submap_data[i],
        //submap_data[i].submap->insertion_finished() ? nullptr :
        absl::make_unique<cartographer::mapping::ProbabilityGrid>(
            *static_cast<const cartographer::mapping::ProbabilityGrid*>(
                static_cast<const cartographer::mapping::Submap2D*>(
                    submap_data[i].submap.get())
                    ->grid())));
  }

  lambda_worker_.PushIntoWorkQueue([this, submap_copies_ptr, submap_ids]() {
    std::unique_ptr<std::vector<
        std::pair<cartographer::mapping::PoseGraphInterface::SubmapData,
                  std::unique_ptr<cartographer::mapping::ProbabilityGrid>>>>
        submap_copies(submap_copies_ptr);

    std::vector<cartographer::mapping::SubmapId> additional_submaps_to_publish;

    for (int i = 0; i < static_cast<int>(submap_ids.size()); i++) {
      submaps_.UpdateCacheWithCopy(submap_ids[i],
                                   std::move((*submap_copies)[i].first),
                                   std::move((*submap_copies)[i].second));
    }

    for (const auto& id_i : submap_ids) { // Algo1.L1
      Submap& s_i(submaps_(id_i));
      CHECK(s_i.is_copy && !s_i.finished || /*!s_i.is_copy && */s_i.finished);

      int sum = 0;
      for (auto& cost : s_i.grid().correspondence_cost_cells()) {
        if (cost != 0) sum++;
      }

      auto& submap_frontier_cells = submap_frontier_points_[id_i];

      int previous_frontier_size =
          static_cast<int>(submap_frontier_cells.second.size());
      submap_frontier_cells.second.clear(); // Alog1.L2 & Algo1.L3

      Eigen::Array2i offset; // Algo1.L6:begin
      cartographer::mapping::CellLimits cell_limits;
      s_i.grid().ComputeCroppedLimits(&offset, &cell_limits);

      const int full_x_dim = s_i.limits().cell_limits().num_x_cells;
      const int full_y_dim = s_i.limits().cell_limits().num_y_cells;

      using DynamicArray =
          Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic>;
      Eigen::Map<const DynamicArray> full_correspondence_costs(
          s_i.grid().correspondence_cost_cells().data(), full_x_dim,
          full_y_dim);

      const int x_dim = cell_limits.num_x_cells;
      const int y_dim = cell_limits.num_y_cells;

      DynamicArray correspondence_costs(x_dim + core_radius * 2, y_dim + core_radius * 2);
      correspondence_costs.topRows(core_radius) = 0;
      correspondence_costs.bottomRows(core_radius) = 0;
      correspondence_costs.leftCols(core_radius) = 0;
      correspondence_costs.rightCols(core_radius) = 0;
      // correspondence_costs.row(0) = 0;
      // correspondence_costs.row(x_dim + 1) = 0;
      // correspondence_costs.col(0) = 0;
      // correspondence_costs.col(y_dim + 1) = 0;

      correspondence_costs.block(core_radius, core_radius, x_dim, y_dim) =
          full_correspondence_costs.block(offset.x(), offset.y(), x_dim, y_dim);

      DynamicArray raw_wall_cells(1 - ((correspondence_costs > 0) * (correspondence_costs <= kOccupiedProbabilityValue)).cast<uint16_t>());
      DynamicArray wall_cells = DynamicArray::Ones(x_dim, y_dim);
#ifdef INFLATE_MAP
      for(const auto& it : inflateIt) {
        wall_cells *= raw_wall_cells.block(core_radius + it.first, core_radius + it.second, x_dim, y_dim);
      }
#endif

      // s_i.set_nowall(DynamicArray::Ones(x_dim, y_dim));//(wall_cells);
#ifdef INFLATE_MAP
      auto ptr = s_i.grid_ptr();
      if(ptr != nullptr && !s_i.walled) {
        s_i.walled = true;
        ROS_WARN_STREAM("set wall" << id_i);
        Eigen::Map<DynamicArray> costs(
          const_cast<uint16_t*>(ptr->correspondence_cost_cells().data()), full_x_dim,
          full_y_dim);
        costs.block(offset.x(), offset.y(), x_dim, y_dim) = costs.block(offset.x(), offset.y(), x_dim, y_dim) * wall_cells + (1 - wall_cells);
      }
#endif

      DynamicArray free_cells(
          (correspondence_costs >= kFreeProbabilityValue).cast<uint16_t>());
      DynamicArray unknown_cells(
          ((correspondence_costs == 0) +
           (correspondence_costs > kOccupiedProbabilityValue) *
               (correspondence_costs < kFreeProbabilityValue))
              .cast<uint16_t>());

      DynamicArray free_neighbours = free_cells.block(core_radius - 1, core_radius - 1, x_dim, y_dim) +
                                     free_cells.block(core_radius - 1, core_radius + 0, x_dim, y_dim) +
                                     free_cells.block(core_radius - 1, core_radius + 1, x_dim, y_dim) +
                                     free_cells.block(core_radius + 0, core_radius + 1, x_dim, y_dim) +
                                     free_cells.block(core_radius + 1, core_radius + 1, x_dim, y_dim) +
                                     free_cells.block(core_radius + 1, core_radius + 0, x_dim, y_dim) +
                                     free_cells.block(core_radius + 1, core_radius - 1, x_dim, y_dim) +
                                     free_cells.block(core_radius + 0, core_radius - 1, x_dim, y_dim);
      free_neighbours *= wall_cells;

      DynamicArray unknown_neighbours =
          unknown_cells.block(core_radius - 1, core_radius - 1, x_dim, y_dim) +
          unknown_cells.block(core_radius - 1, core_radius + 0, x_dim, y_dim) +
          unknown_cells.block(core_radius - 1, core_radius + 1, x_dim, y_dim) +
          unknown_cells.block(core_radius + 0, core_radius + 1, x_dim, y_dim) +
          unknown_cells.block(core_radius + 1, core_radius + 1, x_dim, y_dim) +
          unknown_cells.block(core_radius + 1, core_radius + 0, x_dim, y_dim) +
          unknown_cells.block(core_radius + 1, core_radius - 1, x_dim, y_dim) +
          unknown_cells.block(core_radius + 0, core_radius - 1, x_dim, y_dim);
      unknown_neighbours *= wall_cells;

      // a useful trick from the origin paper's implement
      DynamicArray frontier(unknown_cells.block(core_radius, core_radius, x_dim, y_dim) *
                            (unknown_neighbours >= 3u).cast<uint16_t>() *
                            (free_neighbours >= 3u).cast<uint16_t>()); //Algo1.L6:end

      std::vector<Submap*> previous_submaps_to_cleanup; // trick for near submaps pre-test
      for (int i = 1; i <= 2; i++) {
        const cartographer::mapping::SubmapId id_prev(
            {id_i.trajectory_id, id_i.submap_index - i});
        auto* submap_prev = submaps_.IfExists(id_prev);
        if (submap_prev != nullptr) {
          previous_submaps_to_cleanup.push_back(submap_prev);
        }
      }

      std::vector<int> frontier_cell_indexes_vec;
      frontier_cell_indexes_vec.reserve(
          static_cast<size_t>(previous_frontier_size) * 2);
      for (int y = 0; y < y_dim; y++)
        for (int x = 0; x < x_dim; x++)
          if (frontier(x, y))
            frontier_cell_indexes_vec.insert(frontier_cell_indexes_vec.end(),
                                             {y, x});

      const int num_frontier_candidates =
          static_cast<int>(frontier_cell_indexes_vec.size()) / 2;

      Eigen::Array2Xi frontier_cell_indexes = Eigen::Map<Eigen::Matrix2Xi>(
          frontier_cell_indexes_vec.data(), 2, num_frontier_candidates); // build the index[] of frontier points

      frontier_cell_indexes.colwise() += Eigen::Array2i{offset.y(), offset.x()};

      Eigen::Array2Xd frontier_points(2, num_frontier_candidates);
      frontier_points.colwise() =
          Eigen::Array2d(s_i.limits().max().x(), s_i.limits().max().y());
      frontier_points -= (s_i.limits().resolution()) *
                         (frontier_cell_indexes.cast<double>() + 0.5); // axis adjustment

      std::vector<bool> ok(static_cast<size_t>(num_frontier_candidates), true);
      // Perform stabbing query tests of current submap's s_i frontier
      // cells against temporally previous submaps
      int final_num_frontier_cells = num_frontier_candidates;
      for (const auto& previous_submap : previous_submaps_to_cleanup) { // 在有限范围内作stabbing query test
        Eigen::Array2Xi frontier_cell_2_indexes =
            ((frontier_points.colwise() -
              previous_submap->limits().max().array()) /
                 (-previous_submap->limits().resolution()) -
             0.5)
                .round()
                .cast<int>();

        for (int i = 0; i < num_frontier_candidates; i++) {
          Eigen::Array2i xy_index{frontier_cell_2_indexes(1, i),
                                  frontier_cell_2_indexes(0, i)};
          const bool is_unknown = previous_submap->is_unknown(xy_index);
          if (ok[i] && !is_unknown) final_num_frontier_cells--;
          ok[i] = ok[i] && is_unknown;
        }
      }

      std::vector<double> final_frontier_points_vec;
      final_frontier_points_vec.reserve(
          static_cast<size_t>(final_num_frontier_cells) * 3);
      for (int i = 0; i < num_frontier_candidates; i++) {
        if (ok[i]) {
          final_frontier_points_vec.insert(
              final_frontier_points_vec.end(),
              {frontier_points(0, i), frontier_points(1, i), 1.});
        }
      } // trick finish

      submap_frontier_cells.first = Eigen::Map<Eigen::Matrix3Xd>(
          final_frontier_points_vec.data(), 3, final_num_frontier_cells);
      submap_frontier_cells.second = {
          static_cast<size_t>(final_num_frontier_cells),
          cartographer::mapping::SubmapId{-1, -1}}; // add frontiers to local set

      const double max_x =
          s_i.limits().max().x() - s_i.limits().resolution() * offset.y();
      const double max_y =
          s_i.limits().max().y() - s_i.limits().resolution() * offset.x();
      const double min_x =
          s_i.limits().max().x() -
          s_i.limits().resolution() * (offset.y() + cell_limits.num_y_cells);
      const double min_y =
          s_i.limits().max().y() -
          s_i.limits().resolution() * (offset.x() + cell_limits.num_x_cells);

      const Eigen::Vector3d p1 =
          s_i.local_pose_inverse * Eigen::Vector3d{max_x, max_y, 0.};
      const Eigen::Vector3d p2 =
          s_i.local_pose_inverse * Eigen::Vector3d{min_x, min_y, 0.};
      auto& bounding_box_info = bounding_boxes_[s_i.id];
      bounding_box_info.local_box = std::make_pair(p1, p2);
      bounding_box_info.last_global_box = CalculateBoundingBox(s_i); // Alog1.L4

      for (const auto& previous_submap : previous_submaps_to_cleanup) {
        // Perform testing of intersecting submaps' frontier points against
        // the active submap
        auto& previous_submap_frontier_points =
            submap_frontier_points_.at(previous_submap->id);
        const auto submap_frontier_points_to_cleanup =
            std::move(previous_submap_frontier_points);
        int num_frontier_cells_to_clean =
            static_cast<int>(submap_frontier_points_to_cleanup.second.size());

        Eigen::Array2Xi frontier_cell_2_indexes =
            ((submap_frontier_points_to_cleanup.first.array()
                  .topRows(2)
                  .colwise() -
              s_i.limits().max().array()) /
                 (-s_i.limits().resolution()) -
             0.5)
                .round()
                .cast<int>();

        std::vector<double> final_cleaned_frontier_points_vec;
        final_cleaned_frontier_points_vec.reserve(
            static_cast<size_t>(num_frontier_cells_to_clean) * 3);
        for (int i = 0; i < num_frontier_cells_to_clean; i++) {
          Eigen::Array2i xy_index{frontier_cell_2_indexes(1, i),
                                  frontier_cell_2_indexes(0, i)};
          if (s_i.is_unknown(xy_index)) {
            final_cleaned_frontier_points_vec.insert(
                final_cleaned_frontier_points_vec.end(),
                {submap_frontier_points_to_cleanup.first(0, i),
                 submap_frontier_points_to_cleanup.first(1, i), 1.});
            previous_submap_frontier_points.second.push_back(
                submap_frontier_points_to_cleanup.second[i]);
          }
        }

        previous_submap_frontier_points.first = Eigen::Map<Eigen::Matrix3Xd>(
            final_cleaned_frontier_points_vec.data(), 3,
            previous_submap_frontier_points.second.size());

        if (std::find(additional_submaps_to_publish.begin(),
                      additional_submaps_to_publish.end(),
                      previous_submap->id) ==
            additional_submaps_to_publish.end())
          additional_submaps_to_publish.push_back(previous_submap->id);
      }

      // Keep only finished submaps in the tree in order to avoid lots of
      // insertions and removals while the submaps are being built due to the
      // bounding box being expanded.
      const auto iter =
          std::find(active_submaps_.begin(), active_submaps_.end(), s_i.id); // Algo1.L16
      if (s_i.finished) { // Algo1.L17
        // LOG(WARNING) << "Removing from active submaps: " << s_i.id;
        if (iter != active_submaps_.end()) active_submaps_.erase(iter);
        rt_.insert(std::make_pair(bounding_box_info.last_global_box, s_i.id)); // Algo1.L18
      } else {
        if (iter == active_submaps_.end()) {
          active_submaps_.push_back(s_i.id);
        }
      }

      // intersecting test
      const auto intersecting_submaps = GetIntersectingFinishedSubmaps(id_i); // Algo1.L5
      for (const auto& intersecting_submap : intersecting_submaps) {
        if (std::find(additional_submaps_to_publish.begin(),
                      additional_submaps_to_publish.end(),
                      intersecting_submap) ==
            additional_submaps_to_publish.end()) // Algo1.L19
          additional_submaps_to_publish.push_back(intersecting_submap); // Algo1.L20
      }

      for (const auto& active_submap : active_submaps_) {
        if (active_submap.trajectory_id != s_i.id.trajectory_id &&
            bg::intersects(
                bounding_box_info.last_global_box,
                bounding_boxes_.at(active_submap).last_global_box) &&
             std::find(additional_submaps_to_publish.begin(),
                      additional_submaps_to_publish.end(),
                      active_submap) == additional_submaps_to_publish.end()) // Algo1.L19
          additional_submaps_to_publish.push_back(active_submap); // Algo1.L20
      }
    } // end for submap

    if (!CheckForOptimizationEvent()) {
      PublishSubmaps(submap_ids, additional_submaps_to_publish);
    }
  });
}

void Detector::RebuildTree() {
  std::unique_lock<std::mutex> lock(mutex_);
  std::vector<Value> rectangles;

  for (const auto& s_i : submaps_.submaps()) {
    const auto bounding_box_iter = bounding_boxes_.find(s_i.first);
    if (bounding_box_iter == bounding_boxes_.end()) {
      continue;
    }
    auto& bounding_box_info = bounding_box_iter->second;
    bounding_box_info.last_global_box = CalculateBoundingBox(*s_i.second);

    if (s_i.second->finished) {
      rectangles.emplace_back(
          std::make_pair(bounding_box_info.last_global_box, s_i.first));
    }
  }

  // Invokes rtree's packing constructor.
  rt_ = RTree{std::move(rectangles)};
}

}  // namespace frontier
