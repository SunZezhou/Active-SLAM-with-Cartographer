#include <ros/ros.h>
#include <mutex>
#include <map>
#include <thread>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <cartographer_ros_msgs/SubmapList.h>
// #include <cartographer_ros_msgs/Frontier.h>
#include <filter/PointArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include "process.h"

int wall_threshold = 100;
// cv::Mat StructElem;

// std::mutex mapMutex;
Map globalMap;

std::mutex frontierMutex;
FrontierArrayMap updateFrontiers;

void mapCallback(const nav_msgs::OccupancyGrid &map)
{
    cv::Mat_<int8_t> mapRaw(map.data);
    mapRaw = mapRaw.reshape(1, map.info.height);
    {
        std::lock_guard<std::mutex> guard(globalMap.mtx);
        // globalMap.land = mapRaw == 0;
        globalMap.wall = mapRaw >= wall_threshold;
        globalMap.base = {map.info.origin.position.x, map.info.origin.position.y};
        globalMap.resolution = map.info.resolution;
    }
    return;
}

void frontierCallback(const visualization_msgs::MarkerArrayConstPtr &msg)
{
    // ROS_WARN_STREAM("callback");
    std::vector<SubmapId> index;
    FrontierArrayMap result;
    for (const auto &marker : msg->markers)
    {
        int tr_id, sm_id;
        sscanf(marker.ns.c_str(), "Trajectory %d, submap %d", &tr_id, &sm_id);
        SubmapId id = {tr_id, sm_id};
        result[id] = {};
        std::transform(marker.points.cbegin(), marker.points.cend(), std::back_inserter(result[id]), [&](const geometry_msgs::Point &p) {
            return std::array<double, 2>{p.x, p.y};
        });
        index.push_back(id);
    }
    {
        std::lock_guard<std::mutex> guard(frontierMutex);
        for (auto i : index)
        {
            updateFrontiers[i].swap(result[i]);
        }
    }
    return;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "filter");
    ros::NodeHandle n;
    int tolerant_radius = 3;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string mapTopic = "map", submapTopic = "submap_list", frontierTopic = "frontier_marker", aimTopic = "filtered_points", centroidsTopic = "centroids", base_frame = "base_footprint";
    n.param("map", mapTopic, mapTopic);
    n.param("frontier", frontierTopic, frontierTopic);
    n.param("aim", aimTopic, aimTopic);
    n.param("centroids", centroidsTopic, centroidsTopic);
    n.param("radius", tolerant_radius, tolerant_radius);
    n.param("wall_threshold", wall_threshold, wall_threshold);
    n.param("base_frame", base_frame, base_frame);
    // StructElem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * tolerant_radius + 1, 2 * tolerant_radius + 1), cv::Point(tolerant_radius, tolerant_radius));
    ros::Subscriber mapSub = n.subscribe(mapTopic, 1, mapCallback);
    ros::Subscriber frontierSub = n.subscribe(frontierTopic, 1, frontierCallback);
    ros::Publisher aimPub = n.advertise<filter::PointArray>(aimTopic, 1);
    ros::Publisher centroidsPub = n.advertise<visualization_msgs::MarkerArray>(centroidsTopic, 1);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
#if 1
    std::thread thread([&]() {
        ros::Rate rate(10);
        FrontierArrayMap globalFrontiers;
        // ros::Time total = ros::Time(0);
        // int count = 0;
        while (!ros::isShuttingDown())
        {
            std::map<SubmapId, std::vector<geometry_msgs::Point>> result;
            {
                FrontierArrayMap currentUpdateFrontiers = {};
                {
                    std::lock_guard<std::mutex> guard_frontier(frontierMutex);
                    updateFrontiers.swap(currentUpdateFrontiers);
                }
                if (currentUpdateFrontiers.size() == 0)
                    continue;
                // ros::Time t = ros::Time::now();
                FrontierArrayMap newFrontiers = process(currentUpdateFrontiers, globalMap, tfBuffer, base_frame);
                // uint64_t dt0, dt1;
                // std::tie(newFrontiers, dt0, dt1) = process(currentUpdateFrontiers, globalMap, tfBuffer, base_frame);
                // total += ros::Time::now() - t;
                // ROS_WARN_STREAM(count << "," << dt0 << "," << dt1);
                // count++;
                for (auto &it : newFrontiers)
                {
                    result[it.first] = {};
                    for (const auto &p : it.second)
                    {
                        geometry_msgs::Point point;
                        point.x = p[0];
                        point.y = p[1];
                        result[it.first].push_back(point);
                    }
                    globalFrontiers[it.first].swap(it.second);
                }
            }
            {
                visualization_msgs::MarkerArray mArr;
                filter::PointArray pa;
                for (auto &r : result)
                { // TODO:
                    visualization_msgs::Marker m;
                    m.header.frame_id = "/map";
                    m.ns = "centroid-marker" + std::to_string(r.first.trajectory_id) + "," + std::to_string(r.first.submap_index);
                    m.type = visualization_msgs::Marker::POINTS;
                    m.action = visualization_msgs::Marker::MODIFY;
                    m.pose.orientation.w = 1;
                    m.scale.x = 0.2;
                    m.scale.y = 0.2;
                    // m.scale.x = 0.05;
                    // m.scale.y = 0.05;
                    m.scale.z = 1;
                    m.color.r = 1;
                    m.color.g = 0;
                    m.color.b = 0;
                    m.color.a = 1;
                    m.points = r.second;
                    mArr.markers.push_back(m);
                    //
                    std::copy(r.second.begin(), r.second.end(), std::back_inserter(pa.points));
                }
                centroidsPub.publish(mArr);
                aimPub.publish(pa);
            }
        }
    });
#endif
    ros::spin();
    return 0;
}
