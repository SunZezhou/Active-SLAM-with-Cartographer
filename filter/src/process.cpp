#include <ros/time.h>
#include "process.h"
#include "meanshift/meanShift.hpp"

enum PixelType
{
    PixelType_Unknown = 0,
    PixelType_Searched,
    PixelType_Pixel
};

struct PointCompare
{
    bool operator()(const cv::Point2i a, const cv::Point2i b) const
    {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    }
};

const std::vector<cv::Point2i> BFSList = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

FrontierArrayMap process(const FrontierArrayMap &frontiers, Map &globalMap, const tf2_ros::Buffer &tfBuffer, const std::string &base_frame)
{
    FrontierArrayMap new_frontiers;
    std::vector<Point> points;
    std::vector<std::pair<SubmapId, size_t>> pointIndex;
    points.reserve(frontiers.size());
    pointIndex.reserve(frontiers.size());
    for (const auto &sub : frontiers)
    {
        for (size_t p_i = 0; p_i < sub.second.size(); p_i++)
        {
            points.push_back({(float)sub.second[p_i][0], (float)sub.second[p_i][1]});
            pointIndex.push_back({sub.first, p_i});
        }
        new_frontiers[sub.first] = {};
    }
    if (points.size() == 0)
        return {};
    std::vector<Cluster> clusters = meanShift(points, 0.075);
    do
    {
        std::lock_guard<std::mutex> guard(globalMap.mtx);
        if (globalMap.resolution == 0)
            break;
        cv::Mat1b mask = cv::Mat1b::zeros(globalMap.wall.size());
        cv::Rect maskArea = cv::Rect2i{{0, 0}, mask.size()};
        auto translation = tfBuffer.lookupTransform(base_frame, "map", ros::Time(0)).transform.translation;
        cv::Point2i basePoint = (cv::Point2d{translation.x, translation.y} - globalMap.base) / globalMap.resolution;
        for (auto &c : clusters)
        {
            static auto split = [&mask, &maskArea, &basePoint, &globalMap](Cluster &cluster) -> std::vector<int> {
                std::map<cv::Point2i, size_t, PointCompare> pixelSet;
                std::vector<int> result;
                if (cluster.getSize() == 0)
                    return result;
                int i = 0;
                int minx = mask.cols, miny = mask.rows, maxx = 0, maxy = 0;
                for (const auto &p : cluster)
                {
                    cv::Point2i point = (cv::Point2d{p[0], p[1]} - globalMap.base) / globalMap.resolution;
                    if (minx > point.x)
                        minx = point.x;
                    if (miny > point.y)
                        miny = point.y;
                    if (maxx < point.x)
                        maxx = point.x;
                    if (maxy < point.y)
                        maxy = point.y;
                    if (!maskArea.contains(point))
                        continue;
                    pixelSet[point] = cluster.origin_index[i++];
                    mask.at<uint8_t>(point) = PixelType::PixelType_Pixel;
                }
                cv::Rect2i area = cv::Rect2i{cv::Point2i{minx, miny}, cv::Point2i{maxx + 1, maxy + 1}} & maskArea;
                while (pixelSet.size())
                { //sub-cluster
                    std::tuple<cv::Point2i, int, double> vPoint{basePoint, -1, std::numeric_limits<double>::max()};
                    std::queue<cv::Point2i> searchQueue;
                    searchQueue.push(pixelSet.begin()->first);
                    while (searchQueue.size())
                    { //BFS
                        cv::Point2i cur = searchQueue.front();
                        searchQueue.pop();
                        if (!area.contains(cur))
                            continue;
                        if (mask.at<uint8_t>(cur) == PixelType::PixelType_Searched)
                            continue;
                        if (mask.at<uint8_t>(cur) == PixelType_Pixel)
                        {
                            size_t index = pixelSet[cur];
                            pixelSet.erase(cur);
                            if ((globalMap.wall.at<uint8_t>(cur) != 0))
                                continue;
                            double distance = cv::norm(basePoint - cur);
                            if (distance < std::get<2>(vPoint))
                                vPoint = std::make_tuple(cur, index, distance);
                        }
                        if (globalMap.wall.at<uint8_t>(cur) != 0)
                            continue;
                        mask.at<uint8_t>(cur) = PixelType_Searched;
                        for (const auto &step : BFSList)
                        {
                            auto aim = cur + step;
                            if (area.contains(aim))
                                searchQueue.push(cur + step);
                        }
                    }
                    cv::Point2d p = std::get<0>(vPoint);
                    p = p * globalMap.resolution + globalMap.base;
                    result.push_back(std::get<1>(vPoint));
                }
                return result;
            };
            for (const auto &i : split(c))
            {
                if (i < 0)
                    continue;
                auto index = pointIndex[i];
                new_frontiers[index.first].push_back(frontiers.at(index.first)[index.second]);
            }
        }
    } while (0);
    return new_frontiers;
}