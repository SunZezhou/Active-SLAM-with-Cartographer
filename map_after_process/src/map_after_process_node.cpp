#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/opencv.hpp>

const std::string MapSubTopic = "/map";
const std::string MapPubTopic = "/map_after_process";
// const std::string ImageLogPath = "/mnt/d/image-log/";

const int8_t LAND = 0;
const int8_t WALL = 100;
const int8_t UNKNOWN = -1;

int DILATE_RADIUS = 10;

ros::Publisher mapPub;
std::vector<int8_t> LookupTableImpl(256);
// int8_t* LookupTable = &LookupTableImpl[128];

void mapCallback(const nav_msgs::OccupancyGrid &subData)
{
    nav_msgs::OccupancyGrid pubData;
    pubData.header = subData.header;
    pubData.info = subData.info;
    cv::Mat_<int8_t> map(subData.data);
    map = map.reshape(1, subData.info.height);
    // std::vector<int8_t> data(subData.data.size());
    // std::transform(subData.data.cbegin(), subData.data.cend(), data.begin(), [](const int8_t& d) -> int8_t {
    //     return LookupTable[d];
    // });
    cv::LUT(map, LookupTableImpl, map);
    cv::Mat wall = map == 100;
    cv::dilate(wall, wall, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * DILATE_RADIUS + 1, 2 * DILATE_RADIUS + 1), cv::Point(DILATE_RADIUS, DILATE_RADIUS)));
    wall &= 1;
    wall.convertTo(wall, CV_8S);
    wall = -wall;
    map &= ~wall;
    map |= wall & 100;
    std::vector<int8_t> data;
    data.assign(map.data, map.data + map.total());
    pubData.data = data;
    mapPub.publish(pubData);
#if 0
    std::string filename = std::to_string(subData.info.map_load_time.toSec());
    cv::Mat_<int8_t> map(data);
    map = map.reshape(1, subData.info.height);
    try
    {
        cv::imwrite(ImageLogPath + filename + ".png", map);
    }
    catch (cv::Exception e)
    {
        ROS_WARN_STREAM(e.what());
    }
#endif
    return;
}

int main(int argc, char *argv[])
{
    for (int i = 0; i < 256; i++)
    {
        if (i >= 51 && i <= 100)
            LookupTableImpl[i] = WALL;
        // else if (i <= 50 && i >= 0)
        //     LookupTableImpl[i] = LAND;
        else
            LookupTableImpl[i] = UNKNOWN;
    }
    ros::init(argc, argv, "map_after_process");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string mapSubTopic, mapPubTopic;
    n.param("map_sub_topic", mapSubTopic, MapSubTopic);
    n.param("map_pub_topic", mapPubTopic, MapPubTopic);
    n.param("radius", DILATE_RADIUS, DILATE_RADIUS);
    ROS_INFO_STREAM(DILATE_RADIUS);
    ros::Subscriber mapSub = n.subscribe(mapSubTopic, 1, mapCallback);
    mapPub = n.advertise<nav_msgs::OccupancyGrid>(mapPubTopic, 1);
    ros::spin();
    return 0;
}