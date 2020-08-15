#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <string>

ros::Subscriber sub_turtle_pose;
ros::Subscriber sub_goal_pose;
ros::Publisher pub_map_odom;
tf::TransformBroadcaster* tf_map_odom;
tf::TransformListener* tf_listener;


Eigen::Matrix4f current_robot_pose;
Eigen::Matrix4f goal_pose;


nav_msgs::Odometry map_to_odom;

tf::Transform t;

// 接收机器人当前姿态
void callback1(const geometry_msgs::PoseStampedConstPtr &msg)
{
    float x, y, z, qx, qy, qz, qw;
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    z = msg->pose.position.z;

    qx = msg->pose.orientation.x;
    qy = msg->pose.orientation.y;
    qz = msg->pose.orientation.z;
    qw = msg->pose.orientation.w;

    current_robot_pose(0, 3) = x;
    current_robot_pose(1, 3) = y;
    current_robot_pose(2, 3) = z;

    // Eigen::Quaternionf q(qw, qx, qy, qz);
    // q.normalize();
    // current_robot_pose.block(0,0, 3, 3) = q.toRotationMatrix();


    Eigen::AngleAxisf rot = Eigen::AngleAxisf(qw * M_PI / 180.0, Eigen::Vector3f::UnitZ());
    current_robot_pose.block(0,0,3,3) = rot.toRotationMatrix();

    tf::StampedTransform transform;
    if(tf_listener->waitForTransform("odom", "base_footprint", ros::Time(0), ros::Duration(0.01)))
    {
        tf_listener->lookupTransform("odom", "base_footprint", ros::Time(0), transform);
        geometry_msgs::TransformStamped trans;
        tf::transformStampedTFToMsg(transform, trans);

        Eigen::Matrix4f Todom_base = Eigen::Matrix4f::Identity();
        Todom_base(0,3) = trans.transform.translation.x;
        Todom_base(1,3) = trans.transform.translation.y;
        Todom_base(2,3) = trans.transform.translation.z;

        Eigen::Quaternionf qq(trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z);
        qq.normalize();
        Todom_base.block(0, 0, 3, 3) = qq.toRotationMatrix();
//current_robot_pose.block(0,0,3,3) = qq.toRotationMatrix();

        Eigen::Matrix4f Tw_odom = current_robot_pose * Todom_base.inverse();

        
        map_to_odom.header.stamp = msg->header.stamp;
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";
        map_to_odom.pose.pose.position.x = Tw_odom(0, 3);
        map_to_odom.pose.pose.position.y = Tw_odom(1, 3);
        map_to_odom.pose.pose.position.z = Tw_odom(2, 3);

        Eigen::Matrix3f rota = Tw_odom.block(0,0,3,3);
        Eigen::Quaternionf qqq(rota);
        qqq.normalize();

        map_to_odom.pose.pose.orientation.x = qqq.x();
        map_to_odom.pose.pose.orientation.y = qqq.y();
        map_to_odom.pose.pose.orientation.z = qqq.z();
        map_to_odom.pose.pose.orientation.w = qqq.w();

        
        t.setOrigin(tf::Vector3(Tw_odom(0, 3), Tw_odom(1, 3), Tw_odom(2, 3)));
        t.setRotation(tf::Quaternion(qqq.x(), qqq.y(), qqq.z(),qqq.w()));

        ROS_INFO_STREAM("caught robot pose");
    }
    else
    ROS_INFO_STREAM("waiting robot pose");

}


void callback2(const geometry_msgs::PoseStampedConstPtr &msg)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_node");

    ros::NodeHandle nh("~");

    tf_map_odom = new tf::TransformBroadcaster();
    tf_listener = new tf::TransformListener();

    sub_turtle_pose = nh.subscribe("/move_base_simple/currentPose", 1, callback1);
    sub_goal_pose = nh.subscribe("/wy_topic_2", 1, callback2);
    pub_map_odom = nh.advertise<nav_msgs::Odometry>("map_to_odom", 1);

    current_robot_pose = Eigen::Matrix4f::Identity();
    goal_pose = Eigen::Matrix4f::Identity();

    map_to_odom.header.stamp = ros::Time::now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.pose.pose.position.x = 0;
    map_to_odom.pose.pose.position.y = 0;
    map_to_odom.pose.pose.position.z = 0;

    map_to_odom.pose.pose.orientation.x = 0;
    map_to_odom.pose.pose.orientation.y = 0;
    map_to_odom.pose.pose.orientation.z = 0;
    map_to_odom.pose.pose.orientation.w = 1;


    t.setOrigin(tf::Vector3(0,0,0));
    t.setRotation(tf::Quaternion(0,0,0,1));

    ros::Rate r(100);
    while(ros::ok())
    {
        ros::spinOnce();

        tf_map_odom->sendTransform(tf::StampedTransform(t, ros::Time::now(), "map", "odom"));

        pub_map_odom.publish(map_to_odom);

        r.sleep();

    }

    ros::spin();
}
