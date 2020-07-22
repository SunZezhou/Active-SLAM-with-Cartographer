#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path

poses = []


def poseCb(data):
    global poses
    rospy.logwarn('recv')
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    poses.append(pose)


def node():
    global poses
    rospy.init_node('pose_path', anonymous=False)
    rospy.Subscriber('/odom_combined', PoseWithCovarianceStamped, poseCb)
    pub = rospy.Publisher('/odom_combined_path', Path, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        path = Path()
        path.header.frame_id = '/odom_combined'
        path.header.stamp = rospy.Time.now()
        path.poses = poses
        rospy.logwarn('publish')
        pub.publish(path)
        rate.sleep()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
