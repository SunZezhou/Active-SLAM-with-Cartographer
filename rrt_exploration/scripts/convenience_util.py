#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64
from cartographer_ros_msgs.msg import PoseConvenience

totalPub = None


def pcCb(data):
    global totalPub
    rospy.loginfo('publish')
    # std::log(2 * M_PI * M_E) * (3.0 / 2.0) + std::log(covMat.determinant()) / 2
    totalPub.publish(sum(data.convs) * 1e24)


def node():
    global totalPub
    rospy.init_node('conv_total', anonymous=False)
    totalPub = rospy.Publisher('/conv_total', Float64, queue_size=1)
    rospy.Subscriber('/pose_convenience', PoseConvenience, pcCb)
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
