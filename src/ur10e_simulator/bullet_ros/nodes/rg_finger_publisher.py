#!/usr/bin/env python3
"""
@author Lukas Rustler
"""
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf


def send_tf():
    rospy.init_node('finger_tf_broadcaster')
    rate = rospy.Rate(100)
    ls = tf.TransformListener()
    joint_pub = rospy.Publisher("/jaws/joint_states", JointState, queue_size=1)
    while not rospy.is_shutdown():
        while True:
            try:
                (trans, _) = ls.lookupTransform('/tool0', '/left_inner_finger', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        while True:
            try:
                (_, rot) = ls.lookupTransform('/base_link', '/tool0', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        joint_trans = trans[2] + 65/1000

        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = ["jaws_joint"]

        js.position = [joint_trans]
        js.velocity = [0.0]
        js.effort = [0.0]
        joint_pub.publish(js)
        rate.sleep()


if __name__ == "__main__":
    try:
        send_tf()
    except rospy.ROSInterruptException:
        pass