#!/usr/bin/env python  
import roslib
roslib.load_manifest('vicon_tf_listener')
roslib.load_manifest('vicon_bridge')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('vicon_tf_listener')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'vicon/ladyheadSat/root')

    head_vel = rospy.Publisher('vicon/ladyheadSat/root', geometry_msgs.msg.Twist, queue_size=1000)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/vicon/ladyheadSat', '/root', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        head_vel.publish(cmd)

        rate.sleep()

        #roslaunch vicon_tf_listener listener_demo.launch