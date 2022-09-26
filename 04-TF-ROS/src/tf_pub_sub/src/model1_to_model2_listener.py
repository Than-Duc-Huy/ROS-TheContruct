#!/usr/bin/env python
import sys
import rospy
import math
import tf
import geometry_msgs.msg
import time

class Model1ToModel2Listener(object):

    def __init__(self, follower_model_name, model_to_be_followed_name):

        self._listener = tf.TransformListener()
        # We ad this sleep to allow the listener time to be initialised
        # Otherwise the lookupTransform could give not found frame issue.
        time.sleep(1)

        self._follower_model_name = follower_model_name
        self._model_to_be_followed_name = model_to_be_followed_name

        rospy.loginfo("follower_model_name="+str(self._follower_model_name))
        rospy.loginfo("model_to_be_followed_name=" +
                      str(self._model_to_be_followed_name))

        self._turtle_vel = rospy.Publisher(
            self._follower_model_name+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

        self._rate = rospy.Rate(10.0)
        self._ctrl_c = False

        self._follower_model_frame = "/"+self._follower_model_name
        self._model_to_be_followed_frame = "/"+self._model_to_be_followed_name

        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        print("shutdown time! Stop the robot")
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self._turtle_vel.publish(cmd)
        self._ctrl_c = True

    def start_loop(self):

        while not self._ctrl_c:
            
            # This is another option to guarantee that we wait until teh transofrms are ready
            # This avoids also teh issue of not finding the frame by time out
            # self._listener.waitForTransform(
            #    self._follower_model_frame, self._model_to_be_followed_frame, rospy.Time(0), rospy.Duration(0.5))
            
            try:
                (trans, rot) = self._listener.lookupTransform(
                    self._follower_model_frame, self._model_to_be_followed_frame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self._turtle_vel.publish(cmd)
            self._rate.sleep()


def main():
    rospy.init_node('tf_listener_turtle')

    if len(sys.argv) < 3:
        print("usage: rosrun tf_exercises_pkg turtle_tf_listener.py follower_model_name model_to_be_followed_name")
    else:
        follower_model_name = sys.argv[1]
        model_to_be_followed_name = sys.argv[2]

        listener_obj = Model1ToModel2Listener(
            follower_model_name, model_to_be_followed_name)

        try:
            listener_obj.start_loop()
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    main()
