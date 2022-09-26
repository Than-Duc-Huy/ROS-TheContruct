#! /usr/bin/env python
import rospy
import time
import tf
from turtle_tf_3d.get_model_gazebo_pose import GazeboModel


class MultiTfBroadcast(object):

    def __init__(self, robot_name_list, loop_rate=5.0):

        self._robot_name_list = robot_name_list
        self._gazebo_model_object = GazeboModel(self._robot_name_list)

        # hz
        self._rate = rospy.Rate(loop_rate)

        # We start the Tf broadcaster
        self._broad_caster_tf = tf.TransformBroadcaster()

        # Leave enough time to be sure the Gazebo Model logs have finished
        time.sleep(1)
        rospy.loginfo("Ready..Starting to Publish TF data now...")

    def handle_turtle_pose(self, pose_msg, robot_name, reference_frame_data="/world"):

        self._broad_caster_tf.sendTransform((pose_msg.position.x, pose_msg.position.y, pose_msg.position.z),
                                            (pose_msg.orientation.x, pose_msg.orientation.y,
                                             pose_msg.orientation.z, pose_msg.orientation.w),
                                            rospy.Time.now(),
                                            robot_name,
                                            reference_frame_data)

    def start_loop(self):

        while not rospy.is_shutdown():
            for robot_name in self._robot_name_list:
                pose_now = self._gazebo_model_object.get_model_pose(robot_name)
                if not pose_now:
                    robot_name_msg = "The " + \
                        str(robot_name) + \
                        "'s Pose is not yet available...Please try again later"
                    rospy.loginfo(robot_name_msg)
                else:
                    self.handle_turtle_pose(pose_now, robot_name)
            self._rate.sleep()


def main():

    rospy.init_node('publisher_of_tf_node', anonymous=True)
    multi_tf_broadcast_obj = MultiTfBroadcast(
        robot_name_list=["turtle1", "turtle2", "coke_can"])

    try:
        multi_tf_broadcast_obj.start_loop()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
