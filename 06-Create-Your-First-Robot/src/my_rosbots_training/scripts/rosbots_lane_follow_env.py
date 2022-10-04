import rospy
import numpy
import time
from scipy.interpolate import interp1d
from gym import spaces
import rosbots_env
from gym.envs.registration import register

import sys
import rospkg
rospack = rospkg.RosPack()
detection_pkg_path = rospack.get_path('line_follower')
sys.path.insert(0, detection_pkg_path + "/scripts")
from line_follow import LineFollower
# The path is __init__.py of openai_ros, where we import the TurtleBot2MazeEnv directly
timestep_limit_per_episode = 20000  # Can be any Value

register(
    id='RosBotsEnv-v0',
    entry_point='rosbots_lane_follow_env:RosBotsEnv',
    timestep_limit=timestep_limit_per_episode,
)


class RosBotsEnv(rosbots_env.RosBotsEnv):
    def __init__(self):
        """
        This Task Env is designed for allowing the ROSbots to navigate following lanes. Using the robot's visual sensor.
        It will learn how to move around the town without goin off-course.
        """
        # Only variable needed to be set here
        number_actions = rospy.get_param('/rosbots/n_actions')
        self.action_space = spaces.Discrete(number_actions)

        # We set the reward range, which is not compulsory but here we do it.
        self.reward_range = (-numpy.inf, numpy.inf)

        # number_observations = rospy.get_param('/turtlebot2/n_observations')
        """
        We set the Observation space
        cube_observations = [
            round([camera_feed])
        ]
        """

        # Actions and Observations
        self.linear_forward_speed = rospy.get_param(
            '/rosbots/linear_forward_speed')
        self.linear_turn_speed = rospy.get_param('/rosbots/linear_turn_speed')
        self.angular_speed = rospy.get_param('/rosbots/angular_speed')
        self.init_linear_forward_speed = rospy.get_param(
            '/rosbots/init_linear_forward_speed')
        self.init_linear_turn_speed = rospy.get_param(
            '/rosbots/init_linear_turn_speed')

        # Control Parameters
        self.dMax = rospy.get_param('/rosbots/max_lane_offset')
        self.dSafe = rospy.get_param('/rosbots/safe_lane_offset')
        self.look_ahead_distance = rospy.get_param(
            '/rosbots/look_ahead_distance')

        # Rewards
        self.follow_lane_reward = rospy.get_param(
            "/rosbots/follow_lane_reward")
        self.left_right_reward = rospy.get_param("/rosbots/left_right_reward")
        self.veer_off_reward = rospy.get_param("/rosbots/veer_off_reward")
        self.end_episode_points = rospy.get_param(
            "/rosbots/end_episode_points")

        self.action_taken = None

        self.cumulated_steps = 0.0

        # Here we will add any init functions prior to starting the MyRobotEnv
        super(RosBotsEnv, self).__init__()

        # Instantiate the image processing class
        self.line_follow = LineFollower()

        # low = numpy.array([0.0])
        # high = numpy.array([19200.0])

        # We only use two integers #CHANGED THE OBSERVATION SPACE
        # self.observation_space = spaces.Box(low, high) #observation space boundary
        self.observation_space = spaces.Box(0, 255, shape=(80, 80, 3))

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>" +
                       str(self.observation_space))

    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.line_follow.clean_up()
        self.move_base(self.init_linear_forward_speed,
                       self.init_linear_turn_speed,
                       epsilon=0.05,
                       update_rate=10,
                       use_offset=False)  # Do not check for lane

        return True

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # For Info Purposes
        self.cumulated_reward = 0.0
        # Set to false Done, because its calculated asyncronously
        self._episode_done = False

        # We wait a small ammount of time to start everything because in very fast resets, laser scan values are sluggish
        # and sometimes still have values from the prior position that triguered the done.
        time.sleep(0.2)

    def _set_action(self, action):
        """
        This set action will Set the linear and angular speed of the ROSbots
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """

        rospy.logdebug("Start Set Action ==>"+str(action))
        self.action_taken = action
        # We convert the actions to speed movements to send to the parent class CubeSingleDiskEnv
        if action == 0:  # FORWARD
            linear_speed = self.linear_forward_speed
            angular_speed = 0.0
            self.last_action = "FORWARD"
        elif action == 1:  # LEFT
            linear_speed = self.linear_turn_speed
            angular_speed = self.angular_speed
            self.last_action = "TURN_LEFT"
        elif action == 2:  # RIGHT
            linear_speed = self.linear_turn_speed
            angular_speed = -1*self.angular_speed
            self.last_action = "TURN_RIGHT"

        # We tell ROSbots the linear and angular speed to set to execute
        self.move_base(linear_speed,
                       angular_speed,
                       epsilon=0.05,
                       update_rate=10)

        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robots observations
        To know which Variables we have acces to, we need to read the
        RosBotsEnv API DOCS
        :return:
        """
        rospy.logdebug("Start Get Observation ==>")

        image_1 = self.get_camera_rgb_image_raw()
        image_2 = self.get_camera2_rgb_image_raw()

        discretized_observations = self.discretize_observation(
            image_1, image_2)
        # rospy.logerr("Observations==>"+str(discretized_observations))
        rospy.logdebug("END Get Observation ==>")
        # rospy.logerr(discretized_observations.shape)
        return discretized_observations

    def _is_done(self, observations):

        if self._episode_done:
            rospy.logerr("ROSBots veered OFF-COURSE ==>")
            return self._episode_done
        else:
            rospy.logerr("ROSBots is Ok ==>")

        return self._episode_done

    def _compute_reward(self, observations, done):
        """
        Our reward system will be based on how far or near the robot's forward with respect to the lane
        E.g 0(centered) = 100 and 1(off-course) = -10
        """

        if not done:
            if (self.action_taken != None):
                if(self.action_taken == 0):
                    reward = self.follow_lane_reward  # Favour going foward to turning left/right
                else:
                    reward = self.left_right_reward
        else:
            reward = -1*self.follow_lane_reward

        rospy.logdebug("reward=" + str(reward))
        self.cumulated_reward += reward
        rospy.logdebug("Cumulated_reward=" + str(self.cumulated_reward))
        self.cumulated_steps += 1
        rospy.logdebug("Cumulated_steps=" + str(self.cumulated_steps))

        return reward

    # Internal TaskEnv Methods

    def discretize_observation(self, image_data,  image2_data):
        """
        Discards all the laser readings that are not multiple in index of look_ahead_distance
        value.
        """

        # discretized_ranges = []

        # 1. Process image
        # 2. Reduce the 'cur_offset' to 2dp
        # 3. Create a mapping for the ranging aspect

        # Results returned is a tuple (detection:boolean, angular_z:float, observation_image: numpy.array())
        self.detection_results = self.line_follow.process_data(observation_image=image_data,
                                                               detection_image=image2_data,
                                                               rows_to_watch=self.look_ahead_distance,
                                                               show_window=True)

        self.cur_offset = float(
            "%.4f" % self.detection_results[1])  # 0.5  and 1.52

        no_line_detected = self.detection_results[0]

        if ((self.cur_offset < self.dSafe) or (self.cur_offset > self.dMax) or no_line_detected):
            self._episode_done = True
        else:
            self._episode_done = False

        # return the image converted to numpy array as observation data
        return self.detection_results[2]
