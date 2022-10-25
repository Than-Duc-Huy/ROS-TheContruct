import moveit_commander #Namespace, is this a pacakge?
# Access to MoveGroupCommander, PlanningSceneInterface, RobotCommander
import copy
import rospy
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group_name = "robot_name"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

# Get planning (reference frame)
planning_frame = move_group.get_planning_frame()

# Name of eef_link
eff_link = move_group.get_end_effector_link()

# All Planning group in the robot
group_names = robot.get_group_names()

# Get current states
states = robot.get_current_state()

#==============================================
#========== Planning Joint Goal
joint_goal = move_group.get_current_joint_values() # Will get the correct number of elements 
joint_goal[0] = 0

move_group.go(joint_goal, wait=True)
move_group.stop() # Stop moving

#========== Planning Pose Goal
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.posiiotn.y = 0.1
pose_goal.position.z = 0.4

move_group.set_pose_target(pose_goal)

success = move_group.go(wait=True)  #Go based on the pose_target
move_group.stop() # Prevent residual movement

move_group.clear_pose_targets() # Clear so that it won't interfere with the next movement


#========== Planning Cartesian Paths
waypoints = []
scale = 10
wpose = move_group.get_current_pose().pose
wpose.position.z -= scale*0.1
wpose.position.y += scale*0.2
waypoints.append(copy.deepcopy(wpose))

(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0) # waypoints, eef_step, jump_threshold

## Plan and fraction are just planning


#========== Displaying Trajectory
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_state = robot.get_current_state()
display_trajectory.append(plan)

## Publish
display_trajectory_publisher.publish(display_trajectory)
move_group.execute(plan, wait=True)

#========== Executing a Plan
move_group.execute(plan, wait = True) # Wait for the plan to complete