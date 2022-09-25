# ROS Action 
- Asynchronous calls to services. Create 5 topics
  - cancel
  - feedback
  - goal
  - result
  - status
- Information exchange topics: goal, result, feedback

- Status 
  - Pending: 0
  - Active: 1
  - Done: 2
  - Warn: 3
  - Error: 4

## Client
- After build/make: Message.action will be built into
  - MessageAction
  - MessageGoal
  - MessageResult
  - MessageFeedback
  - in package_name.msg (the topic messages)
- The action message is the file `.action` but after compiling, it will be in package.msg folder when import to python
- Action Message has 3 parts
  - goal
  - result
  - feedback

### Python Example Code
```python
import actionlib
import action_package.msg import ActionAction, ActionGoal, ActionResult, ActionFeedback

def callback(feedback):

#Initialize
rospy.init_node("Client Action node")
action_server_name = "/action_server"
client = actionlib.SimpleActionClient(action_server_name, ActionAction)
client.wait_for_server()

#Set and send goal
goal = ActionGoal()
goal.variable = value

client.send_goal(goal, callback) # Callback function

#Get state & Do something else
state = client.get_state()
while state < 2: # Is not done
    #Do something else

#Result is after the action
done = client.wait_for_result() # Return boolean if the result is sent

```
### GUI
```bash
rosrun actionlib_tools axclient.py /action_server
```
## Server
### Python Example
```python
import actionlib
from Action.msg import ActionFeedback, ActionResult, ActionAction

class ActionServer():
  _feedback = ActionFeedback()
  _result = ActionResult()

  def __init__(self):
    self._as_ = actionlib.SimpleActionServer("action_server", ActionAction, self.goalCallback, auto_start = False)
    self._as_.start() #Start on class call

  def goalCallback(self,goal): # SimpleActionServer will pass in the goal
    success = True

    # Do things with goal.variable1, goal.variable2 as defined in the message
    self._as.publish_feedback(self._feedback) ## PUBLISH FEEDBACK
    ## Check preemp cancelation
    if self._as.is_preempt_requested():
        rospy.loginfo("Goal cancelled")
        self._as.set_preempted() # SET PREEMPT
        success = False
        break
    ## Check success
    if success:
        self._result.variable1 = self._feedback.variable1 #as defined
        rospy.loginfo("Success")
        self._as.set_succeeded(self._result) # SET SUCCEEDED
if __name__ == '__main__':
  rospy.init_node("ActionServer")
  ActionServer()
  rospy.spin() # Runs forever
```

### Note
- From `Action.action`, after compilation
  - Python: ActionAction, ActionFeedback, ActionResult, ActionGoal
  - Message type: ActionActionFeedback, ActionActionResult, ActionActionGoal

## Message
- Action file
```
#goal
package/type variable
---
#result
package/type variable
---
#feedback
package/type variable

```

### CMake
- `actionlib_msgs` is a must
```cmake
find_package(catkin REQUIRED COMPONENTS
      # your packages are listed here
      std_msgs
      actionlib_msgs
)
add_action_files(
      FILES
      Name.action
)
generate_messages(
      DEPENDENCIES
      std_msgs
      actionlib_msgs 
      # Your packages go here
)
catkin_package(
      CATKIN_DEPENDS
      rospy
      # Your package dependencies go here
)
```
### package.xml
- `actionlib_msgs` is a must
```xml
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend> <!--Needed to compile the custom action msgs based on std_msgs-->
  <build_export_depend>actionlib</build_export_depend>
  <build_export_depend>actionlib_msgs</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <exec_depend>actionlib</exec_depend> <!--Needed in runtime to run action-->
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>rospy</exec_depend>  <!--Needed to run ROS python node-->
```