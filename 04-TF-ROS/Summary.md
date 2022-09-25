# TF Theory
```bash
rosrun tf2_tools view_frames.py  # pdf of TF hierarchy 
rosrun rqt_tf_tree rqt_tf_tree # rqt of TF hierarchy
```


- `/tf` only publishes the direct TF (wrt to world), not to each other
```bash
rostopic echo -n1 /tf # -n1 to get only 1 message

rosrun tf tf_echo <source_frame> <target_frame> # Get the relative transformation
```

# TF Sub & Pub
## Publisher, Broadcaster
```python
import tf
from turtle_tf_3d.get_model_gazebo_pose import GazeboModel

model = GazeboModel(["turtle1","turtle2"])
broadcaster_tf = tf.TransformBroadcaster() #TF broadcaster / publisher instance


pose = model.get_model_pose("turtle1")
pose_msg = pose

broadcaster_tf.sendTransform((pose_msg.position.x, pose_msg.position.y, pose_msg.position.z),
                            (pose_msg.orientation.x, pose_msg.orientation.y,
                            pose_msg.orientation.z, pose_msg.orientation.w),
                            rospy.Time.now(),
                            robot_name="turtle1", # Can create new frame if none present
                            reference_frame_data = "/world") # publish the TF, time is needed in,

```

- Note
```bash
rosservice call /gazebo/get_world_properties "{}" #get world properties from Gazebo
```

## Subscriber
```python
import tf

listener = tf.TransformListener() # TF Listen
time.sleep(1) # Allow listener to be initialized

listener.waitForTransform(   # Another way to wait
               target_frame = follower_model_frame, 
               source_frame = smodel_to_be_followed_frame, 
               time = rospy.Time(0),
               timeout = rospy.Duration(0.5))


# Lookup, query the relative transformation
listener.lookupTransform(target_frame, source_frame, frame_time = rospy.Time(0)) # For the time interpolation to work. rospy.Time(0)  Most recent common time


# or insert a waitfortransform
try:
    frame_time = rospy.Time.now()
    listener.waitForTransform(target_frame, 
                              source_frame,
                              frame_time=rospy.Time.now(),  #Or rospy.Time(0)
                              rospy.Duration(0.5))
            
    (trans, rot) = self._listener.lookupTransform(target_frame, source_frame, frame_time = rospy.Time(0))
    
except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    print(str(e))
    continue

```

### Quaternion to Euler
```python
tf.transforms.quaternion_from_euler(1,2,3)
tf.transforms.euler_from_quaternion(1,2,3,4)
```

# Robot State Publisher