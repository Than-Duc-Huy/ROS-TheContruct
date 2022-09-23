# Linux Scripting
- Understand linux scripting will allow you to write code to automate the launching of programs (on top of roslaunch files, and parameter definition)

```
Variable=Value # No space as it will think that it is a command
```
# Python
```
from sensor_msgs.msg import LaserScan

# Subscribe and get LaserScan in the Class definition
self.laser_subscriber = rospy.Subscriber('/kobuki/laser/'scan', LaserScan, self.laser_callback)

def laser_callback(self.msg):
    self.laser_msg = msg


# Call Data
self.laser_msg.ranges # Array of float 
self.laser_msg.ranges[360] # Data at the 360 position 
 
# Deal with infinite values
float("inf")
float("-inf")

# For C++ 
std::numeric_limits<sensor_msgs::Range::_range_type>::infinity()
std::numeric_limits<float>::infinity()
```

- Use OOP