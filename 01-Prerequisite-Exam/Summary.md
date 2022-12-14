# Exam 
# Linux Scripting

```Bash

ARGS1 = $1
if [ $ARG1 == 'forward' ]; then 
	<bash command>

```


Reset position
## Permisison
``` bash
chmod 740 <file name>
# read = 4 , write = 2, execute = 1
# owner/ group/ all user
```

## Advanced Utilities
``` bash
ps faux
htop

ps faux | grep <file/process name>


kill <PID>
bg #resume execution of suspended program


```

- Understand linux scripting will allow you to write code to automate the launching of programs (on top of roslaunch files, and parameter definition)

``` bash
Variable=Value # No space as it will think that it is a command

```
# Python
```python
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

## Useful simple control
``` python
import rospy
from goemetry_msgs import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
```

