#!bin/bash

cd ~/catkin_ws/src/linux_exam/this/is/my/linux/exam/
rm *
touch exam1.py
touch exam2.py
touch exam3.py
# 4 read 2 write 1 execute

chmod 764 exam1.py
chmod 501 exam2.py
chmod 241 exam3.py

