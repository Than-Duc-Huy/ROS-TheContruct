
#!/bin/bash

ARG=$1

if [ "$ARG" == "small_square" ]; then
    rosrun linux_exam small_square.py
elif [ "$ARG" == "medium_square" ]; then
    rosrun linux_exam medium_square.py
elif [ "$ARG" == "big_square" ]; then
    rosrun linux_exam big_square.py

fi