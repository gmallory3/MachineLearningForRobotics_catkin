#!/usr/bin/env sh

/home/dream/caffe/build/tools/caffe train \
    --solver=/home/GTL/jloy/catkin_ws/src/shore_follower/models/solver_fast.prototxt 2>&1 | tee log.txt
