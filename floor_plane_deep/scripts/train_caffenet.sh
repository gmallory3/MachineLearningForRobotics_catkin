#!/usr/bin/env sh

TOOLS=/home/dream/caffe/build/tools
#TOOLS=/cs-share/pradalier/caffe/build/tools
$TOOLS/caffe train \
    --solver=/home/GTL/jloy/catkin_ws/src/floor_plane_deep/models/solver.prototxt
