#!/usr/bin/env sh
# Compute the mean image from the imagenet training lmdb
# N.B. this is available in data/ilsvrc12

EXAMPLE=/home/GTL/jloy/catkin_ws/src/floor_plane_deep/data
DATA=/home/GTL/jloy/catkin_ws/src/floor_plane_deep/data

TOOLS=/home/dream/caffe/build/tools
#TOOLS=/cs-share/pradalier/caffe/build/tools

$TOOLS/compute_image_mean $DATA/floorplane_train_lmdb \
  $DATA/imagenet_mean.binaryproto

echo "Done."
