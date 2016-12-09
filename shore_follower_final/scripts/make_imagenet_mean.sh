#!/usr/bin/env sh
# Compute the mean image from the imagenet training lmdb
# N.B. this is available in data/ilsvrc12

FILE=choppy_trial02
EXAMPLE=/home/GTL/jloy/catkin_ws/src/shore_follower/$FILE
DATA=/home/GTL/jloy/catkin_ws/src/shore_follower/$FILE

TOOLS=/home/dream/caffe/build/tools

$TOOLS/compute_image_mean $EXAMPLE/followshore_train_lmdb \
  $DATA/imagenet_mean_fast.binaryproto

echo "Done."