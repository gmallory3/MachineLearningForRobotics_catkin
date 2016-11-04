#!/usr/bin/env sh

TOOLS=/home/dream/caffe/build/tools
#TOOLS=/cs-share/pradalier/caffe/build/tools
$TOOLS/caffe train \
    --solver=solver.prototxt
