#!/usr/bin/env sh

/home/gpu_user/caffe/build/tools/caffe train \
    --solver=solver_fast.prototxt 2>&1 | tee log.txt