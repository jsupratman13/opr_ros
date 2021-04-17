#!/bin/sh
THRESHOLD=-3
WEIGHT="wl_npz/wl_2layer_20190622_142021.npz"
cd ~/robocup/for2050/src/vision/detector/whiteline_detection_deep
python chainer_socket.py --thre ${THRESHOLD} --weight ${WEIGHT}
