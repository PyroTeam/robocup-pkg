#!/bin/bash

## Script permettant de générer un message nav_msgs/OccupancyGrid représentant la map de la Robocup LLSF 2014

# rostopic pub /grid nav_msgs/OccupancyGrid "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# info:
#   map_load_time: {secs: 0, nsecs: 0}
#   resolution: 0.56
#   width: 19
#   height: 9
#   origin:
#     position: {x: 5.32, y: 0.28, z: 0.0}
#     orientation: {x: 0, y: 1, z: 0, w: 0.0}
# data: [
# 000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,
# 000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,
# 000,000,100,000,100,000,100,000,100,000,100,000,100,000,100,000,100,000,000,
# 000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,
# 000,000,100,000,000,000,100,000,100,000,100,000,100,000,000,000,100,000,000,
# 000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,
# 000,000,000,000,100,000,100,000,000,000,000,000,100,000,100,000,000,000,000,
# 000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,
# 100,000,100,000,100,000,000,000,100,000,100,000,000,000,100,000,100,000,100]"


rostopic pub /grid nav_msgs/OccupancyGrid "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
info:
  map_load_time: {secs: 0, nsecs: 0}
  resolution: 0.56
  width: 19
  height: 9
  origin:
    position: {x: 5.32, y: 0.28, z: 0.0}
    orientation: {x: 0, y: 1, z: 0, w: 0.0}
data: [
000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,
000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,
000,000,100,000,100,000,100,000,100,000,100,000,100,000,100,000,100,000,000,
000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,
000,000,100,000,000,000,100,000,100,000,100,000,100,000,000,000,100,000,000,
000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,
000,000,000,000,100,000,100,000,000,000,000,000,100,000,100,000,000,000,000,
000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,000,
100,000,100,000,100,000,000,000,100,000,100,000,000,000,100,000,100,000,100]"