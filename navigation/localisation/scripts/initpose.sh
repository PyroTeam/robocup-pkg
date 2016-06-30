#! /bin/bash

TEAM=$1
ROBOT_NB=$2
NAMESPACE=$(rosparam get /${TEAM}/machineLocation/robot${ROBOT_NB})
if [[ "${NAMESPACE}" != "" ]]; then
  NAMESPACE="/${NAMESPACE}"
fi

echo "NAMESPACE: \"${NAMESPACE}\""

rostopic info ${NAMESPACE}/initialpose
RES=$?
while [[ $RES -ne 0 ]]; do
  sleep 1
  rostopic info ${NAMESPACE}/initialpose
  RES=$?
done

TEAMCOLOR=$(rosparam get ${NAMESPACE}/teamColor)
if [[ "${TEAMCOLOR}" == "magenta" ]]; then
  INVERT=1
else
  INVERT=0
fi

echo "TEAMCOLOR: \"${TEAMCOLOR}\""

ORIENTATION_STD="{x: 0.0, y: 0.0, z: -0.92387953251128, w: 0.3826834323650}"
ORIENTATION_INV="{x: 0.0, y: 0.0, z: -0.3826834323650, w: 0.92387953251128}"

POSE_STD="{x: 3.5, y: -0.5, z: 0.0}"
POSE_INV="{x: -3.5, y: -0.5, z: 0.0}"

if [[ $INVERT -eq 0 ]]; then
  POSE=${POSE_STD}
  ORIENTATION=${ORIENTATION_STD}
else
  POSE=${POSE_INV}
  ORIENTATION=${ORIENTATION_INV}
fi

echo "POSE: \"${POSE}\""
echo "ORIENTATION: \"${ORIENTATION}\""

rostopic pub ${NAMESPACE}/initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: map
pose:
  pose:
    position: ${POSE}
    orientation: ${ORIENTATION}
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" &
ROSTOPIC_PID=$!
sleep 1
kill $ROSTOPIC_PID

exit 0
