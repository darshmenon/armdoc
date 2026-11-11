#!/bin/bash
# Healthy fake arm, then the same arm with a trajectory that never moves, then a fake MoveIt without a planning scene.
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
run() {                      # run <fake_arm flags...> -- <armdoc flags...>
  local fa=() ad=() cur=fa
  for x in "$@"; do if [ "$x" = "--" ]; then cur=ad; else eval "$cur+=(\"\$x\")"; fi; done
  python3 /ws/fake_arm.py "${fa[@]}" >/dev/null 2>&1 & local pid=$!
  sleep 2
  ros2 run armdoc armdoc "${ad[@]}"
  kill "$pid" 2>/dev/null; wait "$pid" 2>/dev/null
  true
}
echo '$ armdoc --ready                 # a healthy arm'
run -- --ready 8
echo; echo '$ armdoc --once 12            # a goal is executing but the joints do not move'
run --stall -- --once 12 --stall-time 4
echo; echo '$ armdoc --moveit --once 10   # move_group is up but serves no planning scene'
run --moveit --no-scene -- --moveit --once 10 --grace 3
echo; echo 'Attach it to your own arm:  docker run --rm --net=host -e ROS_DOMAIN_ID=<id> armdoc armdoc --ready'
