#!/bin/bash
# demo (default)        run the fake-arm demo
# armdoc [flags]        run armdoc itself, e.g. `armdoc --ns /ur3 --ready`
# anything else         run as given (shell, ros2 ..., python3 ...)
set -e
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
case "${1:-demo}" in
  demo)   exec /ws/demo.sh ;;
  armdoc) shift; exec ros2 run armdoc armdoc "$@" ;;
  *)      exec "$@" ;;
esac
