#!/bin/sh
# Copy the ROS-free report module and a recorded example next to the page, so it can be hosted as static files.
set -e
cd "$(dirname "$0")"
rm -rf armdoc examples && mkdir -p armdoc examples
touch armdoc/__init__.py
cp ../armdoc/report.py armdoc/
cp ../docs/ur3_run.jsonl examples/
