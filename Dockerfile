# One-command demo of armdoc against a fake arm, or attach it to a real stack on the same ROS_DOMAIN_ID:
#   docker build -t armdoc .
#   docker run --rm armdoc                                   # demo: healthy arm, then a stalled trajectory
#   docker run --rm --net=host -e ROS_DOMAIN_ID=0 armdoc armdoc --ready    # check a running arm on this machine
FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-control-msgs ros-humble-controller-manager-msgs ros-humble-diagnostic-msgs \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . src/armdoc
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select armdoc \
    && cp src/armdoc/test/fake_arm.py /ws/fake_arm.py \
    && cp src/armdoc/docker/entrypoint.sh src/armdoc/docker/demo.sh /ws/ && chmod +x /ws/entrypoint.sh /ws/demo.sh

ENV ROS_DOMAIN_ID=77
ENTRYPOINT ["/ws/entrypoint.sh"]
CMD ["demo"]
