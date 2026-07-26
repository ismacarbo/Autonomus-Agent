FROM ros:humble-ros-base

RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-slam-toolbox && \
    rm -rf /var/lib/apt/lists/*

COPY bridge_node.py /opt/thesis_slam_bridge/bridge_node.py
COPY mapper_params.yaml /opt/thesis_slam_bridge/mapper_params.yaml
COPY entrypoint.sh /opt/thesis_slam_bridge/entrypoint.sh
RUN chmod +x /opt/thesis_slam_bridge/entrypoint.sh

ENTRYPOINT ["/opt/thesis_slam_bridge/entrypoint.sh"]
