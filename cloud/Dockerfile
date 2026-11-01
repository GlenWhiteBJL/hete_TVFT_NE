# syntax=docker/dockerfile:1
FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Keep the ROS final snapshot sources supplied by the official Noetic image.
# The optional, combined host CA bundle is mounted only for this network step;
# session CA files and proxy credentials are never copied into image layers.
RUN --mount=type=secret,id=ca_bundle \
    ca_path=/etc/ssl/certs/ca-certificates.crt && \
    if [[ -f /run/secrets/ca_bundle ]]; then ca_path=/run/secrets/ca_bundle; fi && \
    apt-get -o "Acquire::https::CaInfo=${ca_path}" update && \
    DEBIAN_FRONTEND=noninteractive apt-get \
      -o "Acquire::https::CaInfo=${ca_path}" install -y --no-install-recommends \
      ros-noetic-mavros-msgs ros-noetic-rosbag \
      python3-catkin-pkg python3-numpy python3-scipy python3-yaml \
      python3-matplotlib util-linux && \
    rm -rf /var/lib/apt/lists/*

ENV MPLBACKEND=Agg \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /opt/ne_tvft_ws
COPY CMakeLists.txt package.xml setup.py requirements.txt src/ne_tvft_experiment/
COPY src/ src/ne_tvft_experiment/src/
COPY scripts/ src/ne_tvft_experiment/scripts/
COPY config/ src/ne_tvft_experiment/config/
COPY launch/ src/ne_tvft_experiment/launch/
COPY tests/ src/ne_tvft_experiment/tests/
COPY cloud/run_in_container.sh /usr/local/bin/run-ne-tvft
RUN chmod +x /usr/local/bin/run-ne-tvft src/ne_tvft_experiment/scripts/*.py && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -j2 -l2

WORKDIR /opt/ne_tvft_ws/src/ne_tvft_experiment
ENTRYPOINT ["/usr/local/bin/run-ne-tvft"]
CMD ["20"]
