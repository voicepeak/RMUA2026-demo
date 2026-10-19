# P0/P1 development image. Official competition acceptance remains pending.
ARG ROS_BASE=osrf/ros:noetic-desktop-full@sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797
FROM ${ROS_BASE} AS messages
SHELL ["/bin/bash", "-c"]
COPY interfaces/ros/airsim_ros /opt/rmua_build/src/airsim_ros
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make -C /opt/rmua_build -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DCMAKE_INSTALL_PREFIX=/opt/rmua_msgs install

FROM ${ROS_BASE}
SHELL ["/bin/bash", "-c"]
ARG GIT_REVISION=unknown
LABEL org.opencontainers.image.title="RMUA next development" \
      org.opencontainers.image.revision="${GIT_REVISION}" \
      rmua.acceptance="P0/P1 development; competition disabled"
COPY --from=messages /opt/rmua_msgs /opt/rmua_msgs
COPY . /opt/rmua
ENV PYTHONPATH=/opt/rmua OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONUNBUFFERED=1
WORKDIR /opt/rmua
ENTRYPOINT ["/bin/bash", "/opt/rmua/deployment/entrypoint.sh"]
CMD ["python3", "-m", "adapters.ros_noetic.runtime_observer"]
