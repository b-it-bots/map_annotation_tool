FROM bitbots/bitbots-common:kinetic

LABEL maintainer="Argentina Ortega"

WORKDIR /travis
COPY ./ /travis/src/mas_navigation

RUN . /opt/ros/mas_stable/setup.sh && \
    catkin config --init && \
    catkin config --extend /opt/ros/mas_stable && \
    apt-get update -qq && \
    rosdep update -qq && \
    rosdep install -q --from-paths src --ignore-src --rosdistro=kinetic -y && \
    rm -rf /var/lib/apt/lists/* && \
    catkin build --no-status
