ARG TAG
FROM ghcr.io/tesseract-robotics/trajopt:${TAG}
ENV E_TAG=$TAG

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Install the dependency repositories
RUN apt update -y -qq \
  && if [[ $(lsb_release -cs) != noble ]] ; then \
    apt install -y --no-install-recommends software-properties-common \
    && add-apt-repository -y ppa:levi-armstrong/tesseract-robotics \
    && apt install -y libompl-dev taskflow \
  ; else \
    apt install -y libompl-dev libtaskflow-cpp-dev \
  ; fi

# Install the dependency repositories
# Use a tmpfs mount for the workspace so as not to unnecessarily copy files into the final image
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/tmpfs/tesseract_planning
ARG INSTALL_DIR=/opt/tesseract_planning
RUN mkdir -p ${INSTALL_DIR}

RUN --mount=type=tmpfs,target=${WORKSPACE_DIR} --mount=type=bind,target=${WORKSPACE_DIR}/src/tesseract_planning \
  cd ${WORKSPACE_DIR} \
  && vcs import src < src/tesseract_planning/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry \
  && source /opt/tesseract/install/setup.bash \
  && source /opt/trajopt/install/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && cp -r ${WORKSPACE_DIR}/install ${INSTALL_DIR}
