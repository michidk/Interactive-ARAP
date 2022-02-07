# base image
FROM ubuntu:20.04 as base

# setup environment
ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# update sources and install base requirements
RUN apt-get update && \
  apt-get install -y \
  software-properties-common lsb-release

# final dev environment stage
FROM base as dev-env

MAINTAINER Michael Lohr <michael@lohr.dev>
LABEL description="A C++ development environment for the `Interactive ARAP` project."

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# install dependencies & dev tools
RUN apt-get install -y \
  # misc tools
  wget \
  git \
  htop \
  iputils-ping \
  curl \
  # build tools
  build-essential \
  make \
  cmake \
  g++-10 \
  gcc-10 \
  # libeigen
  libeigen3-dev \
  # GLFW
  libglfw3 libglfw3-dev \
  # GL dependencies
  libx11-dev \
  mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev \
  libxrandr-dev \
  libxi-dev \
  libxmu-dev \
  libblas-dev \
  libxinerama-dev \
  libxcursor-dev \
  # install Zenity for dialogs
  # zenity \
  # cleanup
  # && rm -rf /var/lib/apt/lists/*
  && apt-get clean

RUN rm /usr/bin/g++ && ln -s /usr/bin/g++-10 /usr/bin/g++ && rm /usr/bin/gcc && ln -s /usr/bin/gcc-10 /usr/bin/gcc

# Install just
RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /bin

RUN mkdir /src
WORKDIR /src

CMD ["/bin/bash"]
