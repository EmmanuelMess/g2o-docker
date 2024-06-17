FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt install -y git build-essential cmake gcc ninja-build gdb && \
    rm -rf /var/lib/apt/lists/*

# eigen
RUN apt update && \
    apt install -y libeigen3-dev && \
    rm -rf /var/lib/apt/lists/*

# ceres
RUN apt update && \
    apt install -y cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root/

RUN git clone https://github.com/ceres-solver/ceres-solver --branch 2.2.0

WORKDIR /root/ceres-solver

RUN cmake . -G Ninja

RUN ninja -j4
RUN ninja test
RUN ninja install

WORKDIR /root/

#g2o
RUN apt update && \
    apt install -y libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5 && \
    rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/RainerKuemmerle/g2o --branch 20230223_git

WORKDIR /root/g2o

RUN cmake . -G Ninja

RUN ninja -j4

WORKDIR /

ARG UID=1000
RUN useradd -m -u ${UID} -s /bin/bash builder

RUN chown -R ${UID}:${UID} /root

USER builder

