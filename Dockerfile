FROM ubuntu:16.04

ARG docker_user_uid=0
ARG docker_user_gid=0

WORKDIR /realsense

RUN groupadd \
    --gid $docker_user_gid docker_group && \
    useradd \
    --create-home \
    --shell /bin/bash \
    --uid $docker_user_uid \
    --gid $docker_user_gid \
    docker_user && \
	apt-get update && \
    apt-get install \
	--no-install-recommends \
	--assume-yes \
    gcc \
	g++ \
	cmake \
	libglfw3-dev \
	libgtk-3-dev \
	libusb-1.0-0-dev \
    make \
	libelf-dev \
	dkms \
	shim-signed \
	openssl \
	sudo && \
	echo 'docker_user ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
	
USER $docker_user_uid:$docker_user_gid