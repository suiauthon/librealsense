FROM ubuntu:16.04

WORKDIR /realsense

RUN apt-get update && \
    apt-get install --no-install-recommends --assume-yes \
    gcc g++ cmake libglfw3-dev libgtk-3-dev libusb-1.0-0-dev
