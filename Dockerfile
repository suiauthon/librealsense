FROM ubuntu:16.04

WORKDIR /realsense

RUN apt-get update && apt-get install -y gcc g++ cmake libglfw3-dev libgtk-3-dev libssl-dev libusb-1.0-0-dev pkg-config

CMD bash


