FROM ubuntu:15.04

# This image contains Klamp't: Kris' Locomotion and Manipulation Planning Toolbox
# This image utilizes X11 in order to allow users to interact with the GUI.
# This image requires root access to run.

MAINTAINER Steve Kuznetsov <skuznets@redhat.com>

# Install dependencies
RUN apt-get update && \
	apt-get -y install g++ \
	 cmake \
	 git \
	 freeglut3 \
	 freeglut3-dev \
	 libglpk-dev \
	 python-dev \
	 python-opengl \
	 libxmu-dev \
	 libxi-dev \
	 libqt4-dev \
	 libassimp-dev \
	 ffmpeg && \
	apt-get clean

# Copy Klamp't files
RUN mkdir /etc/Klampt
COPY CMakeLists.txt /etc/Klampt/CMakeLists.txt
COPY CMakeModules /etc/Klampt/CMakeModules/
COPY Cpp /etc/Klampt/Cpp
COPY data /etc/Klampt/data/
COPY LICENSE /etc/Klampt/LICENSE
COPY Python /etc/Klampt/Python/

# Install Klamp't dependencies
RUN cd /etc/Klampt/Cpp/Dependencies && \
	make unpack-deps && \
	make deps && \
	echo "/etc/Klampt/Cpp/Dependencies/ode-0.14/ode/src/.libs/" >> /etc/ld.so.conf && \
	ldconfig

# Install Klamp't
RUN cd /etc/Klampt && \
	cmake . && \
	make Klampt && \
	make apps && \ 
	make python && \
	make python-install

