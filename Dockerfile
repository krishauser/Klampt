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
	 libboost-system-dev \
	 libboost-thread-dev \
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

# Copy Klamp't Data
RUN mkdir /etc/Klampt
COPY CMakeLists.txt /etc/Klampt/CMakeLists.txt
COPY CMakeModules /etc/Klampt/CMakeModules/
COPY Contact /etc/Klampt/Contact/
COPY Control /etc/Klampt/Control/
COPY data /etc/Klampt/data/
COPY Documentation /etc/Klampt/Documentation/
COPY doxygen.conf /etc/Klampt/doxygen.conf
COPY doxygen.conf.in /etc/Klampt/doxygen.conf.in
COPY Examples /etc/Klampt/Examples/
COPY Interface /etc/Klampt/Interface/
COPY IO /etc/Klampt/IO/
COPY Library /etc/Klampt/Library/
COPY LICENSE /etc/Klampt/LICENSE
COPY Main /etc/Klampt/Main/
COPY Modeling /etc/Klampt/Modeling/
COPY Planning /etc/Klampt/Planning/
COPY Python /etc/Klampt/Python/
COPY Simulation /etc/Klampt/Simulation/
COPY View /etc/Klampt/View/

# Install Klamp't dependencies
RUN cd /etc/Klampt/Library && \
	make unpack-deps && \
	make deps && \
	echo "/etc/Klampt/Library/ode-0.14/ode/src/.libs/" >> /etc/ld.so.conf && \
	ldconfig

# Install Klamp't
RUN cd /etc/Klampt && \
	cmake . && \
	make Klampt && \
	make apps && \ 
	make python && \
	make python-install

# Volume Mount User Data
VOLUME /home/Klampt/data