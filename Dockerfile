FROM ubuntu:16.04

# This image contains Klamp't: Kris' Locomotion and Manipulation Planning Toolbox

MAINTAINER Mark McCahill

# Install dependencies
RUN apt-get update && \
	apt-get -y install \
	 build-essential \
	 wget \
	 sudo \
	 g++ \
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
	 ffmpeg \
	 doxygen \
	 qt5-default \
	 libtf-dev && \
	apt-get clean

RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main " >> /etc/apt/sources.list.d/ros-latest.list
RUN DEBIAN_FRONTEND=noninteractive apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update && \
    apt-get -y install \
      ros-kinetic-desktop-full \
      python-rosinstall \
      python-rosinstall-generator \
      python-wstool \
      python-rosdep && \
    apt-get clean 
	  
RUN cd / ; \
    git clone -b web_devel https://github.com/krishauser/Klampt.git

RUN rosdep init

# add a non-root user so we can run as that user; make sure that user is in the group "users"
RUN adduser --disabled-password --gecos "" --ingroup users klamptuser

RUN chown -R klamptuser /Klampt
USER klamptuser
RUN rosdep update
RUN cd /Klampt ; \
    git submodule init ; \
    git submodule update

USER root

# Install Klamp't dependencies
RUN cd /Klampt/Library && \
	make unpack-deps && \
	make deps 

USER klamptuser

# Install Klamp't
RUN cd /Klampt && \
	cmake . && \
	make Klampt && \
	make apps && \ 
	make python 
	
USER root
RUN cd /Klampt ; \
	make python-install

USER klamptuser

# Build WebServer program 
RUN cd /Klampt/Web/Server && \
    make

# copy the web server program into place
RUN cp /Klampt/Web/Server/WebServer /Klampt/


USER root

# Supervisord
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y supervisor && \
   mkdir -p /var/log/supervisor

# Config files
COPY   dockerconfigs/supervisord-klampt.conf /etc/supervisor/conf.d/supervisord-klampt.conf

# add a script that supervisord could use to initialize a user and  password based on an optional
# environmental variable ($USERPASS) passed in when the containers is instantiated
COPY dockerconfigs/initialize.sh /

RUN apt-get install  -y locales 
RUN locale-gen en_US en_US.UTF-8
RUN DEBIAN_FRONTEND=noninteractive dpkg-reconfigure locales

# expose the websocket port that the Klampt server listens on
EXPOSE 1234

CMD ["/usr/bin/supervisord"]

