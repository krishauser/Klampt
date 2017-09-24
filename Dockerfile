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
COPY Web /etc/Klampt/Web/

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

# Build WebServer program 
RUN cd /etc/Klampt/Web/Server && \
    make

# Get web client JS libraries
RUN cd /etc/Klampt/Web/Client && \ 
    git submodule init && \
    git submodule update

# Link required client-side files on /var/www/html
RUN mkdir /var/www/html
RUN cd /var/www/html && \
    ln -s /etc/Klampt/Web/Client/ace-builds && \
    ln -s /etc/Klampt/Web/Client/images && \
    ln -s /etc/Klampt/Web/Client/index_debug.html && \
    ln -s /etc/Klampt/Web/Client/index.html && \
    ln -s /etc/Klampt/Web/Client/KlamptClient.js && \
    ln -s /etc/Klampt/Web/Client/kviz_docs.html && \
    ln -s /etc/Klampt/Web/Client/Scenarios && \
    ln -s /etc/Klampt/Web/Client/stats.js && \
    ln -s /etc/Klampt/Web/Client/three.js && \
    ln -s /etc/Klampt/Web/Client/w2ui 

# Volume Mount User Data
VOLUME /home/Klampt/data
