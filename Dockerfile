FROM ubuntu:20.04

# This image contains Klamp't: Kris' Locomotion and Manipulation Planning Toolbox
# This image utilizes X11 in order to allow users to interact with the GUI.
# This image requires root access to run.

MAINTAINER Steve Kuznetsov <skuznets@redhat.com>

# Install dependencies
RUN apt-get update

RUN DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata

RUN apt-get -y install freeglut3-dev qt5-default

RUN apt-get -y install python3-dev python3-pip

RUN pip3 install PyOpenGL PyQt5 Klampt
