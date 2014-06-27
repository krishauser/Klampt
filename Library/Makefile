.PHONY: unpack-deps deps dep-KrisLibrary dep-ode dep-glui dep-tinyxml

ODECONFIG = --enable-shared --with-trimesh=none
#Comment this out if you want to build ODE in single precision
#(may work on Windows)
ODECONFIG += --enable-double-precision

unpack-deps:
	git clone https://github.com/krishauser/KrisLibrary
	tar xvzf ode-0.11.1.tar.gz
	tar xvzf glui-2.36.tgz

deps: dep-KrisLibrary dep-tinyxml dep-glui dep-ode

dep-KrisLibrary:
	cd KrisLibrary; cmake .
	cd KrisLibrary; make KrisLibrary

dep-tinyxml:
	cd tinyxml; make lib

dep-glui: 
	cd glui-2.36/src; make

dep-ode:
	cd ode-0.11.1; ./configure $(ODECONFIG)
	cd ode-0.11.1; make
