
KLAMPTDIR = ../../
KLAMPTDEPS = ../../Library

LIBS += -L/usr/lib \ 
     	-L$${KLAMPTDIR}/lib \
	-L$${KLAMPTDEPS}/KrisLibrary/lib \
	-L/usr/X11R6/lib \
	-L/usr/X11R6/lib/modules/extensions \
	-L/src \
	-L$${KLAMPTDEPS}/ode-0.11.1/ode/src/.libs \
	-L$${KLAMPTDEPS}/tinyxml \
	-L$${KLAMPTDEPS}/glui-2.36/src/lib \
	-L$${KLAMPTDEPS}/assimp--3.0.1270-sdk/lib 

INCLUDEPATH += $${KLAMPTDIR} \
               /usr/include \
               .. \
               ../KlamptQt \
	       $${KLAMPTDEPS}/KrisLibrary \
	       $${KLAMPTDEPS}/ode-0.11.1/ode/src

LIBS += -lKlampt -lKrisLibrary -lode -ltinyxml -lglpk -lglui -lglut -lGLU -lboost_thread-mt -lboost_system-mt

LIBS += -lassimp
DEFINES += dDOUBLE


