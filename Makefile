include Makefile.config

FLAGS = $(CPPFLAGS) $(addprefix -D, $(DEFINES))

OBJDIR = objs
LIBDIR = lib

DIRS = Modeling View Control Planning Simulation IO Contact Interface
OBJS= $(foreach dir,$(DIRS), $(dir)/$(OBJDIR)/*.o)
LIB = $(addprefix -L, $(LIBDIRS))  $(addprefix -l, $(LIBS))
LIBKLAMPT = -L$(LIBDIR) -lKlampt 

##################### Start the action #########################
default: RobotTest 
.PHONY: RobotTest SimTest SimUtil PlanDemo PosMeasure RealTimePlanning MotorCalibrate URDFtoRob RealTimePlanner UserTrials Pack Unpack Merge deps lib

unpack-deps:
	cd Library; git clone https://github.com/krishauser/KrisLibrary
	cd Library; tar xvzf ode-0.11.1.tar.gz
	cd Library; tar xvzf glui-2.36.tgz

deps: dep-KrisLibrary dep-tinyxml dep-glui dep-ode

dep-KrisLibrary:
	cd Library/KrisLibrary; cp Makefile.config.klampt Makefile.config
	cd Library/KrisLibrary; make KrisLibrary

dep-tinyxml:
	cd Library/tinyxml; make lib

dep-glui: 
	cd Library/glui-2.36/src; make

dep-ode:
	cd Library/ode-0.11.1; ./configure $(ODECONFIG)
	cd Library/ode-0.11.1; make

docs:
	doxygen doxygen.conf

lib:
	cd Modeling; make
	cd Contact; make 
	cd View; make
	cd Simulation; make
	cd Control; make 
	cd Planning; make
	cd IO; make
	cd Interface; make
	mkdir -p $(LIBDIR)
	ar rcs $(LIBDIR)/libKlampt.a $(foreach dir,$(DIRS),$(dir)/$(OBJDIR)/*.o)
	ranlib $(LIBDIR)/libKlampt.a

clean:
	cd Main; make clean
	cd Modeling; make clean
	cd Contact; make clean
	cd Simulation; make clean
	cd Control; make clean
	cd Planning; make clean
	cd IO; make clean
	cd View; make clean
	cd Interface; make clean
	rm $(LIBDIR)/*.a

RobotTest: lib
	cd Main; make test.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/test.o $(LIBKLAMPT) $(LIB) -o $@

RobotTest2: lib
	cd Main; make test2.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/test2.o $(LIBKLAMPT) $(LIB) -o $@

RobotPose: lib
	cd Main; make pose.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/pose.o $(LIBKLAMPT) $(LIB) -o $@

MotorCalibrate:  lib
	cd Main; make motorcalibrate.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/motorcalibrate.o  $(LIBKLAMPT) $(LIB) -o $@

PosMeasure:  lib
	cd Main; make posmeasure.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/posmeasure.o  $(LIBKLAMPT) $(LIB) -o $@

SimTest:  lib
	cd Main; make simtest.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/simtest.o $(LIBKLAMPT) $(LIB) -o $@

SimUtil:  lib
	cd Main; make simutil.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/simutil.o $(LIBKLAMPT) $(LIB) -o $@

URDFtoRob:  lib
	cd Main; make urdftorob.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/urdftorob.o $(LIBKLAMPT) $(LIB) -o $@		

Pack:  lib
	cd Main; make pack.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/pack.o $(LIBKLAMPT) $(LIB) -o $@		

Unpack:  lib
	cd Main; make unpack.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/unpack.o $(LIBKLAMPT) $(LIB) -o $@		

Merge:  lib
	cd Main; make merge.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/merge.o $(LIBKLAMPT) $(LIB) -o $@		

UserTrials:  lib
	cd Main; make usertrials.o
	 $(CC) $(FLAGS) $(OBJS) Main/$(OBJDIR)/usertrials.o $(LIBKLAMPT) $(LIB) -o $@

SafeSerialClient:  lib
	cd Main; make safeserialclient.o
	 $(CC) $(FLAGS)  Main/$(OBJDIR)/safeserialclient.o $(LIBKLAMPT) $(LIB) -o $@

Val3SerialClient:  lib
	cd Main; make val3serialclient.o
	cd Val3; make
	 $(CC) $(FLAGS)  Main/$(OBJDIR)/val3serialclient.o $(wildcard Val3/$(OBJDIR)/*.o) $(LIBKLAMPT) $(LIB) -o $@

Cartpole:  lib
	cd Examples; make cartpole.o
	 $(CC) $(FLAGS) Examples/$(OBJDIR)/cartpole.o $(LIBKLAMPT) $(LIB) -o $@

PlanDemo:  lib
	cd Examples; make plandemo.o
	 $(CC) $(FLAGS) $(OBJS) Examples/$(OBJDIR)/plandemo.o $(LIBKLAMPT) $(LIB) -o $@

RealTimePlanning:  lib
	cd Examples; make realtimeplanning.o
	 $(CC) $(FLAGS) $(OBJS) Examples/$(OBJDIR)/realtimeplanning.o $(LIBKLAMPT) $(LIB) -o $@


python: lib
	cd Python; make

python-docs:
	cd Python; make docs
