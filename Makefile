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
.PHONY: RobotTest SimTest SimUtil PosMeasure URDFtoRob UserTrials UserTrialsMT deps lib

unpack-deps:
	cd Library; git clone https://github.com/krishauser/KrisLibrary
	cd Library; tar xvzf ode-0.11.1.tar.gz
	cd Library; tar xvzf glui-2.36.tgz

deps: dep-KrisLibrary dep-tinyxml dep-glui dep-ode

dep-KrisLibrary:
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
	cd Interface; make
	rm $(LIBDIR)/*.a

RobotTest: lib
	cd Main; make test.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/test.o $(LIBKLAMPT) $(LIB) -o $@

RobotPose: lib
	cd Main; make pose.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/pose.o $(LIBKLAMPT) $(LIB) -o $@

Cartpole:  lib
	cd Main; make cartpole.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/cartpole.o $(LIBKLAMPT) $(LIB) -o $@

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
UserTrials:  lib
	cd Main; make usertrials.o
	 $(CC) $(FLAGS) $(OBJS) Main/$(OBJDIR)/usertrials.o $(LIBKLAMPT) $(LIB) -o $@

UserTrialsMT:  lib
	cd Main; make usertrials_multithread.o
	cd Input; make
	 $(CC) $(FLAGS)  Main/$(OBJDIR)/usertrials_multithread.o $(LIBKLAMPT) $(LIB) -o $@

python: lib
	cd Python; make

python-docs:
	cd Python; make docs
