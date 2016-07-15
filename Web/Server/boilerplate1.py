#!/usr/bin/python
import emb
import log
import sys
import time
from klampt import *
from klampt.webrobotprogram import *
sys.path.append("Web/Server")
#sys.path.append(".")
import kviz
#from kviz import *

class StdoutCatcher:
	def write(self, str):
		log.CaptureStdout(str)
class StderrCatcher:
	def write(self, str):
		log.CaptureStderr(str)
sys.stdout = StdoutCatcher()
sys.stderr = StderrCatcher()


class MyWebViewer(WebSimulationProgram):
    def __init__(self,files):
	
        #create a world from the given files
        world = WorldModel()
        for fn in files:
            print "trying to load:" + fn;
            res = world.readFile(fn)
            if not res:
                raise RuntimeError("Unable to load model "+fn)

        WebSimulationProgram.__init__(self,world,"My Web program")
        kviz._init(world)
	self.frame = 0 
	self.student_init = False

    def compute_JSON(self):
        self.sim.updateWorld()
	if self.frame==1:
		self.starttime=time.time();
        	self.jString=kviz._getInitialJSON()
		#print self.jString		
	 	self.secs = time.time()- self.starttime
        	self.msecs = self.secs * 1000  # millisecs
		print "Getting the scene in JSON format took: " + "{:.2f}".format(self.msecs) + " ms"
	else:	
		self.starttime=time.time();
		self.jString=kviz._getUpdateJSON()
		#print self.jString;
		self.secs = time.time() - self.starttime
        	self.msecs = self.secs * 1000  # millisecs
		print "Getting the transforms in JSON format took: " + "{:.2f}".format(self.msecs) + " ms"

    def internal_display(self):
	self.compute_JSON()

	#try:
	#	global display  #so user can add to display, haven't worked this out completely yet...
    	#	display()
	#except:
	#	pass


    def send_JSON(self):
	self.starttime=time.time(); 
	emb.send(self.jString)
	self.secs = time.time()- self.starttime
        self.msecs = self.secs * 1000  # millisecs
	print "sending JSON took: " + "{:.2f}".format(self.msecs) + " ms"

    def internal_control_loop(self):
	if self.student_init == False:
		try:
			global init
	    		init(self.world.robot(0))
		except Exception as e:
			print "Exception in init code"
			raise
		self.student_init = True

	self.starttime=time.time()
	
        try:
		global control_loop
    		control_loop(self.sim.getTime(),self.sim.controller(0))  #call student code
	except Exception as e:
		print "Exception in control_loop code"
		raise
	
	self.sim.simulate(self.dt)

	self.secs = time.time()- self.starttime
        self.msecs = self.secs * 1000  # millisecs
	print "computing simulation took: " + "{:.2f}".format(self.msecs) + " ms"

	self.frame+=1

    def mousefunc(self,button,state,x,y):
        pass
    def motionfunc(self,x,y):
        pass
    def specialfunc(self,c,x,y):
        pass
    def keyboardfunc(self,c,x,y):
        pass

#if __name__ == "__main__":
#    
print "gltemplate.py: This example demonstrates how to simulate a world and read user input"

boilerplate_setup = False
boilerplate_frame_precomputed = False

def boilerplate_advance():
	global viewer	
	global boilerplate_setup
	global boilerplate_frame_precomputed

	if boilerplate_setup == False:
		viewer = MyWebViewer(["./data/athlete_fractal_1.xml"])
		boilerplate_setup = True

	if boilerplate_frame_precomputed == False:
		viewer.internal_control_loop()
		viewer.internal_display()
	
	viewer.send_JSON()

	viewer.internal_control_loop()  #we'll precompute the next frame to speed things up
	viewer.internal_display()		
	boilerplate_frame_precomputed = True

def boilerplate_advance_no_precompute():
	global viewer	
	global boilerplate_setup
	global boilerplate_frame_precomputed

	if boilerplate_setup == False:
		viewer = MyWebViewer(["./data/athlete_fractal_1.xml"])
		boilerplate_setup = True

	if boilerplate_frame_precomputed == False:
		viewer.internal_control_loop()
		viewer.internal_display()

	viewer.send_JSON()
	boilerplate_frame_precomputed=False

