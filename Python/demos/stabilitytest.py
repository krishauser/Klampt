from klampt import *
from klampt.math import *
from klampt.model.contact import *
from klampt import vis
import time

#these form a square + a downward facing point
fc_contacts = [ContactPoint([-1,-1,0],[0,0,1],0.5),
				ContactPoint([1,-1,0],[0,0,1],0.5),
				ContactPoint([1,1,0],[0,0,1],0.5),
				ContactPoint([-1,1,0],[0,0,1],0.5),
				ContactPoint([0,0,1],[0,0,-1],0.5)]

#these form a square
stable_contacts = [ContactPoint([-1,-1,0],[0,0,1],0.5),
				ContactPoint([1,-1,0],[0,0,1],0.5),
				ContactPoint([1,1,0],[0,0,1],0.5),
				ContactPoint([-1,1,0],[0,0,1],0.5)]

#these form two points pointing in strange directions
unstable_contacts = [ContactPoint([-1,-1,0],[0,1,0],0.5),
				ContactPoint([1,1,0],[1,0,0],0.5)]

print "Force closure (should be True)",forceClosure(fc_contacts)
print "Force closure (should be False)",forceClosure(stable_contacts)	
print "Stable COM (should be list of forces)",comEquilibrium(stable_contacts,[0,0,-1],(0,0,0))
print "Stable COM (should be same list of forces)",comEquilibrium(stable_contacts,[0,0,-1],(0,0,10))
print "Stable COM (should be None)",comEquilibrium(stable_contacts,[0,0,-1],(2,0,10))
print "Any stable (should be True)",comEquilibrium(stable_contacts,[0,0,-1],None)
print "Any stable (should be False)",comEquilibrium(unstable_contacts,[0,0,-1],None)
print "Support polygon planes (should be a square)",supportPolygon(stable_contacts)
print "Support polygon planes (should be entire plane)",supportPolygon(fc_contacts)
print "Support polygon planes (should be invalid)",supportPolygon(unstable_contacts)

"""
h = Hold()
h.contacts = fc_contacts
vis.add("Hold",h)
vis.dialog()
vis.clear();
h.contacts = unstable_contacts
vis.add("Hold",h)
vis.dialog()
vis.clear();
"""

#load and show a simulation, getting the contacts / holds from it
world = WorldModel()
world.readFile("../../data/athlete_plane.xml")
sim = Simulator(world)
sim.enableContactFeedbackAll()

vis.add("world",world)
vis.show()
while vis.shown():
	vis.lock()
	sim.simulate(0.0333)
	sim.updateWorld()

	cm = simContactMap(sim)
	holds = contactMapHolds(cm)
	print "Num contacts",sum(len(h.contacts) for h in holds)
	res = equilibriumTorques(world.robot(0),holds)
	if res is None:
		print "No equilibrium torques/contact forces"
	else:
		tinf,f = res
		t1, f = equilibriumTorques(world.robot(0),holds,norm=1)
		print "Equilibrium forces:"
		for fi in f:
			print "   ",fi
		print "Residual force on CM",vectorops.add(*f)
		print "Torques, L1 / Linf / simulation"
		ts = sim.getActualTorques(0)
		for i,(te1,teinf,tsi) in enumerate(zip(t1[6:],tinf[6:],ts)):
			print "   %s:\t%.3f / %.3f / %.3f"%(world.robot(0).link(6+i).getName(),te1,teinf,tsi)
		print "L1 norm solved",vectorops.norm_L1(t1),"sim",vectorops.norm_L1(ts)
		print "Linf norm solved",vectorops.norm_Linf(tinf),"sim",vectorops.norm_Linf(ts)
	print "Simulation time",sim.getTime()
	vis.unlock()
	
	for i,h in enumerate(holds):
		vis.add("hold "+str(i),h)
	time.sleep(0.01)

vis.kill()

