from klampt import *
from klampt.sim import batch
from klampt.model import map
import random, math

world = WorldModel()
fn = "world1.xml"
res = world.readFile(fn)
if not res:
    raise RuntimeError("Unable to load world "+fn)

item = 'robots[0].config[0]'
itemsampler = lambda: random.uniform(-math.pi, math.pi)
initialConditionSamplers = {item:itemsampler}
initialConditionSamplers['args'] = lambda:(random.uniform(-0.5, 0.5),)

def simInitFun(sim,targetAngle):
    controller = sim.controller(0)
    controller.setPIDCommand([targetAngle],[0])
    kP = 20
    kI = 8
    kD = 5
    controller.setPIDGains([kP],[kI],[kD])

returnItems = ['robots[0].config']
duration = 5
N = 100
res = batch.monteCarloSim(world,duration,initialConditionSamplers,N,returnItems, simInit=simInitFun)

f = open('result.txt', 'w')
for i in range(N):
    initialCond,results = res[i]
    #print results
    startConfig = initialCond['robots[0].config[0]']
    endConfig = results[returnItems[0]]
    print "from",startConfig,"to",initialCond["args"][0],"->",endConfig
    f.write('%lf\n'%(endConfig[0]))
f.close()

