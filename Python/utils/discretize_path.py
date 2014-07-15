import sys
import math
from klampt import trajectory

fin = sys.argv[1]
dt = float(sys.argv[2])
fout = sys.argv[3]

if dt <= 0:
    raise ValueError("Must provide a positive dt")

path = trajectory.Trajectory()
path.load(fin)

dpath = trajectory.Trajectory()
n = int(math.ceil(path.times[-1]/dt))
dpath.times = [i*dt for i in range(n)]
dpath.milestones = [path.eval(i*dt,endBehavior='clamp') for i in range(n)]
dpath.save(fout)

