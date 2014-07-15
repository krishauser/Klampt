import csv
import sys
import os

###### SETTINGS #####

#set to True if you want to shift back the path time so that the starting time is 0
shiftTime = True

#put the time column header here
tcolumn = "%time"

#fill policy can be 'interpolate' or 'last'
fillPolicy = 'last'

#put the configuration column headers here.  You can also put in a number (e.g. 0) for constant DOFs.
#qcolumns = ["field.position"+str(i) for i in range(17)]
qcolumns = [0.0]+["robot-limb-left-joint_command.sorted.field.command"+str(i) for i in range(7)]+["robot-limb-right-joint_command.sorted.field.command"+str(i) for i in range(7)]

######################

def index(items,item):
    try:
        return [i for (i,v) in enumerate(items) if v==item][0]
    except IndexError:
        return -1


times = []
configs = []
with open(sys.argv[1],'r') as csvfile:
    reader = csv.DictReader(csvfile)
    lineno = 0
    for line in reader:
        lineno+=1
        try:
            times.append(float(line[tcolumn]))
        except Exception:
            print "Error reading time on line",lineno
            raise
        try:
            q = []
            for qname in qcolumns:
                if isinstance(qname,str):
                    try:
                        q.append(float(line[qname]))
                    except ValueError:
                        #unable to parse, appending None
                        q.append(None)
                else:
                    #constant
                    q.append(qname)
            configs.append(q)
        except Exception:
            print "Error reading configuration on line",lineno
            raise

#shift backward to times[0]
if shiftTime:
    times = [t-times[0] for t in times]

#fill in None's with interpolated values
for dof in range(len(configs[0])):
    if any(q[dof]==None for q in configs):
        if fillPolicy == 'interpolate':
            from klampt.trajectory import Trajectory
            traj = Trajectory()
            traj.times = [t for t,q in zip(times,configs) if q[dof] != None]
            traj.milestones = [[q[dof]] for q in configs if q[dof] != None]
            for i,t in enumerate(times):
                if configs[i][dof] == None:
                    configs[i][dof] = traj.eval(t,'clamp')[0]
        elif fillPolicy == 'last':
            last = None
            first = None
            for i,t in enumerate(times):
                if configs[i][dof] == None:
                    configs[i][dof] = last
                else:
                    last = configs[i][dof]
                    if first==None: first = configs[i][dof]
            for i,t in enumerate(times):
                if configs[i][dof] == None:
                    configs[i][dof] = first
                else:
                    break
        else:
            raise ValueError("Can only support interpolate and last fill policies")

#output
if len(sys.argv) >= 3:
    fn = sys.argv[2]
else:
    name,ext = os.path.splitext(sys.argv[1])
    fn = name+".path"
print "Saving to",fn,"..."
f = open(fn,'w')
for (t,q) in zip(times,configs):
    f.write("%f\t%d\t"%(t,len(q)))    
    for qi in q:
        f.write(str(qi)+" ")
    f.write("\n")
f.close()
