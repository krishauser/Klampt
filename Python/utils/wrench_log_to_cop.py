"""Takes SimTest contact wrench logs and outputs centers of pressure.
Change the parameter robot if you wish."""

import sys
import csv
from collections import defaultdict

#if robot == None, autodetect robot
robot = None

def load_contact_wrench_log(fn):
    res = defaultdict(list)
    f = open(fn,'r')
    logreader = csv.reader(f)
    count = 0
    for row in logreader:
        if len(row) != 12:
            raise RuntimeError("Invalid CSV row, must be of form time,body1,body2,cpx,cpy,cpz,fx,fy,fz,tx,ty,tz")
        if count==0:
            count+=1
            continue
        count += 1

        time = float(row[0])
        pair = (row[1],row[2])
        contact = tuple([float(v) for v in row[3:6]])
        force = tuple([float(v) for v in row[6:9]])
        torque = tuple([float(v) for v in row[9:12]])
        res[pair].append((time,contact,force,torque))
    return res

def sort_trajectories(trajectories):
    """Given a trajectory list, sorts their entries in time-wise fashion"""
    allpts = sum(trajectories,[])
    return sorted(allpts,key=lambda(x):x[0])

def flatten_trajectories(traj_dict):
    """Converts a dict of form body:[(time1,contact1),...,(timen,contactn)]
    to a list of form (time1,bodyk,contact1)...,(timen,bodyj,contactn)"""
    trajectories = []
    for (body,path) in traj_dict.iteritems():
        trajectories.append([(time,body,contact) for (time,contact) in path])
    return sort_trajectories(trajectories)

#dict mapping (body1,body2) pairs to contact log (time,contact)
contact_log = dict()
contact_log = load_contact_wrench_log(sys.argv[1])

if robot == None:
    for key in contact_log.iterkeys():
        body1,body2 = key
        pos = body1.find("[")
        if pos >= 0:
            robot = body1[:pos]
            print "Auto-detected robot",robot
            break
        pos = body2.find("[")
        if pos >= 0:
            robot = body2[:pos]
            print "Auto-detected robot",robot
            break

robot_contact_log = dict()
for key,value in contact_log.iteritems():
    body1,body2 = key
    if body1.startswith(robot):
        assert (body1,body2) not in robot_contact_log, "Duplicate contact state in reverse mode, %s, %s"%(body1,body2)
        robot_contact_log[(body1,body2)] = value
    elif body2.startswith(robot):
        assert (body2,body1) not in robot_contact_log, "Duplicate contact state in reverse mode, %s, %s"%(body1,body2)
        robot_contact_log[(body2,body1)] = value

if len(robot_contact_log)==0:
    print "No robot found in contact log"
    exit(0)


trajectory = sum(robot_contact_log.values(),[])
sortedtraj = defaultdict(list)
for v in trajectory:
    sortedtraj[v[0]].append(v[1:])

print "Writing to cops.csv..."
f = open("cops.csv",'w')
f.write("time,cop_x,cop_y,cop_z\n")
#now contains stances sorted by time
for time,v in sorted(sortedtraj.items()):
    wsum = 0
    cop = [0,0,0]
    for contact in v:
        cp = contact[0]
        force = contact[1]
        torque = contact[2]
        w = force[2]
        for i in range(3):
            cop[i] += cp[i]*w
        wsum += w
    for i in range(3):
        cop[i] /= wsum
    #throw out inf's and nan's
    if all(abs(v) < 10 for v in cop):
        f.write("%f,%f,%f,%f\n"%(time,cop[0],cop[1],cop[2]))
