"""Takes SimTest contact state logs and outputs stances.
Change the parameters smoothness, mintime, and robot if you wish."""

import sys
import csv
from collections import defaultdict

#use this timing threshold to eliminate chatter
smoothness = 0.05

#minimum time between stance changes
mintime = 0.02

#if robot == None, autodetect robot
robot = None

def load_contact_log(fn):
    res = defaultdict(list)
    f = open(fn,'r')
    logreader = csv.reader(f)
    count = 0
    for row in logreader:
        if len(row) != 4:
            raise RuntimeError("Invalid CSV row, must be of form time,body1,body2,contact")
        if count==0:
            count+=1
            continue
        count += 1

        time = float(row[0])
        pair = (row[1],row[2])
        contact = int(row[3])
        res[pair].append((time,contact))
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

def smooth(log,smoothness):
    """Outputs a new (time,contact) log such that a contact / break is only
    reported if it is stable for the duration given in smoothness"""
    res = []
    incontact = 0
    for i,(time,contact) in enumerate(log):
        if i+1 == len(log):
            break
        ntime,ncontact = log[i+1]
        assert ncontact != contact,"Contact log must report contact changes only"
        if ntime > time+smoothness:
            if contact != incontact:
                res.append((time,contact))
                incontact = contact
    (time,contact)=log[-1]
    if contact != incontact:
        res.append((time,contact))
    return res


#dict mapping (body1,body2) pairs to contact log (time,contact)
contact_log = dict()
contact_log = load_contact_log(sys.argv[1])

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

#merge all contacts on a single body to a single list
#smooth them too
robot_body_contact_logs = defaultdict(list)
for key,value in robot_contact_log.iteritems():
    body1,body2 = key
    if body1.startswith(robot):
        body1 = body1[len(robot)+1:-1]
        robot_body_contact_logs[body1].append(value)
    if body2.startswith(robot):
        body2 = body2[len(robot)+1:-1]
        robot_body_contact_logs[body2].append(value) 
for key,value in robot_body_contact_logs.iteritems():
    robot_body_contact_logs[key] = smooth(sort_trajectories(value),smoothness)

def write_stances(contact_log,fn,mintime=0):
    assert mintime >= 0
    f = open(fn,'w')
    bodies = sorted(contact_log.keys())
    bodyMap = dict((b,i) for (i,b) in enumerate(bodies))
    f.write("time,")
    f.write(",".join(bodies))
    f.write("\n")
    curstance = [0]*len(bodies)
    lasttime = 0
    flattened = flatten_trajectories(contact_log)
    assert len(flattened) > 0
    for (time,body,contact) in flattened:
        if time > lasttime + mintime:
            f.write(str(lasttime)+",")
            f.write(",".join(str(s) for s in curstance))
            f.write("\n")
            lasttime = time
        curstance[bodyMap[body]]=contact
    f.write(str(time)+",")
    f.write(",".join(str(s) for s in curstance))
    f.write("\n")
    f.close()
    return

print "Writing to stances.csv..."
write_stances(robot_body_contact_logs,'stances.csv',mintime)


