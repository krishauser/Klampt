import math
import sys
import xml.etree.ElementTree as ET

########### CONVERSION SETTINGS ############

#number of configs per second
speed = 60
#if true, assigns times according to a smooth acceleration profile
accel = True
#time for pausing between sections
pauseTime = 0.01


#begin code
if len(sys.argv) < 3:
    print "USAGE: multipath_to_timed_multipath.py in.xml out.xml"
    print "  (Edit the settings at top of file to change options)"
    exit(0)

tree = ET.parse(sys.argv[1])
root = tree.getroot()
warned = False
curtime = 0.0
for secnum,sec in enumerate(root.findall('section')):
    milestones = sec.findall('milestone')
    for i,m in enumerate(milestones):
        if 'time' in m.attrib:
            if not warned:
                print "Warning: existing multipath already has time, overriding"
                warned = True
        if accel:
            n = len(milestones)+1
            T = float(n)/speed
            #total time: n/speed
            #accelerate and decelerate to use up that time
            #s(t) is such that x'(t(s)) = x'(s) s'(t) is linear inc. in the
            # first half, linearly dec. in the second.
            #x'(t(s)) = c*t(s) = s'(t) => s'(t) = ct =>
            #   s(t) = d+1/2ct^2
            #   t(s) = sqrt(2s/c)
            #we want to find t(s+1)-t(s)
            t1 = 0
            t2 = 0
            if i*2 < n:
                t1 = T/2*math.sqrt(2*float(i)/n);
            else:
                t1 = T-T/2*math.sqrt(2*float(n-i)/n);
            if (i+1)*2 < n:
                t2 = T/2*math.sqrt(2*float(i+1)/n);
            else:
                t2 = T-T/2*math.sqrt(2*float(n-i-1)/n);
            if(t1 == t2):
                print i,t1,t2,T,n
            assert(t1 != t2)
            m.set('time',str(curtime))
            curtime += t2-t1
        else:
            m.set('time',str(curtime))
            curtime += 1.0/speed
    if pauseTime > 0:
        curtime += pauseTime
        e = ET.Element('milestone',attrib=milestones[-1].attrib.copy())
        #copy in last milestone
        e.set('time',str(curtime))
        sec.append(e)

print "Timed path duration:",curtime

tree.write(sys.argv[2])
