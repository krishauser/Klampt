from ..math import vectorops,so3,se3


class SimLogger:
    """A CSV logger for a simulation. """
    def __init__(self,sim,state_fn,contact_fn=None,colliding='all',saveheader=True):
        """
        Logs a simulation to a CSV file.

        Arguments:
            sim (Simulator): the klampt.Simulator object you wish to use
            state_fn (str): the file that you want to save state to
            contact_fn (str, optional): the file that you want to save contacts to
                (or None if you don't want them)
            colliding (list, optional): either 'all' (default) or a list of all objects
                and object ids that you want to check self collisions between
            saveheader (bool, optional): true if you want a CSV header giving the name of
                each value.  Default = True
        """
        self.saveSensors = False
        self.sim = sim
        self.fn = state_fn
        self.f = None
        if state_fn != None:
            print("SimLogger: Saving state to",state_fn)
            self.f = open(state_fn,'w')
        self.f_contact = None
        if contact_fn != None:
            print("SimLogger: Saving contacts to",contact_fn)
            self.f_contact = open(contact_fn,'w')
        self.colliding = []
        if colliding=='all':
            self.sim.enableContactFeedbackAll()
            n = self.sim.world.numIDs()
            self.colliding = list(range(n))
        else:
            for i in colliding:
                if isinstance(i,int):
                    self.colliding.append(i)
                elif hasattr(i,'getID'):
                    self.colliding.append(i.getID())
                elif isinstance(i,str):
                    raise NotImplementedError("Lookup id from entity name")
                else:
                    raise ValueError("Invalid object given in the colliding list")
        if saveheader:
            #need to call simulate to get proper sensor readings...
            self.sim.simulate(0)
            self.saveHeader()
            self.saveContactHeader()
        return

    def saveHeader(self,extra=[]):
        if self.f is None:
            print("SimLogger: No state file specified")
            return
        world = self.sim.world
        elements = ['time']
        for i in range(world.numRobots()):
            n = world.robot(i).getName()
            elements.append(n+'_cmx')
            elements.append(n+'_cmy')
            elements.append(n+'_cmz')
            for j in range(world.robot(i).numLinks()):
                elements.append(n+'_qcmd['+world.robot(i).link(j).getName()+']')
            for j in range(world.robot(i).numLinks()):
                elements.append(n+'_dqcmd['+world.robot(i).link(j).getName()+']')
            for j in range(world.robot(i).numLinks()):
                elements.append(n+'_q['+world.robot(i).link(j).getName()+']')
            for j in range(world.robot(i).numLinks()):
                elements.append(n+'_dq['+world.robot(i).link(j).getName()+']')
            for j in range(world.robot(i).numDrivers()):
                elements.append(n+'_t['+str(j)+']')
            if self.saveSensors:
                j = 0
                while True:
                    s = self.sim.controller(i).sensor(j)
                    if len(s.name())==0:
                        break
                    names = s.measurementNames()
                    for sn in range(len(names)):
                        elements.append(n+'_'+s.name()+'['+names[sn]+']')
                    j += 1
        for i in range(world.numRigidObjects()):
            n = world.rigidObject(i).getName()
            elements += [n+'_'+suffix for suffix in ['comx','comy','comz','x','y','z','rx','ry','rz','dx','dy','dz','wx','wy','wz']]
        if extra:
            elements += extra
        self.f.write(','.join(elements))
        self.f.write('\n')
        return

    def saveContactHeader(self):
        if self.f_contact is None:
            print("SimLogger: No contact file specified")
            return
        elements = ['time','body1','body2']
        elements += ['numContacts']
        elements += ['cpx_avg','cpy_avg','cpz_avg','cnx_avg','cny_avg','cnz_avg','fx_avg','fy_avg','fz_avg','mx_avg','my_avg','mz_avg']
        self.f_contact.write(','.join(elements))
        self.f_contact.write('\n')

    def saveStep(self,extra=[]):
        sim = self.sim
        world = sim.world
        sim.updateWorld()
        values = []
        values.append(sim.getTime())
        for i in range(world.numRobots()):
            robot = world.robot(i)
            values += robot.getCom()
            controller = sim.controller(i)
            try:
                values += controller.getCommandedConfig()
                values += controller.getCommandedVelocity()
            except Exception:
                values += [0.0]*robot.numLinks()
                values += [0.0]*robot.numLinks()
            values += sim.getActualConfig(i)
            values += sim.getActualVelocity(i)
            assert len(sim.getActualTorques(i)) == world.robot(i).numDrivers()
            values += sim.getActualTorques(i)
            if self.saveSensors:
                j = 0
                while True:
                    s = self.sim.controller(i).sensor(j)
                    if len(s.name())==0:
                        break
                    meas = s.getMeasurements()
                    assert len(meas) == len(s.measurementNames())
                    values += meas
                    j += 1
        for i in range(world.numRigidObjects()):
            obj = world.rigidObject(i)
            T = obj.getTransform()
            values += se3.apply(T,obj.getMass().getCom())
            values += T[1]
            values += so3.moment(T[0])
            values += sim.body(obj).getVelocity()[1]
            values += sim.body(obj).getVelocity()[0]
        
        if self.f_contact:
            for i,id in enumerate(self.colliding):
                for j in range(i+1,len(self.colliding)):
                    id2 = self.colliding[j]
                    if sim.hadContact(id,id2):
                        clist = sim.getContacts(id,id2);
                        f = sim.contactForce(id,id2)
                        m = sim.contactTorque(id,id2)
                        pavg = [0.0]*3
                        navg = [0.0]*3
                        for c in clist:
                            pavg = vectorops.add(pavg,c[0:3])
                            navg = vectorops.add(navg,c[3:6])
                        if len(clist) > 0:
                            pavg = vectorops.div(pavg,len(clist))
                            navg = vectorops.div(navg,len(clist))
                        body1 = world.getName(id)
                        body2 = world.getName(id2)
                        cvalues = [sim.getTime(),body1,body2,len(clist)]
                        cvalues += pavg
                        cvalues += navg
                        cvalues += f
                        cvalues += m
                        self.f_contact.write(','.join(str(v) for v in cvalues))
                        self.f_contact.write('\n')
        if extra:
            values += extra
        if not (self.f is None):
            self.f.write(','.join([str(v) for v in values]))
            self.f.write('\n')

    def close(self):
        if not (self.f is None):
            self.f.close()
        if not (self.f_contact is None):
            self.f_contact.close()



class SimLogPlayback:
    """A replay class for simulation traces from SimLogger or the SimTest app. """
    def __init__(self,sim,state_fn,contact_fn=None):
        """
        Loads from a CSV file.

        Arguments:
            sim (Simulator): the klampt.Simulator object you wish to use.  This should be
                instantiated with all objects that you recorded from.
            state_fn (str): the state file that you want to load
            contact_fn (str, optional): the contact file that you want to load
        
        """
        import csv
        self.sim = sim
        self.state_header = []
        self.state_array = []
        self.contact_header = []
        self.contact_array = []
        self.state_to_index = {}
        self.contact_to_index = {}
        if state_fn != None:
            print("SimLogPlayback: Loading state from",state_fn)
            f = open(state_fn,'r')
            reader = csv.reader(f)
            rowno = 0
            for row in reader:
                if rowno == 0:
                    self.state_header = row
                    self.state_to_index = dict((v,i) for (i,v) in enumerate(self.state_header))
                else:
                    try:
                        self.state_array.append([float(v) for v in row])
                    except ValueError:
                        print("Error in CSV file",state_fn,"on line",rowno+1,"value is not a float")
                        raise
                rowno += 1
            f.close()
        if contact_fn != None:
            print("SimLogPlayback: Loading contacts from",contact_fn)
            self.f_contact = open(contact_fn,'r')
            reader = csv.reader(self.f_contact)
            rowno = 0
            for row in reader:
                if rowno == 0:
                    self.contact_header = row
                    self.contact_to_index = dict((v,i) for (i,v) in enumerate(self.contact_header))
                else:
                    try:
                        self.contact_array.append([float(v) for v in row])
                    except ValueError:
                        print("Error in CSV file",contact_fn,"on line",rowno+1,"value is not a float")
                        raise
                rowno += 1
            self.f_contact.close()
        #check that the simulation matches the log
        warned = False
        self.robot_indices = []
        self.rigid_object_indices = []
        sim = self.sim
        world = sim.world
        if "time" not in self.state_to_index:
            print("SimLogPlayback: Warning, 'time' column is not present in log file")
        robot_patterns = {'q':'%s_q[%s]','dq':'%s_dq[%s]'}
        for i in range(world.numRobots()):
            indices = {}
            found = True
            robot = world.robot(i)
            for name,p in robot_patterns.items():
                nindices = []
                for j in range(robot.numLinks()):
                    item = p % (robot.getName(),robot.link(j).getName())
                    if item not in self.state_to_index:
                        found=False
                        break
                    nindices.append(self.state_to_index[item])
                if not found:
                    break
                indices[name] = nindices
            if not found:
                print("SimLogPlayback: Warning, not all elements of robot",robot.getName(),"present in log file")
                warned = True
                self.robot_indices.append(None)
                continue
            #TODO: load sensor measurements
            self.robot_indices.append(indices)
        rigid_object_items = ['x','y','z','rx','ry','rz','dx','dy','dz','wx','wy','wz']
        for i in range(world.numRigidObjects()):
            indices = {}
            found = True
            obj = world.rigidObject(i)
            for name in rigid_object_items:
                item = obj.getName()+'_'+name
                if item not in self.state_to_index:
                    print("Missing item",item)
                    found=False
                    break
                indices[name] = self.state_to_index[item]
            if not found:
                print("SimLogPlayback: Warning, not all elements of rigid object",obj.getName(),"present in log file")
                warned = True
                self.rigid_object_indices.append(None)
                continue
            #TODO: load sensor measurements
            self.rigid_object_indices.append(indices)
        if warned:
            input("Press enter to continue")
        return

    def updateSim(self,time=-1,timestep=-1):
        sim = self.sim
        world = sim.world
        if time >= 0:
            try:
                timeindex = self.state_to_index['time']
            except IndexError:
                raise ValueError("'time' column is not present in playback file, can't update by time")
            timelist = [v[timeindex] for v in self.state_array]
            timeindex = 0
            for i in range(len(timelist)-1):
                if time < timelist[i]:
                    timeindex = i
                    break
            #print("Time",time,"Time step",timestep)
            self.updateSim(timestep = timeindex)
            return
        if timestep >= len(self.state_array):
            timestep = len(self.state_array)-1
        state = self.state_array[timestep]
        try:
            timeindex = self.state_to_index['time']
            #sim.fakeSimulate(state[timeindex] - sim.getTime())
            #TODO: change the simulation time
        except IndexError:
            pass
        for i in range(world.numRobots()):
            indices = self.robot_indices[i]
            if indices == None:
                continue
            robot = world.robot(i)
            robot.setConfig([state[ind] for ind in indices['q']])
            robot.setVelocity([state[ind] for ind in indices['dq']])
            for j in range(robot.numLinks()):
                link = robot.link(j)
                sim.body(link).setTransform(*link.getTransform())
                sim.body(link).setVelocity(link.getAngularVelocity(),link.getVelocity())
            #TODO: update sensors
        for i in range(world.numRigidObjects()):
            obj = world.rigidObject(i)
            indices = self.rigid_object_indices[i]
            if indices == None:
                continue
            t = (state[indices['x']],state[indices['y']],state[indices['z']])
            R = so3.from_moment((state[indices['rx']],state[indices['ry']],state[indices['rz']]))
            v = (state[indices['dx']],state[indices['dy']],state[indices['dz']])
            w = (state[indices['wx']],state[indices['wy']],state[indices['wz']])
            obj.setTransform(R,t)
            obj.setVelocity(w,v)
            sim.body(obj).setTransform(R,t)
            sim.body(obj).setVelocity(w,v)
