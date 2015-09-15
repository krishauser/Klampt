from klampt import vectorops,so3,se3


class SimLogger:
    def __init__(self,sim,state_fn,contact_fn=None,colliding='all'):
        """
        Logs a simulation to a CSV file.

        Arguments:
        - sim: the klampt.Simulator object you wish to use
        - fn: the file that you want to save to
        - colliding: either 'all' (default) or a list of all objects
          / object ids that you want to check self collisions between
        """
        self.sim = sim
        self.fn = state_fn
        self.f = None
        if state_fn != None:
            self.f = open(state_fn,'w')
        self.f_contact = None
        if contact_fn != None:
            self.f_contact = open(contact_fn,'w')
        self.colliding = []
        if colliding=='all':
            self.sim.enableContactFeedbackAll()
            n = self.sim.world.numIDs()
            self.colliding = range(n)
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
        self.saveHeader()
        self.saveContactHeader()
        return

    def saveHeader(self,extra=[]):
        world = self.sim.world
        elements = ['time']
        for i in xrange(world.numRobots()):
            n = world.robot(i).getName()
            elements.append(n+'_cmx')
            elements.append(n+'_cmy')
            elements.append(n+'_cmz')
            for j in xrange(world.robot(i).numLinks()):
                elements.append(n+'_q['+world.robot(i).link(j).getName()+']')
            for j in xrange(world.robot(i).numLinks()):
                elements.append(n+'_dq['+world.robot(i).link(j).getName()+']')
            for j in xrange(world.robot(i).numDrivers()):
                elements.append(n+'_t['+str(i)+']')
            """
            j = 0
            while True:
                s = self.sim.controller(i).getSensor(j)
                if len(s.name())==0:
                    break
                names = s.measurementNames()
                for sn in range(len(names)):
                    elements.append(n+'_'+s.name()+'['+names[sn]+']')
                j += 1
            """
        for i in xrange(world.numRigidObjects()):
            n = world.rigidObject(i).getName()
            elements += [n+'_'+suffix for suffix in ['comx','comy','comz','x','y','z','rx','ry','rz','dx','dy','dz','wx','wy','wz']]
        if extra:
            elements += extra
        self.f.write(','.join(elements))
        self.f.write('\n')
        return

    def saveContactHeader(self,extra=[]):
        elements = ['time','body1','body2']
        elements += ['numContacts']
        elements += ['cpx_avg','cpy_avg','cpz_avg','cnx_avg','cny_avg','cnz_avg','fx_avg','fy_avg','fz_avg','mx_avg','my_avg','mz_avg']
        elements += extra
        self.f_contact.write(','.join(elements))
        self.f_contact.write('\n')

    def saveStep(self,extra=[]):
        sim = self.sim
        world = sim.world
        sim.updateWorld()
        values = []
        values.append(sim.getTime())
        for i in xrange(world.numRobots()):
            robot = world.robot(i)
            values += robot.getCom()
            values += robot.getConfig()
            values += sim.getActualTorques(i)
            """
            j = 0
            while True:
                s = self.sim.controller(i).getSensor(j)
                if len(s.name())==0:
                    break
                meas = s.measurements()
                values += meas
                j += 1
            """
        for i in xrange(world.numRigidObjects()):
            obj = world.rigidObject(i)
            T = obj.getTransform()
            values += se3.apply(T,obj.getMass().getCom())
            values += T[1]
            values += so3.moment(T)
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
                        values = [sim.getTime(),body1,body2,len(clist)]
                        values += pavg
                        values += navg
                        values += f
                        values += m
                        self.f_contact.write(','.join(str(v) for v in values))
                        self.f_contact.write('\n')
        if extra:
            values += extra
        self.f.write(','.join([str(v) for v in values]))
        self.f.write('\n')

    def close(self):
        self.f.close()
        self.f_contact.close()
