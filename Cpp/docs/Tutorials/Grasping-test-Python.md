# [Lab Service] Tutorials for Klampt Grasping Test

In this tutorial, we will learn how to implement a simple grasping simulation for the gripper that has a free-floating moving base. 

```python
import klampt 
from klampt.vis import GLRealtimeProgram
from klampt import vis
from klampt.math import vectorops, so3, se3
from klampt.model import contact
from moving_base_control import set_moving_base_xform, send_moving_base_xform_linear

world = klampt.WorldModel()
res = world.readFile('box_robot_floating.xml')
if not res:
    raise RuntimeError("Unable to load world")
```

To create rob file and xml file, you can see how [this project](https://github.com/krishauser/IROS2016ManipulationChallenge) works. You can use functions in [moving_base_control.py](https://github.com/krishauser/IROS2016ManipulationChallenge/blob/master/moving_base_control.py) for handling the free-floating moving base. 

### 1. Make Rigid Object from file

You can use the OFF file of the object to be grasped. To locate the object on the ground safely before starting grasp simulation, transformation is computed by using approximation of the bounding box surrounding the object. 

```python
object = klampt.Geometry3D()
object.loadFile(file_name)
obj = world.makeRigidObject("object")
obj.geometry().set(object)
obj.geometry().scale(0.35)
contact_params = obj.getContactParameters()
contact_params.kRestitution = 0.0
contact_params.kFriction = 0.500000
contact_params.kStiffness = 200000.0
contact_params.kDamping = 1000.0
obj.setContactParameters(contact_params)

obj.setTransform(*se3.identity())
bmin, bmax = obj.geometry().getBB()
T = obj.getTransform()
spacing = 0.006
T = (T[0], vectorops.add(T[1], (-(bmin[0] + bmax[0]) * 0.5, -(bmin[1] + bmax[1]) * 0.5, -bmin[2] + spacing)))
obj.setTransform(*T)
obj.appearance().setColor(0.9, 0, 0.4, 1.0)

object_center = T[1]
object_r = np.sqrt(3)* np.max(np.abs([bmin[0], bmax[0], bmin[1], bmax[1], bmin[2], bmax[2]])) + 0.4

```

### 2. Set approach direction & starting position

You may set an approach direction as a unit vector on the hemisphere. By using spherical coordinates, define a unit vector as $(\theta, \phi), \theta\in[0, 2\pi]\phi\in[0, \pi/2]$. 

```python
theta = np.pi
phi = np.pi/6
```

Then set the orientation and position of the moving base of gripper.  

```python
unit_vector_on_hemisphere = [np.sin(phi) * np.cos(theta), np.sin(phi) * np.sin(theta), np.cos(phi)]
R = so3.canonical(unit_vector_on_hemisphere)
R = R[3:9] + R[:3]
t = vectorops.add(vectorops.mul(unit_vector_on_hemisphere, object_r), object_center)
set_moving_base_xform(world.robot(0), R, t)
```

### 3. Simulation

You can find the following example code from [Klampt-examples/Python3/demos/gl_vis.py](https://github.com/krishauser/Klampt-examples/blob/master/Python3/demos/gl_vis.py). GLRealtimeProgram calls a idle() function on a constant time step. 

```python
class GraspGL(GLRealtimeProgram):
    def __init__(self,world,sim):
        GLRealtimeProgram.__init__(self,"GraspGL")
        self.world = world
        self.sim = sim

    def display(self):
        self.sim.updateWorld()
        self.world.drawGL()

    def idle(self):
        # Fill out this part.	
        sim.simulate(self.dt)
        return
```

We need to set some variables in init function.

```python
self.sim.enableContactFeedbackAll()
self.num_total_links = 13 
self.dt = 0.02
self.joint_limits = world.robot(0).getJointLimits()
self.object_id = world.rigidObject(0).getID()

#approach
self.approach_velocity = 0.01

#close 
self.num_failure = 0 # count the number of failure in grasping
self.failure_limit = 80 # if the gripper fails over failure_limit, the trial is regarded as failure.
self.velocity_limit = 0.5 
self.angular_velocity = 0.8

self.gripper_state = 1
self.grasp_sucess = False
```

Every time step, contacts info should be updated in idle() method. For the convenience, you can create contact cheking methods in GraspGL class.

```python
def check_contact_terrain(self):
    num_contacts = 0
    terrain_id = self.world.terrain(0).getID()
    for idx in range(5, 18):
        link_id = self.world.robot(0).link(idx).getID()
        if self.sim.inContact(terrain_id, link_id):
            contact_points = self.sim.getContacts(terrain_id, link_id)
            num_contacts += len(contact_points)
    return num_contacts > 0

def check_contact_object(self):
    contacted_links = []
    for idx in range(5, 18):
        link_id = self.world.robot(0).link(idx).getID()
        if self.sim.inContact(self.object_id, link_id):
            contact_points = self.sim.getContacts(self.object_id, link_id)
            contacted_links += [idx]
    return contacted_links
```

My simulation plan is as below:

The gripper approaches the object until it touches the object. And the gripper starts closing its fingers to grasp the object. If it touches the ground first, grasp simulation is regarded as failure. If the max of the sensed velocities of links is less than the threshold and the number of contacted links is greater than 4, it stops closing. 

Following my plan, the gripper's states are defined as: 

- done = -1
- collide_with_terrain = 0
- approaching = 1
- closing = 2

Here is the example idle() method. 

```python
def idle(self):
    if self.gripper_state <= 0:
        vis.show(False)
    controller = self.sim.controller(0)
    if self.sim.getTime() > 1 and self.gripper_state > 0:
        contacted_links = self.check_contact_object()
        if self.gripper_state is approaching:
            if len(contacted_links) > 0:
                self.gripper_state = closing
            else:
                if self.check_contact_terrain():
                    self.gripper_state = collide_with_terrain
                Tr = self.world.robot(0).link(5).getTransform()
                translation = vectorops.add(so3.apply(Tr[0], [0, 0, -self.approach_velocity]), Tr[1])
                send_moving_base_xform_linear(controller, Tr[0], translation, self.dt)
        if self.gripper_state is closing:
            q = controller.getCommandedConfig()
            if len(contacted_links) < self.num_total_links:
                for link_idx in range(9, 18):
                    new_q = q[link_idx] + self.angular_velocity * self.dt
                    q[link_idx] = np.clip(new_q, self.joint_limits[0][link_idx], self.joint_limits[1][link_idx])
                controller.setPIDCommand(q, [0] * len(q))
            if max(controller.getSensedVelocity()[9:18]) < self.velocity_limit and len(contacted_links) >= 4:
                self.gripper_state = done
                self.grasp_success = True
            else:
                self.num_failure += 1
                if self.num_failure > self.failure_limit:
                    self.gripper_state = done
    self.sim.simulate(self.dt)
    return 
```

After filling in the GraspGL with your control plan, run the simulation and see what happens.

```python
sim = klampt.Simulator(world)
gl_test = GraspGL(world, sim)
gl_test.run()
print('simulation result: ', gl_test.grasp_success)
```

You can evaluate the grasp by 

- Constructing Grasp Wrench Space from the contact information
- Lifting the object and moving the gripper in random directions to check if the grasp is robust.