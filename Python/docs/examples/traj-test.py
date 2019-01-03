from klampt.model import trajectory

#milestones = [[0,0,0],[0,0,0],[1,0,0],[2,0,1],[2.2,0,1.5],[3,0,1],[4,0,-0.3]]
milestones = [[0,0,0],[0.02,0,0],[1,0,0],[2,0,1],[2.2,0,1.5],[3,0,1],[4,0,-0.3]]

traj = trajectory.Trajectory(milestones=milestones)

#prints milestones 0-5
print 0,":",traj.eval(0)
print 1,":",traj.eval(1)
print 2,":",traj.eval(2)
print 3,":",traj.eval(3)
print 4,":",traj.eval(4)
print 5,":",traj.eval(5)
print 6,":",traj.eval(6)
#print some interpolated points
print 0.5,":",traj.eval(0.5)
print 2.5,":",traj.eval(2.5)
#print some stuff after the end of trajectory
print 7,":",traj.eval(6)
print 100.3,":",traj.eval(100.3)
print -2,":",traj.eval(-2)

from klampt import vis

vis.add("point",[0,0,0])
vis.animate("point",traj)
vis.add("traj",traj)
#vis.spin(float('inf'))   #show the window until you close it

traj2 = trajectory.HermiteTrajectory()
traj2.makeSpline(traj)

vis.animate("point",traj2)
vis.hide("traj")
vis.add("traj2",traj2.configTrajectory().discretize(0.1))
#vis.spin(float('inf'))

traj_timed = trajectory.path_to_trajectory(traj,vmax=2,amax=4)
#next, try this line instead
#traj_timed = trajectory.path_to_trajectory(traj,timing='sqrt-L2',speed='limited',vmax=2,amax=4)
#or this line
#traj_timed = trajectory.path_to_trajectory(traj2.configTrajectory().discretize(0.1),timing='sqrt-L2',speed=0.3)
vis.animate("point",traj_timed)
vis.spin(float('inf'))