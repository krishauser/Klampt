# Klamp't Tutorial: Inverse Kinematics in Python

In this tutorial you will learn how to set up and solve inverse kinematics (IK) problems in software. (Klamp't apps automatically support IK constraints while interacting with the robot poser, so using them should be relatively transparent.)

Klamp't natively supports numerical IK only, which uses numerical root-finding techniques to iteratively move an initial configuration toward solving the IK constraints. Another approach is analytic IK, which analytically inverts an IK problem by solving a system of symbolic equations. Numerical IK is more versatile in that any robot, any number of objectives, and any degrees of freedom can be solved simultaneously in a unified framework, and difficulties handling zero or multiple solutions are avoided. However, it is slower than analytic IK (milliseconds rather than microseconds) and requires appropriately chosen initial configurations to avoid local minima. Nevertheless, we prefer it due to its generality and versatility in handling complex contact formations, its convenient use in planning and optimization, and its comparatively simple API.

Difficulty: intermediate

Time: 15-30 minutes

For the Python tutorial, we will start from Exercise 2 in Klampt/Python/exercises/ik. Open up ik.pdf in this folder, and read the instructions. Then run
```
python ex2.py
```
to observe the target point animating in a circle. In this tutorial we'll implement the few lines it takes to implement the IK solver.

The end effector link index, local position, and target position in the world are given to you in this function. Your job is to set up the structures needed to call the IK solver. Look through ex2.py to find the place where your code needs to go.
```
obj = model.ik.objective(robotlink,local=localpos,world=worldpos)
```
Now we need to 1) set up the solver with the robot and objectives, 2) set the initial configuration to 0 by calling robot.setConfig, and then 3) calling the solver:
```
        s = model.ik.solver(obj)

        robotlink.robot().setConfig([0]*robotlink.robot().numLinks())

        s.setMaxIters(100)
        s.setTolerance(1e-3)
        res = s.solve()
        numIter = s.lastSolveIters()
        if not res: print "IK failure!"
```
If res=True, then the robot's configuration is now set to the IK solution. If res=False, then the robot's configuration is set to the best found configuration

Alternatively, we could have used a convenience function in klampt.model.ik:
```
        res = model.ik.solve(obj)
        if not res: print "IK failure!"
```
 However, note that this will only give you the solution to the IK problem. It will not allow you to later interact directly with the solver. For example, this would mean that you would be unable to access the number of iterations used to obtain an IKSolution. Either way, though, it's pretty simple! Now replace the current return statement with:
 ```
         return robot.getConfig()
```
 hen run ex2.py and observe the results. You can also play around with the parameters and the start configuration. For example, commenting out the setConfig line uses the robot's previous configuration as the starting point of the optimization. When does this improve the results? When does this harm them?
 
 The klampt.model.ik module makes it similarly easy to set up fixed position and orientation constraints (klampt.model.ik.objective(robotlink,R=link_orientation,t=link_translation) and fixed point lists (klampt.model.ik.objective(robotlink,local=[p1,p2],world=[q1,q2]).
_Note: The Python API is slightly less powerful than the C++ API because it does not currently support sliding constraints without fixed positions. Support for those items is planned for the near future._

### Why isn't IK working?

A common cause of IK failures is local minima. Klamp't uses a numerical IK solver that iteratively minimizes the error between the current link transform and the goal. It also enforces joint limits. But this iteration can get stuck, most likely due to the joint limits interfering with progress toward the objective. The easiest partial solution for this is to just perform random restarts on the start configuration:

```
        s = model.ik.solver(obj)

        numRestarts = 100
        solved = False
        for i in xrange(numRestarts):
                s.sampleInitial()
                s.setMaxIters(100)
                s.setTolerance(1e-3)
                res = s.solve()
                if res:
                        solved=True
                        break
        if not solved: print "IK failure!"
``` 
Additionally, Klamp't has a convenience routine klampt.model.ik.solve_global that implements this same functionality in a single line.
```
        if not model.ik.solve_global(obj,iters = 100,tol=1e-3,numRestarts=100):
                print "IK failure!"
```       
For feasible objectives, this is likely to come up with a solution in just a few iterations, and not be much more expensive than a single IK solve. But, the increased robustness comes at a price: in the case of infeasible objective, this can take much longer than the standard solver to fail (correctly). By tuning the numRestarts parameter you can trade off between robustness and running time in the case of infeasible objective.

The second likely cause of failures is an incorrectly defined IK objective. The easiest way to debug this is to check the final configuration produced by the IK module. The IK solver does the best it can to satisfy your goal. If it doesn't appear to be doing what you want, then this is probably an error in defining the objective. Another way is to examine the residual vector, which gives the numerical errors on each of the constrained IK dimensions. To do so, call ik.residual(obj). At a solution, these entries should all be near zero.

Klamp't also has visualization functionality to display IK objectives. Simply call visualization.add(name,objective) (you will also want to add the world) and your constraint will be drawn on screen.