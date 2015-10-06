# POSQ
POSQ is an efficient Extend or Steer function for sampling based motion planners.

All the propertiers are described in the IROS 2014 paper "A Novel RRT Extend Function for Efficient and Smooth Mobile Robot Motion Planning". 

The steer function can be used to generate closed-loop forward simulations or to control a real robot. We extend the control law of an existing discontinuous state feedback controller to make it usable as an RRT extend function and prove that all relevant stability properties are retained. 

![Alt text](http://www2.informatik.uni-freiburg.de/~palmieri/pages/images/reachabilityTree.png "Example Paths")


We study the properties of the new approach as extender for RRT and RRT* and compare it systematically to a spline-based approach and a large and small set of motion primitives. The results show that our approach generally produces smoother paths to the goal in less time with smaller trees. For RRT*, the approach produces also the shortest paths and achieves the lowest cost solutions when given more planning time

## Implementations

You will find the steer function in different directories implemented in different versions:
* MATLAB
* C++ implementation based on SMP library developed by Sertac Karaman (http://karaman.mit.edu/software.html). The ROS package https://github.com/srl-freiburg/srl_global_planner shows how to exploit it.
* C++ based on OMPL, altough the OMPL library does not fully allow using a steer function, we have a preliminary version of the POSQ in OMPL. You can add POSQ as extension in your OMPL version (in the directory ompl/src/ompl/extensions). See in the demo folder the file RigidBodyPlanningWithSteering.cpp for an example. Have a look also to the ompl forked in my github: https://github.com/palmieri/ompl .
* Python (Work in progress)


### Developers
Any contribution to the software is welcome. Contact the current developers for any info: 
* Luigi Palmieri (https://github.com/palmieri, palmieri(at)informatik.uni-freiburg.de), main contributor
