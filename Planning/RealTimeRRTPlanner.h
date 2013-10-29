#ifndef REAL_TIME_RRT_PLANNER_H
#define REAL_TIME_RRT_PLANNER_H

#include "RealTimePlanner.h"
#include <planning/MotionPlanner.h>

class RealTimeRRTPlanner : public RealTimePlannerBase
{
public:
  RealTimeRRTPlanner();
  virtual ~RealTimeRRTPlanner() {  }
  virtual void Reset(PlannerObjectiveBase* newgoal);
  Vector& MakeState(const Config& q,const Config& dq);
  Vector& MakeState(const Config& q);
  RRTPlanner::Node* TryIKExtend(RRTPlanner::Node* node,bool search=true);

  //perform lazy collision checking of the path up to n
  //ndelete allows you to check if any nodes are deleted
  //pass in nodes to check, they will be set to NULL if they lie in the
  //deleted subtree
  bool CheckPath(RRTPlanner::Node* n,vector<RRTPlanner::Node*>& ndelete,Timer& timer,Real cutoff);
  virtual int PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff);
  Real EvaluateNodePathCost(RRTPlanner::Node* n) const;

  //settings
  SmartPointer<RampCSpaceAdaptor> stateSpace;
  Real delta;
  Real smoothTime;
  Real ikSolveProbability;

  //temporary state
  int iteration;
  SmartPointer<RRTPlanner> rrt;
  vector<RRTPlanner::Node*> existingNodes;
  Vector tempV;
};

class RealTimeTreePlanner : public RealTimePlannerBase
{
public:
  struct NodeData
  {
    Real t;
    Config q,dq;

    bool reachable;
    int depth;
    Real sumPathCost,terminalCost,totalCost;
  };
  struct EdgeData
  {
    Real cost;
    SmartPointer<RampEdgePlanner> e;
  };
  typedef Graph::TreeNode<NodeData,EdgeData> Node;

  RealTimeTreePlanner();
  virtual ~RealTimeTreePlanner() {  }
  virtual void Reset(PlannerObjectiveBase* newgoal);
  Node* AddChild(Node* node,const Config& q);
  Node* AddChild(Node* node,const ParabolicRamp::ParabolicRampND& ramp);
  Node* AddChild(Node* node,const ParabolicRamp::DynamicPath& path);
  Node* AddChild(Node* node,SmartPointer<RampEdgePlanner>& e);
  //uses a local optimization to extend the tree from the given node.
  //if search is true, finds a parent node that gives a good fit, otherwise
  //adds the ik extension as a child of node.
  Node* TryIKExtend(Node* node,bool search=true);
  //finds the closest node to q (within the costBranch level set)
  Node* Closest(const Config& q,Real costBranch=Inf);
  //extends the tree towards q (within the costBranch level set)
  Node* ExtendToward(const Config& q,Real costBranch=Inf);
  //splits an edge p->n in the tree at interpolant u
  Node* SplitEdge(Node* p,Node* n,Real u);

  //perform lazy collision checking of the path up to n
  //split can be set to a pointer to retrieve the last reachable
  //node.
  bool CheckPath(Node* n,Timer& timer,Real cutoff,Node** split=NULL);
  virtual int PlanFrom(ParabolicRamp::DynamicPath& path,Real cutoff);

  //settings
  SmartPointer<RampCSpaceAdaptor> stateSpace;
  Real delta;
  Real smoothTime;
  Real ikSolveProbability;

  //temporary state
  int iteration;
  SmartPointer<Node> root;
  vector<Node*> nodes;
};

#endif
