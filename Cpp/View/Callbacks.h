#ifndef ROBOT_SIM_VIEW_CALLBACKS_H
#define ROBOT_SIM_VIEW_CALLBACKS_H

#include <Robotics/IKFunctions.h>
#include <Robotics/SBLTree.h>
#include <Robotics/MotionPlanner.h>
#include <Graph/Callback.h>
#include <GLDraw/GL.h>
#include <GLDraw/drawExtra.h>

struct DrawEECallback : public Graph::CallbackBase<SBLTree::Node*>
{
  DrawEECallback(Robot& _robot,const IKGoal& _goal)
    :robot(_robot),goal(_goal)
  {
  }
  virtual bool ForwardEdge(SBLTree::Node* i,SBLTree::Node* j)
  {
    //const EdgePlannerPtr& e=j->edgeFromParent();
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    GLDraw::glVertex3v(EEPosition(*i));
    GLDraw::glVertex3v(EEPosition(*j));
    glEnd();
    return true;
  }

  virtual void Visit(SBLTree::Node* node)
  {
    nodeColor.setCurrentGL();
    glBegin(GL_POINTS);
    GLDraw::glVertex3v(EEPosition(*node));
    glEnd();
  }

  Vector3 EEPosition(const Config& q) const
  {
    robot.UpdateConfig(q);
    return robot.links[goal.link].T_World*goal.localPosition;
  }

  Robot& robot;
  IKGoal goal;
  GLDraw::GLColor nodeColor,edgeColor;
};

struct DrawEECallback2 : public Graph::CallbackBase<int>
{
  DrawEECallback2(Robot& _robot,const IKGoal& _goal,RoadmapPlanner* _prm)
    :robot(_robot),goal(_goal),prm(_prm)
  {
  }

  void DrawEdge(int i,int j)
  {
    if(i <= 1 || j <= 1) return;
    //const EdgePlannerPtr& e=j->edgeFromParent();
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    GLDraw::glVertex3v(EEPosition(prm->roadmap.nodes[i]));
    GLDraw::glVertex3v(EEPosition(prm->roadmap.nodes[j]));
    glEnd();
  }

  virtual bool ForwardEdge(int i,int j) { DrawEdge(i,j);  return true; }
  virtual void CrossEdge(int i,int j) { DrawEdge(i,j); }
  virtual void BackEdge(int i,int j) { DrawEdge(i,j); }

  virtual void Visit(int node)
  {
    if(node <= 1) return;
    nodeColor.setCurrentGL();
    glBegin(GL_POINTS);
    GLDraw::glVertex3v(EEPosition(prm->roadmap.nodes[node]));
    glEnd();
  }

  Vector3 EEPosition(const Config& q) const
  {
    robot.UpdateConfig(q);
    return robot.links[goal.link].T_World*goal.localPosition;
  }

  Robot& robot;
  IKGoal goal;
  RoadmapPlanner* prm;
  GLDraw::GLColor nodeColor,edgeColor;
};

struct DrawEECallback3 : public Graph::CallbackBase<RRTPlanner::Node*>
{
  DrawEECallback3(Robot& _robot,const IKGoal& _goal)
    :robot(_robot),goal(_goal)
  {
  }
  virtual bool ForwardEdge(RRTPlanner::Node* i,RRTPlanner::Node* j)
  {
    //const EdgePlannerPtr& e=j->edgeFromParent();
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    GLDraw::glVertex3v(EEPosition(i->x));
    GLDraw::glVertex3v(EEPosition(j->x));
    glEnd();
    return true;
  }

  virtual void Visit(RRTPlanner::Node* node)
  {
    nodeColor.setCurrentGL();
    glBegin(GL_POINTS);
    GLDraw::glVertex3v(EEPosition(node->x));
    glEnd();
  }

  Vector3 EEPosition(const State& x) const
  {
    Vector q;
    q.setRef(x,0,1,robot.links.size());
    robot.UpdateConfig(q);
    return robot.links[goal.link].T_World*goal.localPosition;
  }

  Robot& robot;
  IKGoal goal;
  GLDraw::GLColor nodeColor,edgeColor;
};

#endif
