#include "RobotPoseGUI.h"
#include <GLdraw/drawMesh.h>
#include <GLdraw/drawgeometry.h>
#include "Modeling/MultiPath.h"
#include <robotics/IKFunctions.h>
#include "Contact/Utils.h"

#include <sstream>

RobotPoseBackend::RobotPoseBackend(RobotWorld* world):
  WorldGUIBackend(world){
  settings["cleanContactsNTol"]= 0.01;
  settings["pathOptimize"]["contactTol"] = 0.05;
  settings["pathOptimize"]["outputResolution"] = 0.01; 
  settings["contact"]["forceScale"]= 0.01;
  settings["contact"]["pointSize"]= 5;
  settings["contact"]["normalLength"]= 0.05; 
  settings["linkCOMRadius"] = 0.01;
  settings["flatContactTolerance"] = 0.001;
  settings["selfCollideColor"][0] = 1;
  settings["selfCollideColor"][1] = 0;
  settings["selfCollideColor"][2] = 0;
  settings["selfCollideColor"][3] = 1;
  settings["movieWidth"] = 640;
  settings["hoverColor"] = 1;
  settings["hoverColor"] = 1;
  settings["hoverColor"] = 0;
  settings["hoverColor"] = 1;
  settings["robotColor"][0] = 0.5;
  settings["robotColor"][1] = 0.5;
  settings["robotColor"][2] = 0.5;
  settings["robotColor"][3] = 1;
  settings["movieHeight"] = 480;
  settings["envCollideColor"][0] = 1;
  settings["envCollideColor"][1] = 0.5;
  settings["envCollideColor"][2] = 0;
  settings["envCollideColor"][3] = 1;

  settings["poser"]["color"][0] = 1;
  settings["poser"]["color"][0] = 1;
  settings["poser"]["color"][0] = 0;
  settings["poser"]["color"][0] = 0.5;

  settings["defaultStanceFriction"] = 0.5;
  settings["configResourceColor"][0] = 0.5;
  settings["configResourceColor"][1] = 0.5;
  settings["configResourceColor"][2] = 0.5;
  settings["configResourceColor"][3] = 0.25;
  settings["linkFrameSize"] = 0.2;
  settings["cleanContactsXTol"] = 0.01;
}

void RobotPoseBackend::Start()
{
  if(!settings.read("robotpose.settings")) {
    printf("Didn't read settings from simtest.settings\n");
    printf("Writing default settings to simtest_default.settings\n");
    settings.write("simtest_default.settings");
  }

 WorldGUIBackend:Start();
  robot = world->robots[0].robot;
  cur_link=0;
  cur_driver=0;
  draw_geom = 1;
  draw_bbs = 0;
  draw_com = 0;
  draw_frame = 0;
  draw_expanded = 0;
  pose_ik = 0;
  self_colliding.resize(robot->links.size(),false);   

  for(size_t i=0;i<world->rigidObjects.size();i++)
    objectWidgets[i].Set(world->rigidObjects[i].object,&world->rigidObjects[i].view);
  for(size_t i=0;i<world->robots.size();i++)
    allWidgets.widgets.push_back(&robotWidgets[i]);
  for(size_t i=0;i<world->rigidObjects.size();i++)
    allWidgets.widgets.push_back(&objectWidgets[i]);
  
  poseWidget.Set(world->robots[0].robot,&world->robots[0].view);
  objectWidgets.resize(world->rigidObjects.size());
  

  self_colliding.resize(robot->links.size(),false);
  env_colliding.resize(robot->links.size(),false);

  UpdateConfig();

  MapButtonToggle("pose_ik",&pose_ik);
  MapButtonToggle("draw_geom",&draw_geom);
  MapButtonToggle("draw_bbs",&draw_bbs);
  MapButtonToggle("draw_com",&draw_com);
  MapButtonToggle("draw_frame",&draw_frame);
}

void RobotPoseBackend::UpdateConfig()
{
  for(size_t i=0;i<robotWidgets.size();i++)
    world->robots[i].robot->UpdateConfig(robotWidgets[i].Pose());

  //update collisions
  for(size_t i=0;i<robot->links.size();i++)
    self_colliding[i]=false;
  robot->UpdateGeometry();
  for(size_t i=0;i<robot->links.size();i++) {
    for(size_t j=i+1;j<robot->links.size();j++) {
      if(robot->SelfCollision(i,j)) {
	self_colliding[i]=self_colliding[j]=true;
      }
    }
  }
  for(size_t i=0;i<robot->links.size();i++) {
    for(size_t j=i+1;j<robot->links.size();j++) {
      if(robot->SelfCollision(i,j)) {
	self_colliding[i]=self_colliding[j]=true;
      }
    }
  }
  SendCommand("update_config","");
}


void RobotPoseBackend::RenderWorld()
{
  Robot* robot = world->robots[0].robot;
  ViewRobot& viewRobot = world->robots[0].view;
  //ResourceBrowserProgram::RenderWorld();
  for(size_t i=0;i<world->terrains.size();i++)
    world->terrains[i].view.Draw();
  for(size_t i=0;i<world->rigidObjects.size();i++)
    world->rigidObjects[i].view.Draw();

  if(draw_geom) {
    //set the robot colors
    GLColor robotColor(settings["robotColor"][0],settings["robotColor"][1],settings["robotColor"][2],settings["robotColor"][3]);
    GLColor highlight(settings["hoverColor"][0],settings["hoverColor"][1],settings["hoverColor"][2],settings["hoverColor"][3]);
    GLColor selfcolliding(settings["selfCollideColor"][0],settings["selfCollideColor"][1],settings["selfCollideColor"][2],settings["selfCollideColor"][3]);
    GLColor envcolliding(settings["envCollideColor"][0],settings["envCollideColor"][1],settings["envCollideColor"][2],settings["envCollideColor"][3]);

    viewRobot.SetColors(robotColor);
    for(size_t i=0;i<robot->links.size();i++) {
      if(self_colliding[i]) viewRobot.SetColor(i,selfcolliding);
      if(env_colliding[i]) viewRobot.SetColor(i,envcolliding);
      else if((int)i == cur_link)
	viewRobot.SetColor(i,highlight); 
    }
    allWidgets.DrawGL(viewport);
    viewRobot.Draw();
  }
  else {
    allWidgets.DrawGL(viewport);
  }

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  //  ResourceBrowserProgram::RenderCurResource();
  glDisable(GL_BLEND);
   
  if(draw_com) {
    viewRobot.DrawCenterOfMass();
    Real comSize = settings["linkCOMRadius"];
    for(size_t i=0;i<robot->links.size();i++)
      viewRobot.DrawLinkCenterOfMass(i,comSize);
  }
  if(draw_frame) {
    viewRobot.DrawLinkFrames();
    glDisable(GL_DEPTH_TEST);
    glPushMatrix();
    glMultMatrix((Matrix4)robot->links[cur_link].T_World);
    drawCoords(settings["linkFrameSize"]);
    glPopMatrix();
    glEnable(GL_DEPTH_TEST);
  }
  /*
    if(!poseGoals.empty()) {
    glPolygonOffset(0,-1000);
    glEnable(GL_POLYGON_OFFSET_FILL);
    for(size_t i=0;i<poseGoals.size();i++) {
    Vector3 curpos = robot->links[poseGoals[i].link].T_World*poseGoals[i].localPosition;
    Vector3 despos = poseGoals[i].endPosition;
    if(poseGoals[i].destLink >= 0)
    despos = robot->links[poseGoals[i].destLink].T_World*despos;
    glDisable(GL_LIGHTING);
    glColor3f(1,0,0);
    glLineWidth(5.0);
    glBegin(GL_LINES);
    glVertex3v(curpos);
    glVertex3v(despos);
    glEnd();
    glLineWidth(1.0);

    poseWidgets[i].DrawGL(viewport);

    float color2[4] = {1,0.5,0,1};
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,color2); 
    glPushMatrix();
    if(poseGoals[i].rotConstraint == IKGoal::RotFixed) {
    RigidTransform T;
    poseGoals[i].GetFixedGoalTransform(T);
    if(poseGoals[i].destLink >= 0)
    T = robot->links[poseGoals[i].destLink].T_World*T;
    glMultMatrix(Matrix4(T));
    drawBox(0.04,0.04,0.04);
    }
    else {
    glTranslate(despos);
    drawSphere(0.02,16,8);
    }
    glPopMatrix();
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
    }
  */
}


Stance RobotPoseBackend::GetFlatStance()
{
  Robot* robot = world->robots[0].robot;
  Stance s;
  if(poseWidget.ikPoser.poseGoals.empty()) {
    printf("Storing flat ground stance\n");
    ContactFormation cf;
    GetFlatContacts(*robot,settings["flatContactTolerance"],cf);

    Real friction = settings["defaultStanceFriction"];
    for(size_t i=0;i<cf.links.size();i++) {
      //assign default friction
      for(size_t j=0;j<cf.contacts[i].size();j++)
	cf.contacts[i][j].kFriction = friction;
      Hold h;
      LocalContactsToHold(cf.contacts[i],cf.links[i],*robot,h);
      s.insert(h);
    }
  }
  else {
    printf("Storing flat contact stance\n");
    for(size_t i=0;i<poseWidget.ikPoser.poseGoals.size();i++) {
      int link = poseWidget.ikPoser.poseGoals[i].link;
      vector<ContactPoint> cps;
      GetFlatContacts(*robot,link,settings["flatContactTolerance"],cps);
      Real friction = settings["defaultStanceFriction"];
      //assign default friction
      for(size_t j=0;j<cps.size();j++)
	cps[j].kFriction = friction;
      Hold h;
      LocalContactsToHold(cps,link,*robot,h);
      h.ikConstraint = poseWidget.ikPoser.poseGoals[i];
      s.insert(h);
    }
  }
  return s;
}

/*
RsourcePtr PoserToResource()
{
  Robot* robot = world->robots[0].robot;
  string type = resource_types[ResourceBrowserProgram::cur_resource_type];
  if(type == "Config") 
    return MakeResource("",robot->q);
  else if(type == "IKGoal") {
    int ind = poseWidget.ikPoser.ActiveWidget();
    if(ind < 0) {
      printf("Not hovering over any IK widget\n");
      return NULL;
    }
    return MakeResource("",poseWidget.ikPoser.poseGoals[ind]);
  }
  else if(type == "Stance") {
    Stance s = GetFlatStance();
    return MakeResource("",s);
  }
  else if(type == "Grasp") {
    int link = 0;
    cout<<"Which robot link to use? > "; cout.flush();
    cin >> link;
    cin.ignore(256,'\n');
    Grasp g;
    g.constraints.resize(1);
    g.constraints[0].SetFixedTransform(robot->links[link].T_World);
    g.constraints[0].link = link;
    vector<bool> descendant;
    robot->GetDescendants(link,descendant);
    for(size_t i=0;i<descendant.size();i++)
      if(descendant[i]) {
	g.fixedDofs.push_back(i);
	g.fixedValues.push_back(robot->q[i]);
      }
      
    ResourcePtr r=ResourceBrowserProgram::CurrentResource();
    const GeometricPrimitive3DResource* gr = dynamic_cast<const GeometricPrimitive3DResource*>((const ResourceBase*)r);
    if(gr) {
      cout<<"Making grasp relative to "<<gr->name<<endl;
      //TODO: detect contacts
	
      RigidTransform T = gr->data.GetFrame();
      RigidTransform Tinv;
      Tinv.setInverse(T);
      g.Transform(Tinv);
    }
    else {
      const RigidObjectResource* obj = dynamic_cast<const RigidObjectResource*>((const ResourceBase*)r);
      if(obj) {
	cout<<"Making grasp relative to "<<obj->name<<endl;
	//TODO: detect contacts
	  
	RigidTransform T = obj->object.T;
	RigidTransform Tinv;
	Tinv.setInverse(T);
	g.Transform(Tinv);
      }
    }
    return MakeResource("",g);
  }
  else {
    fprintf(stderr,"Poser does not contain items of the selected type\n");
    return NULL;
  }
}
*/
/*
void RobotPoseBackend::CleanContacts(Hold& h)
{
  Real ntol = settings["cleanContactsNTol"];
  Real xtol = settings["cleanContactsXTol"];
  CHContacts(h.contacts,ntol,xtol);
}
*/
//BUTTON HANDLING METHODS
