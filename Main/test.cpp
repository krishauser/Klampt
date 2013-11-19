#include "RobotViewProgram.h"
#include <robotics/IKFunctions.h>
#include <GLdraw/TransformWidget.h>
#include <GLdraw/drawMesh.h>
#include <GLdraw/drawgeometry.h>
//#include <GL/glut.h>
#include <math/random.h>
#include <glui.h>
#include <fstream>
using namespace Math3D;
using namespace GLDraw;

enum {
  LINK_SPINNER_ID,
  LINK_LISTBOX_ID,
  LINK_VALUE_SPINNER_ID,
  DRIVER_SPINNER_ID,
  DRIVER_LISTBOX_ID,
  DRIVER_VALUE_SPINNER_ID,
  IK_UPDATE_ID,
  DRAW_EXPANDED_CHECKBOX_ID,
  PRINT_SELF_COLLISIONS_ID,
};


class JointTestProgram : public RobotViewProgram
{
public:
  //internal state
  int cur_link,cur_driver;
  vector<bool> self_colliding;

  //GUI state
  GLUI* glui;
  GLUI_Spinner* link_spinner, *link_value_spinner;
  GLUI_Listbox* link_listbox;
  GLUI_StaticText* link_info;
  GLUI_Spinner* driver_spinner, *driver_value_spinner;
  GLUI_Listbox* driver_listbox;
  GLUI_StaticText* driver_info;
  float link_value;
  float driver_value;
  int hoverLink,hoverWidget;
  Vector3 hoverPt;
  int pose_ik;
  vector<IKGoal> poseGoals;
  vector<TransformWidget> poseWidgets;
  int draw_geom,draw_bbs,draw_com,draw_frame,draw_expanded;
  int draw_self_collision_tests;

  vector<GLDisplayList> originalDisplayLists,expandedDisplayLists;

  JointTestProgram(Robot* robot,Environment* env)
    :RobotViewProgram(robot,env)
  {}

  virtual bool Initialize()
  {
    cur_link=0;
    cur_driver=0;
    link_value = 0;
    driver_value = 0;
    draw_geom = 1;
    draw_bbs = 0;
    draw_com = 0;
    draw_frame = 0;
    draw_expanded = 0;
    draw_self_collision_tests = 0;
    hoverLink=hoverWidget=-1;
    hoverPt.setZero();
    pose_ik = 0;
    if(!RobotViewProgram::Initialize()) return false;
    self_colliding.resize(robot->links.size(),false);   
    UpdateConfig();

    /*
    //TEST: robot-to-robot IK test.  only works for AL5Dx2
    IKGoal test;
    test.link = 8;
    test.destLink = 16;
    test.localPosition.set(0,0,0.05);
    test.endPosition.set(0,0,0.05);
    //test.SetFixedPosition(test.endPosition);
    Matrix3 R;
    R.setRotateZ(120);
    test.SetFixedRotation(R);

    vector<IKGoal> problem(1,test);
    int iters=100;
    bool res=SolveIK(*robot,problem,1e-3,iters);
    printf("Solved IK: %d, %d iters, error %g\n",(int)res,iters,RobotIKError(*robot,test));
    UpdateConfig();
    */

    //setup GUI 
    glui = GLUI_Master.create_glui_subwindow(main_window,GLUI_SUBWINDOW_RIGHT);
    glui->set_main_gfx_window(main_window);
    GLUI_Panel* panel = glui->add_panel("Link controls");
    link_spinner = glui->add_spinner_to_panel(panel,"Index",GLUI_SPINNER_INT,&cur_link,LINK_SPINNER_ID,ControlFunc);
    link_spinner->set_int_limits(0,robot->links.size()-1,GLUI_LIMIT_WRAP);

    link_listbox = glui->add_listbox_to_panel(panel,"Name",&cur_link,LINK_LISTBOX_ID,ControlFunc);
    for(size_t i=0;i<robot->links.size();i++) {
      char buf[256];
      strcpy(buf,robot->linkNames[i].c_str());
      link_listbox->add_item(i,buf);
    }

    link_value_spinner = glui->add_spinner_to_panel(panel,"Angle",GLUI_SPINNER_FLOAT,&link_value,LINK_VALUE_SPINNER_ID,ControlFunc);
    link_info = glui->add_statictext_to_panel(panel,"Info");
    UpdateLinkValueGUI();
    UpdateLinkInfoGUI();

    panel = glui->add_panel("Driver controls");
    glui->add_checkbox_to_panel(panel,"Pose by IK",&pose_ik);
    driver_spinner = glui->add_spinner_to_panel(panel,"Index",GLUI_SPINNER_INT,&cur_driver,DRIVER_SPINNER_ID,ControlFunc);
    driver_spinner->set_int_limits(0,(int)robot->drivers.size()-1,GLUI_LIMIT_WRAP);

    driver_listbox = glui->add_listbox_to_panel(panel,"Name",&cur_driver,DRIVER_LISTBOX_ID,ControlFunc);
    for(size_t i=0;i<robot->drivers.size();i++) {
      char buf[256];
      strcpy(buf,robot->driverNames[i].c_str());
      driver_listbox->add_item(i,buf);
    }

    driver_value_spinner = glui->add_spinner_to_panel(panel,"Angle",GLUI_SPINNER_FLOAT,&driver_value,DRIVER_VALUE_SPINNER_ID,ControlFunc);
    driver_info = glui->add_statictext_to_panel(panel,"Info");

    glui->add_checkbox("Draw geometry",&draw_geom);
    glui->add_checkbox("Draw COM",&draw_com);
    glui->add_checkbox("Draw frame",&draw_frame);
    glui->add_checkbox("Draw bboxes",&draw_bbs);
    glui->add_checkbox("Draw expanded",&draw_expanded,DRAW_EXPANDED_CHECKBOX_ID,ControlFunc);
    glui->add_checkbox("Draw collision tests",&draw_self_collision_tests);
    glui->add_button("Print self colliding links",PRINT_SELF_COLLISIONS_ID,ControlFunc);
    UpdateDriverValueGUI();
    UpdateDriverInfoGUI();
    return true;
  }
  
  void UpdateConfig()
  {
    robot->UpdateFrames();

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
  }

  void UpdateLinkInfoGUI()
  {
    char buf[256];
    sprintf(buf,"V [%g %g], T [%g,%g]",robot->velMin(cur_link),robot->velMax(cur_link),-robot->torqueMax(cur_link),robot->torqueMax(cur_link));
    link_info->set_text(buf);
  }

  void UpdateLinkSpinnerGUI()
  {
    link_spinner->set_int_val(cur_link);
    UpdateLinkInfoGUI();
    //driver_spinner->set_int_val(cur_driver);
  }

  void UpdateLinkListboxGUI()
  {
    link_listbox->set_int_val(cur_link);
    UpdateLinkInfoGUI();
    //driver_listbox->set_int_val(cur_driver);
  }

  void UpdateCurLinkGUI()
  {
    UpdateLinkSpinnerGUI();
    UpdateLinkListboxGUI();
  }

  void UpdateLinkValueGUI()
  {
    Vector2 limits(robot->qMin(cur_link),robot->qMax(cur_link));
    link_value_spinner->set_float_limits(limits.x,limits.y);
    link_value = robot->q(cur_link);
  }

  void UpdateDriverInfoGUI()
  {
    if(cur_driver < 0 || cur_driver >= (int)robot->drivers.size()) return;
    char buf[256];
    sprintf(buf,"V [%g %g], T [%g,%g], PID %g,%g,%g",robot->drivers[cur_driver].vmin,robot->drivers[cur_driver].vmax,robot->drivers[cur_driver].tmin,robot->drivers[cur_driver].tmax,robot->drivers[cur_driver].servoP,robot->drivers[cur_driver].servoI,robot->drivers[cur_driver].servoD);
    driver_info->set_text(buf);
  }

  void UpdateDriverSpinnerGUI()
  {
    driver_spinner->set_int_val(cur_driver);
    //driver_spinner->set_int_val(cur_driver);
    UpdateDriverInfoGUI();
  }

  void UpdateDriverListboxGUI()
  {
    driver_listbox->set_int_val(cur_driver);
    //driver_listbox->set_int_val(cur_driver);
    UpdateDriverInfoGUI();
  }

  void UpdateCurDriverGUI()
  {
    UpdateDriverSpinnerGUI();
    UpdateDriverListboxGUI();
  }

  void UpdateDriverValueGUI()
  {
    if(cur_driver < 0 || cur_driver >= (int)robot->drivers.size()) return;
    Vector2 limits = robot->GetDriverLimits(cur_driver);
    driver_value_spinner->set_float_limits(limits.x,limits.y);
    driver_value = robot->GetDriverValue(cur_driver);
  }

  virtual void RenderWorld()
  {
    if(env) {
      glEnable(GL_LIGHTING);
      viewEnv.Draw(env);
      glDisable(GL_LIGHTING);
    }
    //drawCoords(0.1);
    //viewRobot.DrawJointCoords();

   
    if(draw_geom) {
      //set the robot colors
      GLColor hover(0.75,0.75,0);
      GLColor highlight(1,1,0);
      GLColor driven(1,0.5,0);
      GLColor colliding(1,0,0);
      viewRobot.SetGrey();
      for(size_t i=0;i<robot->links.size();i++) {
	if(self_colliding[i]) viewRobot.SetColor(i,colliding);
	if((int)i == hoverLink)
	  viewRobot.SetColor(i,hover); 
	else if((int)i == cur_link)
	  viewRobot.SetColor(i,highlight); 
	else if(cur_driver >= 0 && cur_driver < (int)robot->drivers.size() &&
		robot->DoesDriverAffect(cur_driver,i))
	  viewRobot.SetColor(i,driven); 
      }
      viewRobot.Draw();
    }
    if(draw_com) {
      viewRobot.DrawCenterOfMass();
      for(size_t i=0;i<robot->links.size();i++)
	viewRobot.DrawLinkCenterOfMass(i,0.01);
    }
    if(draw_frame) {
      viewRobot.DrawLinkFrames();
      glDisable(GL_DEPTH_TEST);
      glPushMatrix();
      glMultMatrix((Matrix4)robot->links[cur_link].T_World);
      drawCoords(0.2);
      glPopMatrix();
      glEnable(GL_DEPTH_TEST);
    }
    if(draw_self_collision_tests) {
      glDisable(GL_LIGHTING);
      glLineWidth(2.0);
      glColor3f(1,0,0);
      glBegin(GL_LINES);
      for(size_t i=0;i<robot->links.size();i++) {
	Vector3 comi = robot->links[i].T_World*robot->links[i].com;
	for(size_t j=0;j<robot->links.size();j++) {
	  if(robot->selfCollisions(i,j)) {
	    Vector3 comj = robot->links[j].T_World*robot->links[j].com;
	    glVertex3v(comi);
	    glVertex3v(comj);
	  }
	}
      }
      glEnd();
    }
    if(draw_bbs) {
      for(size_t j=0;j<robot->geometry.size();j++) {
	if(robot->geometry[j].Empty()) continue;
	Box3D bbox = robot->geometry[j].GetBB();
	Matrix4 basis;
	bbox.getBasis(basis);
	glColor3f(1,0,0);
	drawOrientedWireBox(bbox.dims.x,bbox.dims.y,bbox.dims.z,basis);
      }
    }
    if(!poseGoals.empty()) {
      glPolygonOffset(0,-1000);
      glEnable(GL_POLYGON_OFFSET_FILL);
      for(size_t i=0;i<poseGoals.size();i++) {
	Vector3 curpos = robot->links[poseGoals[i].link].T_World*poseGoals[i].localPosition;
	Vector3 despos = poseGoals[i].endPosition;
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
  }

  virtual void Handle_Control(int id)
  {
    switch(id) {
    case LINK_SPINNER_ID:
      UpdateLinkListboxGUI();
      UpdateLinkValueGUI();
      GLUI_Master.sync_live_all();
      break;
    case LINK_LISTBOX_ID:
      UpdateLinkSpinnerGUI();
      UpdateLinkValueGUI();
      GLUI_Master.sync_live_all();
      break;
    case LINK_VALUE_SPINNER_ID:
      robot->q(cur_link)=link_value;
      UpdateConfig();
      if(cur_driver>=0 && cur_driver<(int)robot->drivers.size() &&
	 robot->DoesDriverAffect(cur_driver,cur_link)) {
	UpdateDriverValueGUI();
	GLUI_Master.sync_live_all();
      }
      break;
    case DRIVER_SPINNER_ID:
      UpdateDriverListboxGUI();
      UpdateDriverValueGUI();
      GLUI_Master.sync_live_all();
      break;
    case DRIVER_LISTBOX_ID:
      UpdateDriverSpinnerGUI();
      UpdateDriverValueGUI();
      GLUI_Master.sync_live_all();
      break;
    case DRIVER_VALUE_SPINNER_ID:
      if(cur_driver>=0 && cur_driver<(int)robot->drivers.size()) {
	robot->SetDriverValue(cur_driver,driver_value);
	UpdateConfig();
	if(robot->DoesDriverAffect(cur_driver,cur_link)) {
	  UpdateLinkValueGUI();
	  GLUI_Master.sync_live_all();
	}
      }
      break;
    case DRAW_EXPANDED_CHECKBOX_ID:
      ToggleDrawExpandedCheckbox();
      break;
    case PRINT_SELF_COLLISIONS_ID:
      {
	printf("Self-colliding:\n");
	for(size_t i=0;i<robot->links.size();i++) {
	  if(self_colliding[i]) {
	    bool printed=false;
	    for(size_t j=i;j<robot->links.size();j++) {
	      if(self_colliding[j] && robot->SelfCollision(i,j)) {
		printf("%d %d\t",i,j);
		printed = true;
	      }
	    }
	    if(printed) printf("\n");
	  }
	}
      }
      break;
    }
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    switch(key) {
    case 'h':
      printf("Help:\n");
      printf("[space]: next link\n");
      printf("z: previous link\n");
      printf("+: increase joint value by 0.1\n");
      printf("-: decrease joint value by 0.1\n");
      printf("d: in pose-by-IK mode, delete constraint\n");
      printf("c: in pose-by-IK mode, constrain current rotations\n");
      printf("p: print the current configuration\n");
      break;
    case ' ':
      cur_link++;
      if(cur_link >= (int)robot->links.size()) cur_link=0;
      UpdateCurLinkGUI();
      UpdateLinkValueGUI();
      break;
    case 'z':
      cur_link--;
      if(cur_link < 0) cur_link = robot->links.size()-1;
      UpdateCurLinkGUI();
      UpdateLinkValueGUI();
      break;
    case '+':
    case '=':
      {
	Real val = robot->q(cur_link)+0.1;
	robot->q(cur_link) = Clamp(val,robot->qMin(cur_link),robot->qMin(cur_link));
	UpdateConfig();
	UpdateLinkValueGUI();
      }
      break;
    case '-':
    case '_':
      {
	Real val = robot->q(cur_link)-0.1;
	robot->q(cur_link) = Clamp(val,robot->qMin(cur_link),robot->qMin(cur_link));
	UpdateConfig();
	UpdateLinkValueGUI();
      }
      break;
    case 'c':
      if(true || pose_ik) {
	if(hoverLink < 0) {
	  printf("Before constraining a link you need to hover over it\n");
	}
	else {
	  for(size_t i=0;i<poseGoals.size();i++)
	    if(poseGoals[i].link == hoverLink) {
	      poseGoals.erase(poseGoals.begin()+i);
	      poseWidgets.erase(poseWidgets.begin()+i);
	      break;
	    }
	  printf("Fixing link %s\n",robot->LinkName(hoverLink).c_str());
	  poseGoals.resize(poseGoals.size()+1);
	  poseGoals.back().link = hoverLink;
	  poseGoals.back().localPosition = robot->links[hoverLink].com;
	  poseGoals.back().SetFixedPosition(robot->links[hoverLink].T_World*robot->links[hoverLink].com);
	  poseGoals.back().SetFixedRotation(robot->links[hoverLink].T_World.R);
	  poseWidgets.resize(poseWidgets.size()+1);
	  poseWidgets.back().T.R = robot->links[hoverLink].T_World.R;
	  poseWidgets.back().T.t = poseGoals.back().endPosition;
	}
      }
      break;
    case 'd':
      if(hoverWidget != -1) {
	printf("Deleting IK goal on link %s\n",robot->LinkName(poseGoals[hoverWidget].link).c_str());
	poseGoals.erase(poseGoals.begin()+hoverWidget);
	poseWidgets.erase(poseWidgets.begin()+hoverWidget);
	hoverWidget = -1;
      }
      else {
	for(size_t i=0;i<poseGoals.size();i++)
	  if(poseGoals[i].link == hoverLink) {
	    printf("Deleting IK goal on link %s\n",robot->LinkName(hoverLink).c_str());
	    poseGoals.erase(poseGoals.begin()+i);
	    poseWidgets.erase(poseWidgets.begin()+i);
	    break;
	  }
      }
      break;
    case 'p':
      cout<<robot->q<<endl;
      break;
    }
    Refresh();
  }

  void Handle_Motion(int x, int y)
  {
    Real closestDistance=Inf;
    Ray3D r;
    ClickRay(x,y,r);

    hoverWidget = -1;
    for(size_t i=0;i<poseWidgets.size();i++) {
      Real d;
      if(poseWidgets[i].Hover(x,viewport.h-y,viewport,d)) {
	if(d < closestDistance) {
	  closestDistance = d;
	  hoverWidget = (int)i;
	}
      }
      if(poseWidgets[i].requestRedraw) {
	Refresh();
	poseWidgets[i].requestRedraw = false;
      }
    }
    if(hoverWidget >= 0) {
      poseWidgets[hoverWidget].SetHighlight(true);
      hoverLink = -1;
      Refresh();
    }
    else {
      if(hoverWidget >= 0) {
	poseWidgets[hoverWidget].SetHighlight(false);
	Refresh();
      }
      hoverWidget = -1;
      int oldHoverLink = hoverLink;
      ClickRobot(r,hoverLink,hoverPt);
      if(hoverLink != oldHoverLink) Refresh();
    }
  }

  virtual void BeginDrag(int x,int y,int button,int modifiers)
  {
    if(button == GLUT_RIGHT_BUTTON) {
      if(hoverWidget >= 0) {
	Real d;
	if(poseWidgets[hoverWidget].BeginDrag(x,viewport.y+viewport.h-y,viewport,d)) {
	  poseWidgets[hoverWidget].SetFocus(true);
	}
	else {
	  //doesn't want focus -- weird?
	}
      }
      else if(pose_ik && hoverLink >= 0) {
	//clicked on a new point 
	poseGoals.resize(poseGoals.size()+1);
	poseGoals.back().link = hoverLink;
	poseGoals.back().localPosition = hoverPt ;
	poseGoals.back().SetFixedPosition(robot->links[hoverLink].T_World*hoverPt);
	poseWidgets.resize(poseWidgets.size()+1);
	poseWidgets.back().T.R = robot->links[hoverLink].T_World.R;
	poseWidgets.back().T.t = robot->links[hoverLink].T_World*hoverPt;
	poseWidgets.back().enableRotation = false;
	poseWidgets.back().SetHighlight(true);
	poseWidgets.back().SetFocus(true);
	Real d;
	poseWidgets.back().Hover(x,viewport.h-y,viewport,d);
	hoverWidget = (int)poseGoals.size()-1;
	hoverLink = -1;
	Refresh();
      }
    }
  }

  virtual void EndDrag(int x,int y,int button,int modifiers)
  {
    if(button == GLUT_RIGHT_BUTTON) {
      if(hoverWidget >= 0) {
	poseWidgets[hoverWidget].SetFocus(false);
      }
    }
  }

  virtual void DoFreeDrag(int dx,int dy,int button)
  {
    if(button == GLUT_LEFT_BUTTON)  DragRotate(dx,dy);
    else if(button == GLUT_RIGHT_BUTTON) {
      if(hoverWidget >= 0) {
	poseWidgets[hoverWidget].Drag(dx,-dy,viewport);
	if(poseGoals[hoverWidget].rotConstraint == IKGoal::RotFixed) {
	  poseGoals[hoverWidget].SetFixedRotation(poseWidgets[hoverWidget].T.R);
	  poseGoals[hoverWidget].SetFixedPosition(poseWidgets[hoverWidget].T.t);
	}
	else {
	  poseGoals[hoverWidget].SetFixedPosition(poseWidgets[hoverWidget].T.t);
	}

	int iters=100;
	bool res=SolveIK(*robot,poseGoals,1e-3,iters,0);
	UpdateConfig();
	UpdateLinkValueGUI();
	UpdateDriverValueGUI();
      }
      else if(!pose_ik) {
	if(hoverLink < 0) return;
	for(size_t i=0;i<robot->drivers.size();i++) {
	  if(robot->DoesDriverAffect(i,hoverLink)) {
	    Real val = Clamp(robot->GetDriverValue(i)+dy*0.02,robot->drivers[i].qmin,robot->drivers[i].qmax);
	    robot->SetDriverValue(i,val);
	  }
	}
	robot->UpdateFrames();
	UpdateConfig();
	UpdateLinkValueGUI();
	UpdateDriverValueGUI();
      }
      Refresh();
    }
  }

  void ToggleDrawExpandedCheckbox()
  {
    if(originalDisplayLists.empty()) {
      //first call -- initialize display lists
      originalDisplayLists.resize(robot->links.size());
      expandedDisplayLists.resize(robot->links.size());
      for(size_t j=0;j<robot->links.size();j++) {
	originalDisplayLists[j]=viewRobot.linkAppearance[j].faceDisplayList;
	if(!robot->geometry[j].Empty()) {
	  Real p=robot->geometry[j].margin;
	  if(p > 0) {
	    //draw the expanded mesh
	    expandedDisplayLists[j].beginCompile();
	    drawExpanded(robot->geometry[j],p);
	    expandedDisplayLists[j].endCompile();
	  }
	  else
	    expandedDisplayLists[j] = originalDisplayLists[j];
	}
      }
    }
    if(draw_expanded) {
      for(size_t j=0;j<robot->links.size();j++) {
	viewRobot.linkAppearance[j].faceDisplayList = expandedDisplayLists[j];
      }
    }
    else {
      for(size_t j=0;j<robot->links.size();j++) 
	viewRobot.linkAppearance[j].faceDisplayList = originalDisplayLists[j];
    }
  }
};


int main(int argc, char** argv)
{
  if(argc < 2) {
    printf("USAGE: RobotTest robot_file\n");
    return 0;
  }
  Robot robot;
  if(!robot.Load(argv[1])) {
    printf("Error loading robot file %s\n",argv[1]);
    return 1;
  }

  printf("Done loading robot file %s\n",argv[1]);

  JointTestProgram program(&robot,NULL);
  return program.Run();
}
