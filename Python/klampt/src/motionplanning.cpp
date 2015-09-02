#include "motionplanning.h"
#include <planning/AnyMotionPlanner.h>
#include "pyerr.h"
#include <graph/IO.h>
#include <math/random.h>
#include <Python.h>
#include <iostream>
#include <fstream>
#include <exception>
#include <vector>
using namespace std;

void setRandomSeed(int seed)
{
  Math::Srand(seed);
}

PyObject* PyListFromVector(const std::vector<double>& x)
{
  PyObject* ls = PyList_New(x.size());
  PyObject* pItem;
  if(ls == NULL) {
    goto fail;
  }
	
  for(Py_ssize_t i = 0; i < PySequence_Size(ls); i++) {
    pItem = PyFloat_FromDouble(x[(int)i]);
    if(pItem == NULL)
      goto fail;
    PyList_SetItem(ls, i, pItem);
  }
  
  return ls;
  
 fail:
  Py_XDECREF(ls);
  throw PyException("Failure during PyListFromVector");
  return NULL;
}

bool PyListToVector(PyObject* seq,std::vector<double>& x)
{
  if(!PySequence_Check(seq))
    return false;
  
  x.resize(PySequence_Size(seq));
  for(Py_ssize_t i = 0; i < PySequence_Size(seq); i++) 
    x[(int)i] = PyFloat_AsDouble(PySequence_GetItem(seq, i));
  return true;
}

class PyCSpace;
class PyEdgePlanner;

/** A CSpace that calls python routines for its functionality */
class PyCSpace : public CSpace
{
public:
  PyCSpace()
    :sample(NULL),sampleNeighborhood(NULL),feasible(NULL),visible(NULL),
     distance(NULL),interpolate(NULL),edgeResolution(0.001)
  {}

  virtual ~PyCSpace() {
    Py_XDECREF(sample);
    Py_XDECREF(sampleNeighborhood);
    Py_XDECREF(feasible);
    Py_XDECREF(visible);
    Py_XDECREF(distance);
    Py_XDECREF(interpolate);
  }

  void operator = (const PyCSpace& rhs)
  {
    sample = rhs.sample;
    sampleNeighborhood = rhs.sampleNeighborhood;
    feasible = rhs.feasible;
    visible = rhs.visible;
    distance = rhs.distance;
    interpolate = rhs.interpolate;
    edgeResolution = rhs.edgeResolution;
    Py_XINCREF(sample);
    Py_XINCREF(sampleNeighborhood);
    Py_XINCREF(feasible);
    Py_XINCREF(visible);
    Py_XINCREF(distance);
    Py_XINCREF(interpolate);
  }

  virtual void Sample(Config& x) {
    if(!sample) {
      throw PyException("Python sample method not defined");
    }
    PyObject* result = PyObject_CallFunctionObjArgs(sample,NULL);
    if(!result) {
      if(!PyErr_Occurred()) {
	throw PyException("Python sample method failed");
      }
      else {
	throw PyPyErrorException();
      }
    }
    vector<Real> v;
    bool res=PyListToVector(result,v);
    x=v;
    if(!res) {
      Py_DECREF(result);
      throw PyException("Python sample method didn't return sequence");
    }
    Py_DECREF(result);
  }

  virtual void SampleNeighborhood(const Config& c,double r,Config& x)
  {
    if(!sampleNeighborhood) {
      CSpace::SampleNeighborhood(c,r,x);
    }
    else {
      PyObject* pyc=PyListFromVector(c);
      PyObject* pyr=PyFloat_FromDouble(r);
      PyObject* result = PyObject_CallFunctionObjArgs(sampleNeighborhood,pyc,pyr,NULL);
      if(!result) {
	Py_DECREF(pyc);
	Py_DECREF(pyr);
	if(!PyErr_Occurred()) {
	  throw PyException("Python sampleneighborhood method failed");
	}
	else {
	  throw PyPyErrorException();
	}
      }
      vector<Real> v;
      bool res=PyListToVector(result,v);
      x=v;
      if(!res) {
	Py_DECREF(pyc);
	Py_DECREF(pyr);
	Py_DECREF(result);
	throw PyException("Python sampleNeighborhood method did not return a list");
      }
      Py_DECREF(pyc);
      Py_DECREF(pyr);
      Py_DECREF(result);
    }
  }

  virtual bool IsFeasible(const Config& x)
  {
    if(!feasible) {
      throw PyException("Python feasible method not defined");
    }
    PyObject* pyx = PyListFromVector(x);
    PyObject* result = PyObject_CallFunctionObjArgs(feasible,pyx,NULL);
    if(result == NULL) {
      Py_DECREF(pyx);
      if(!PyErr_Occurred()) {
	throw PyException("An error occurred when calling feasible");
      }
      else {
	throw PyPyErrorException();
      }
    }
    if(!PyBool_Check(result)) {
      Py_DECREF(pyx);
      Py_DECREF(result);
      throw PyException("Python feasible method didn't return bool");
    }
    bool res=(result == Py_True);
    Py_DECREF(pyx);
    Py_DECREF(result);
    return res;
  }

  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);

  virtual double Distance(const Config& x, const Config& y)
  {
    if(!distance) {
      return CSpace::Distance(x,y);
    }
    else {
      PyObject* args = PyTuple_New(2);
      PyTuple_SetItem(args, 0, PyListFromVector(x));
      PyTuple_SetItem(args, 1, PyListFromVector(y));
      PyObject* result = PyObject_CallObject(distance,args);
      if(!result) {
	Py_DECREF(args);
	if(!PyErr_Occurred()) {
	  throw PyException("Python distance method failed");
	}
	else {
	  throw PyPyErrorException();
	}
      }
      if(!PyFloat_Check(result)) {
	Py_DECREF(args);
	Py_DECREF(result);
	throw PyException("Python distance didn't return float");
      }
      double res=PyFloat_AsDouble(result);
      Py_DECREF(args);
      Py_DECREF(result);
      return res;
    }
  }
  virtual void Interpolate(const Config& x,const Config& y,double u,Config& out)
  {
    if(!interpolate) {
      CSpace::Interpolate(x,y,u,out);
    }
    else {
      PyObject* args = PyTuple_New(3);
      PyTuple_SetItem(args, 0, PyListFromVector(x));
      PyTuple_SetItem(args, 1, PyListFromVector(y));
      PyTuple_SetItem(args, 2, PyFloat_FromDouble(u));
      PyObject* result = PyObject_CallObject(interpolate,args);
      if(!result) {
	Py_DECREF(args);
	if(!PyErr_Occurred()) {
	  throw PyException("Python interpolate method failed");
	}
	else {
	  throw PyPyErrorException();
	}
      }
      vector<Real> v;
      bool res=PyListToVector(result,v);
      out=v;
      if(!res) {
	Py_DECREF(args);
	Py_DECREF(result);
	throw PyException("Python interpolate method did not return a list");
      }
      Py_DECREF(args);
      Py_DECREF(result);
    }
  }

  virtual void Properties(PropertyMap& props) const
  {
    props = properties;
    if(!distance) {
      props.set("euclidean",1);
      props.set("metric","euclidean");
      if(!interpolate)
	props.set("geodesic",1);
    }
  }

  PyObject *sample,
    *sampleNeighborhood,
    *feasible,
    *visible,
    *distance,
    *interpolate;
  double edgeResolution;
  PropertyMap properties;
};

class PyEdgePlanner : public EdgePlanner
{
public:
  PyCSpace* space;
  Config a;
  Config b;

  PyEdgePlanner(PyCSpace* _space,const Config& _a,const Config& _b)
    :space(_space),a(_a),b(_b)
  {}
  virtual ~PyEdgePlanner() {}
  virtual bool IsVisible() {
    assert(space->visible!=false);
    PyObject* args = PyTuple_New(2);
    PyTuple_SetItem(args, 0, PyListFromVector(a));
    PyTuple_SetItem(args, 1, PyListFromVector(b));
    PyObject* result = PyObject_CallObject(space->visible,args);
    if(!result) {
      Py_DECREF(args);
      if(!PyErr_Occurred()) {
	throw PyException("Python visible method failed");
      }
      else {
	throw PyPyErrorException();
      }
    }
    if(!PyBool_Check(result)) {
      Py_DECREF(args);
      Py_DECREF(result);
      throw PyException("Python visible didn't return bool");
    }
    bool res=(result == Py_True);
    Py_DECREF(args);
    Py_DECREF(result);
    return res;
  }
  virtual void Eval(double u,Config& x) const
  {
    return space->Interpolate(a,b,u,x);
  }
  virtual const Config& Start() const { return a; }
  virtual const Config& Goal() const { return b; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const { return new PyEdgePlanner(space,a,b); }
  virtual EdgePlanner* ReverseCopy() const { return new PyEdgePlanner(space,b,a); }
};


EdgePlanner* PyCSpace::LocalPlanner(const Config& a,const Config& b)
{
  if(!visible) {
    return new StraightLineEpsilonPlanner(this,a,b,edgeResolution); 
  }
  else {
    return new PyEdgePlanner(this,a,b);
  }
}






static vector<SmartPointer<PyCSpace> > spaces;
static vector<SmartPointer<MotionPlannerInterface> > plans;
static MotionPlannerFactory factory;
static list<int> spacesDeleteList;
static list<int> plansDeleteList;

int makeNewCSpace()
{
  if(spacesDeleteList.empty()) {
    spaces.push_back(new PyCSpace);
    return (int)(spaces.size()-1);
  }
  else {
    int index = spacesDeleteList.front();
    spacesDeleteList.erase(spacesDeleteList.begin());
    spaces[index] = new PyCSpace;
    return index;
  }
}

void destroyCSpace(int cspace)
{
  if(cspace < 0 || cspace >= (int)spaces.size()) 
    throw PyException("Invalid cspace index");
  spaces[cspace] = NULL;
  spacesDeleteList.push_back(cspace);
}

CSpaceInterface::CSpaceInterface()
{
  index = makeNewCSpace();
}

CSpaceInterface::CSpaceInterface(const CSpaceInterface& space)
{
  index = makeNewCSpace();
  *spaces[index] = *spaces[space.index];
}

CSpaceInterface::~CSpaceInterface()
{
  destroy();
}

void CSpaceInterface::destroy()
{
  if(index >= 0) {
    destroyCSpace(index);
    index = -1;
  }
}

void CSpaceInterface::setFeasibility(PyObject* pyFeas)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Py_XDECREF(spaces[index]->feasible);
  Py_XINCREF(pyFeas);
  spaces[index]->feasible = pyFeas;
}

void CSpaceInterface::setVisibility(PyObject* pyVisible)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Py_XDECREF(spaces[index]->visible);
  Py_XINCREF(pyVisible);
  spaces[index]->visible = pyVisible;
}

void CSpaceInterface::setVisibilityEpsilon(double eps)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  if(eps <= 0) 
    throw PyException("Invalid epsilon");
  Py_XDECREF(spaces[index]->visible);
  spaces[index]->visible = NULL;
  spaces[index]->edgeResolution = eps;
}

void CSpaceInterface::setSampler(PyObject* pySamp)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Py_XDECREF(spaces[index]->sample);
  Py_XINCREF(pySamp);
  spaces[index]->sample = pySamp;
}

void CSpaceInterface::setNeighborhoodSampler(PyObject* pySamp)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Py_XDECREF(spaces[index]->sampleNeighborhood);
  Py_XINCREF(pySamp);
  spaces[index]->sampleNeighborhood = pySamp;
}

void CSpaceInterface::setDistance(PyObject* pyDist)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Py_XDECREF(spaces[index]->distance);
  Py_XINCREF(pyDist);
  spaces[index]->distance = pyDist;
}

void CSpaceInterface::setInterpolate(PyObject* pyInterp)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Py_XDECREF(spaces[index]->interpolate);
  Py_XINCREF(pyInterp);
  spaces[index]->interpolate = pyInterp;
}

void CSpaceInterface::setProperty(const char* key,const char* value)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  spaces[index]->properties[key] = value;
}

void setPlanJSONString(const char* string)
{
  if(!factory.LoadJSON(string))
    throw PyException("Invalid JSON string");
}

std::string getPlanJSONString()
{
  return factory.SaveJSON();
}

void setPlanType(const char* type)
{
  factory.type = type;
}

void setPlanSetting(const char* setting,double value)
{
  //printf("Setting factory setting %s to %g\n",setting,value);
  if(0==strcmp(setting,"knn")) 
    factory.knn = (int)value;
  else if(0==strcmp(setting,"connectionThreshold"))
    factory.connectionThreshold = value;
  else if(0==strcmp(setting,"perturbationRadius"))
    factory.perturbationRadius = value;
  else if(0==strcmp(setting,"bidirectional"))
    factory.bidirectional = (bool)(int)(value);
  else if(0==strcmp(setting,"grid"))
    factory.useGrid = (bool)(int)(value);
  else if(0==strcmp(setting,"gridResolution"))
    factory.gridResolution = value;
  else if(0==strcmp(setting,"suboptimalityFactor")) 
    factory.suboptimalityFactor = value;
  else if(0==strcmp(setting,"ignoreConnectedComponents")) 
    factory.ignoreConnectedComponents = (bool)(int)(value);
  else if(0==strcmp(setting,"randomizeFrequency"))
    factory.randomizeFrequency = (int)value;
  else if(0==strcmp(setting,"shortcut"))
    factory.shortcut = (value != 0);
  else if(0==strcmp(setting,"restart"))
    factory.restart = (value != 0);
  else {
    throw PyException("Invalid setting");
  }
}

void setPlanSetting(const char* setting,const char* value)
{
  if(0==strcmp(setting,"pointLocation"))
    factory.pointLocation = value;
  else if(0==strcmp(setting,"restartTermCond"))
    factory.restartTermCond = value;
  else {
    throw PyException("Invalid setting");
  }
}


int makeNewPlan(int cspace)
{
  if(cspace < 0 || cspace >= (int)spaces.size() || spaces[cspace]==NULL) 
    throw PyException("Invalid cspace index");
  if(plansDeleteList.empty()) {
    plans.push_back(factory.Create(spaces[cspace]));
    return (int)plans.size()-1;
  }
  else {
    int index = plansDeleteList.front();
    plansDeleteList.erase(plansDeleteList.begin());
    plans[index] = factory.Create(spaces[cspace]);
    return index;
  }
}

void destroyPlan(int plan)
{
  if(plan < 0 || plan >= (int)plans.size() || plans[plan]==NULL) 
    throw PyException("Invalid plan index");
  plans[plan] = NULL;
  plansDeleteList.push_back(plan);
}

PlannerInterface::PlannerInterface(const CSpaceInterface& cspace)
{
  index = makeNewPlan(cspace.index);
}

PlannerInterface::~PlannerInterface()
{
  destroy();
}

void PlannerInterface::destroy()
{
  if(index >= 0) {
    destroyPlan(index);
    index = -1;
  }
}

bool PlannerInterface::setEndpoints(PyObject* start,PyObject* goal)
{
  if(index < 0 || index >= (int)plans.size() || plans[index]==NULL) 
    throw PyException("Invalid plan index");
  vector<Real> qstart,qgoal;
  bool res=PyListToVector(start,qstart);
  if(!res) 
    throw PyException("Invalid start endpoint");
  res=PyListToVector(goal,qgoal);
  if(!res) 
    throw PyException("Invalid start endpoint");
  int istart=plans[index]->AddMilestone(qstart);
  int igoal=plans[index]->AddMilestone(qgoal);
  if(istart < 0) {
    throw PyException("Start configuration is infeasible");
  }
  if(igoal < 0) {
    throw PyException("Goal configuration is infeasible");
  }
  if(istart != 0 || igoal != 1) {
    printf("Start milestone %d, goal milestone %d\n",istart,igoal);
    throw PyException("Plan already initialized");
  }
  return true;
}

int PlannerInterface::addMilestone(PyObject* milestone)
{
  if(index < 0 || index >= (int)plans.size() || plans[index]==NULL) 
    throw PyException("Invalid plan index");
  vector<Real> q;
  bool res=PyListToVector(milestone,q);
  if(!res) 
    throw PyException("Invalid start endpoint");
  int mindex=plans[index]->AddMilestone(q);
  return mindex;
}

void DumpPlan(MotionPlannerInterface* planner,const char* fn)
{
  RoadmapPlanner prm(NULL);
  planner->GetRoadmap(prm);
  
  Graph::Graph<string,string> Gstr;
  Graph::NodesToStrings(prm.roadmap,Gstr);
  
  ofstream out(fn);
  Graph::Save_TGF(out,Gstr);
  out.close();
}

void PlannerInterface::planMore(int iterations)
{
  if(index < 0 || index >= (int)plans.size() || plans[index]==NULL) 
    throw PyException("Invalid plan index");
  plans[index]->PlanMore(iterations);
  //printf("Plan now has %d milestones, %d components\n",plans[plan]->NumMilestones(),plans[plan]->NumComponents());
  //DumpPlan(plans[plan],"plan.tgf");
}

PyObject* PlannerInterface::getPathEndpoints()
{
  if(index < 0 || index >= (int)plans.size() || plans[index]==NULL) 
    throw PyException("Invalid plan index");  
  if(!plans[index]->IsSolved()) {
    Py_RETURN_NONE;
  }
  MilestonePath path;
  plans[index]->GetSolution(path);
  PyObject* pypath = PyList_New(path.NumMilestones());
  for(int i=0;i<path.NumMilestones();i++)
    PyList_SetItem(pypath,(Py_ssize_t)i,PyListFromVector(path.GetMilestone(i)));
  return pypath;
}

PyObject* PlannerInterface::getPath(int milestone1,int milestone2)
{
  if(index < 0 || index >= (int)plans.size() || plans[index]==NULL) 
    throw PyException("Invalid plan index");  
  if(!plans[index]->IsConnected(milestone1,milestone2)) {
    Py_RETURN_NONE;
  }
  MilestonePath path;
  plans[index]->GetPath(milestone1,milestone2,path);
  PyObject* pypath = PyList_New(path.NumMilestones());
  for(int i=0;i<path.NumMilestones();i++)
    PyList_SetItem(pypath,(Py_ssize_t)i,PyListFromVector(path.GetMilestone(i)));
  return pypath;
}


double PlannerInterface::getData(const char* setting)
{
  if(index < 0 || index >= (int)plans.size() || plans[index]==NULL) 
    throw PyException("Invalid plan index");  
  if(0==strcmp(setting,"iterations")) {
    return plans[index]->NumIterations();
  }
  else if(0==strcmp(setting,"milestones")) {
    return plans[index]->NumMilestones();
  }
  else if(0==strcmp(setting,"components")) {
    return plans[index]->NumComponents();
  }
  else {
    throw PyException("Invalid plan option");
    return 0;
  }
}

PyObject* PlannerInterface::getStats()
{
  if(index < 0 || index >= (int)plans.size() || plans[index]==NULL) 
    throw PyException("Invalid plan index");  
  PropertyMap stats;
  plans[index]->GetStats(stats);
  PyObject* res = PyDict_New();
  for(PropertyMap::const_iterator i=stats.begin();i!=stats.end();i++) {
    PyObject* value = PyString_FromString(i->second.c_str());
    PyDict_SetItemString(res,i->first.c_str(),value);
    Py_XDECREF(value);
  }
  return res;
}

PyObject* PlannerInterface::getRoadmap()
{
  if(index < 0 || index >= (int)plans.size() || plans[index]==NULL) 
    throw PyException("Invalid plan index");
  RoadmapPlanner prm(NULL);
  plans[index]->GetRoadmap(prm);
  PyObject* pyV = PyList_New(prm.roadmap.nodes.size());
  for(size_t i=0;i<prm.roadmap.nodes.size();i++)
    PyList_SetItem(pyV,(Py_ssize_t)i,PyListFromVector(prm.roadmap.nodes[i]));
  PyObject* pyE = PyList_New(0);
  for(size_t i=0;i<prm.roadmap.nodes.size();i++) {
    RoadmapPlanner::Roadmap::Iterator e;
    for(prm.roadmap.Begin(i,e);!e.end();e++) {
      PyObject* pair = Py_BuildValue("(ii)",e.source(),e.target());
      PyList_Append(pyE,pair);
      Py_XDECREF(pair);
    }
  }
  //this steals the references
  return Py_BuildValue("NN",pyV,pyE);
}

void PlannerInterface::dump(const char* fn)
{
  if(index < 0 || index >= (int)plans.size() || plans[index]==NULL) 
    throw PyException("Invalid plan index");
  DumpPlan(plans[index],fn);
}

void destroy()
{
  spaces.resize(0);
  spacesDeleteList.resize(0);
  plans.resize(0);
  plansDeleteList.resize(0);
}
