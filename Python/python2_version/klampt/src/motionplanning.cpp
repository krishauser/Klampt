#if defined (__APPLE__) || defined (MACOSX)
  #include "mac_fixes.h"
#endif //Mac fixes 

#include "motionplanning.h"
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <KrisLibrary/planning/CSpaceHelpers.h>
#include <KrisLibrary/planning/EdgePlannerHelpers.h>
#include <KrisLibrary/planning/Objective.h>
#include "pyerr.h"
#include "pyconvert.h"
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/graph/IO.h>
#include <KrisLibrary/Timer.h>
#include <Python.h>
#include <iostream>
#include <fstream>
#include <exception>
#include <vector>
#include <map>
using namespace std;

#if PY_MAJOR_VERSION >= 3
#define PyInt_Check PyLong_Check
#endif //PY_MAJOR_VERSION >= 3

void setRandomSeed(int seed)
{
  Math::Srand(seed);
}

void UpdateStats(AdaptiveCSpace::PredicateStats& s,double testcost,bool testtrue,double strength=1.0);


class PyCSpace;
class PyEdgePlanner;

/** A CSpace that calls python routines for its functionality */
class PyCSpace : public CSpace
{
public:
  PyCSpace()
    :sample(NULL),sampleNeighborhood(NULL),
     distance(NULL),interpolate(NULL),edgeResolution(0.001),cacheq(NULL),cacheq2(NULL),cachex(NULL),cachex2(NULL),
     visibleDistance(0),notVisibleDistance(0)
  {
    feasibleStats.cost = 0;
    feasibleStats.probability = 0.5;
    feasibleStats.count = 0;
    visibleStats.cost = 0;
    visibleStats.probability = 0.5;
    visibleStats.count = 0;
  }

  virtual ~PyCSpace() {
    Py_XDECREF(sample);
    Py_XDECREF(sampleNeighborhood);
    for(size_t i=0;i<visibleTests.size();i++)
      Py_XDECREF(visibleTests[i]);
    Py_XDECREF(distance);
    Py_XDECREF(interpolate);
    Py_XDECREF(cachex);
    Py_XDECREF(cachex2);
  }

  PyObject* UpdateTempConfig(const Config& q) {
    //PROBLEM when the values of q change, its address doesnt! we still have to re-make it
    if(&q == cacheq) return cachex;
    Py_XDECREF(cachex);
    cacheq = &q;
    cachex = PyListFromConfig(q);
    return cachex;
  }
  PyObject* UpdateTempConfig2(const Config& q) {
    //PROBLEM when the values of q change, its address doesnt! we still have to re-make it
    if(&q == cacheq2) return cachex2;
    Py_XDECREF(cachex2);
    cacheq2 = &q;
    cachex2 = PyListFromConfig(q);
    return cachex2;
  }

  int ConstraintIndex(const char* name)
  {
    for(size_t i=0;i<constraints.size();i++)
      if(constraintNames[i] == name) return (int)i;
    return -1;
  }

  void operator = (const PyCSpace& rhs)
  {
    sample = rhs.sample;
    sampleNeighborhood = rhs.sampleNeighborhood;
    visibleTests = rhs.visibleTests;
    constraints = rhs.constraints;
    constraintNames = rhs.constraintNames;
    distance = rhs.distance;
    interpolate = rhs.interpolate;
    edgeResolution = rhs.edgeResolution;
    feasibleStats = rhs.feasibleStats;
    visibleStats = rhs.visibleStats;
    visibleDistance = rhs.visibleDistance;
    notVisibleDistance = rhs.notVisibleDistance;
    Py_XINCREF(sample);
    Py_XINCREF(sampleNeighborhood);
    for(size_t i=0;i<visibleTests.size();i++)
      Py_XINCREF(visibleTests[i]);
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
    bool res=PyListToConfig(result,x);
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
      PyObject* pyc=UpdateTempConfig(c);
      PyObject* pyr=PyFloat_FromDouble(r);
      PyObject* result = PyObject_CallFunctionObjArgs(sampleNeighborhood,pyc,pyr,NULL);
      if(!result) {
        Py_DECREF(pyr);
        if(!PyErr_Occurred()) {
          throw PyException("Python sampleneighborhood method failed");
        }
        else {
          throw PyPyErrorException();
        }
      }
      bool res=PyListToConfig(result,x);
      if(!res) {
        Py_DECREF(pyr);
        Py_DECREF(result);
        throw PyException("Python sampleNeighborhood method did not return a list");
      }
      Py_DECREF(pyr);
      Py_DECREF(result);
    }
  }

  virtual bool IsFeasible(const Config& q) {
    Timer timer;
    bool res = CSpace::IsFeasible(q);
    UpdateStats(feasibleStats,timer.ElapsedTime(),res);
    return res;
  }

  virtual bool IsFeasible(const Config& q,int constraint) {
    Timer timer;
    bool res = CSpace::IsFeasible(q,constraint);
    UpdateStats(feasibleStats,timer.ElapsedTime(),res);
    return res;
  }

  virtual bool IsVisible(const Config& a,const Config& b) {
    return PathChecker(a,b)->IsVisible();
  }

  virtual bool IsVisible(const Config& a,const Config& b,int obstacle) {
    return PathChecker(a,b,obstacle)->IsVisible();
  }

  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b,int obstacle);

  virtual EdgePlannerPtr PathChecker(const Config& a,const Config& b);

  virtual double Distance(const Config& x, const Config& y)
  {
    if(!distance) {
      return CSpace::Distance(x,y);
    }
    else {
      PyObject* pyx = UpdateTempConfig(x);
      PyObject* pyy = UpdateTempConfig2(y);
      PyObject* result = PyObject_CallFunctionObjArgs(distance,pyx,pyy,NULL);
      if(!result) {
        if(!PyErr_Occurred()) {
          throw PyException("Python distance method failed");
        }
        else {
          throw PyPyErrorException();
        }
      }
      if(!PyFloat_Check(result)) {
        Py_DECREF(result);
        throw PyException("Python distance didn't return float");
      }
      double res=PyFloat_AsDouble(result);
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
      PyObject* pyx = UpdateTempConfig(x);
      PyObject* pyy = UpdateTempConfig2(y);
      PyObject* pyu = PyFloat_FromDouble(u);
      PyObject* result = PyObject_CallFunctionObjArgs(interpolate,pyx,pyy,pyu,NULL);
      Py_DECREF(pyu);
      if(!result) {
        if(!PyErr_Occurred()) {
          throw PyException("Python interpolate method failed");
        }
        else {
          throw PyPyErrorException();
        }
      }
      bool res=PyListToConfig(result,out);
      if(!res) {
        Py_DECREF(result);
        throw PyException("Python interpolate method did not return a list");
      }
      Py_DECREF(result);
    }
  }

  virtual void Properties(PropertyMap& props)
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
    *distance,
    *interpolate;
  vector<PyObject*> visibleTests;
  double edgeResolution;
  PropertyMap properties;

  const Config *cacheq,*cacheq2;
  PyObject *cachex,*cachex2;
  AdaptiveCSpace::PredicateStats feasibleStats,visibleStats;
  double visibleDistance,notVisibleDistance;
};


class PyUpdateEdgePlanner : public PiggybackEdgePlanner
{
public:
  PyCSpace* space;
  PyUpdateEdgePlanner(PyCSpace* _space,shared_ptr<EdgePlanner> e)
  :PiggybackEdgePlanner(e),space(_space)
  {}
  void UpdateCSpace(double time,bool visible) {
    UpdateStats(space->visibleStats,time,visible);
    if(visible) 
      space->visibleDistance += (Length()-space->visibleDistance)/(space->visibleStats.count*space->visibleStats.probability);
    else
      space->notVisibleDistance += (Length()-space->notVisibleDistance)/(space->visibleStats.count*(1.0-space->visibleStats.probability));
  }
  virtual bool IsVisible() {
    Timer timer;
    bool res = PiggybackEdgePlanner::IsVisible();
    UpdateCSpace(timer.ElapsedTime(),res);
    return res;
  }
};

class PyEdgePlanner : public EdgePlanner
{
public:
  PyCSpace* space;
  Config a;
  Config b;
  int obstacle;

  PyEdgePlanner(PyCSpace* _space,const Config& _a,const Config& _b,int _obstacle=-1)
    :space(_space),a(_a),b(_b),obstacle(_obstacle)
  {}
  virtual ~PyEdgePlanner() {}
  
  virtual bool IsVisible() {
    assert(space->visibleTests.size() == space->constraints.size());
    PyObject* pya = space->UpdateTempConfig(a);
    PyObject* pyb = space->UpdateTempConfig2(b);
    if(obstacle < 0) { //test all obstacles
      PyObject* args = PyTuple_New(2);
      Py_INCREF(pya);
      Py_INCREF(pyb);
      PyTuple_SetItem(args, 0, pya);
      PyTuple_SetItem(args, 1, pyb);
      for(size_t i=0;i<space->visibleTests.size();i++) {
        if(space->visibleTests[i] == NULL) {
          stringstream ss;
          ss<<"Python visible test for constraint "<<space->constraintNames[i]<<"not defined"<<endl;
          Py_DECREF(args);
          throw PyException(ss.str().c_str());
        }

        PyObject* result = PyObject_CallObject(space->visibleTests[i],args);
        if(!result) {
          Py_DECREF(pya);
          Py_DECREF(pyb);
          Py_DECREF(args);
          if(!PyErr_Occurred()) {
            throw PyException("Python visible method failed");
          }
          else {
            throw PyPyErrorException();
          }
        }
        if(!PyBool_Check(result) && !PyInt_Check(result)) {
          Py_DECREF(pya);
          Py_DECREF(pyb);
          Py_DECREF(args);
          Py_DECREF(result);
          throw PyException("Python visible test didn't return bool");
        }
        int res=PyObject_IsTrue(result);
        Py_DECREF(result);
        if(res != 1) {
          Py_DECREF(pya);
          Py_DECREF(pyb);
          Py_DECREF(args);
          return false;
        }
      }
      Py_DECREF(pya);
      Py_DECREF(pyb);
      Py_DECREF(args);
    }
    else {
      //call visibility test for one obstacle
      if(space->visibleTests[obstacle] == NULL) {
        stringstream ss;
        ss<<"Python visible test for constraint "<<space->constraintNames[obstacle]<<"not defined"<<endl;
        throw PyException(ss.str().c_str());
      }

      PyObject* result = PyObject_CallFunctionObjArgs(space->visibleTests[obstacle],pya,pyb,NULL);
      if(!result) {
        if(!PyErr_Occurred()) {
          throw PyException("Python visible method failed");
        }
        else {
          throw PyPyErrorException();
        }
      }
      if(!PyBool_Check(result) && !PyInt_Check(result)) {
        Py_DECREF(result);
        throw PyException("Python visible test didn't return bool");
      }
      int res=PyObject_IsTrue(result);
      Py_DECREF(result);
      if(res != 1) {
        return false;
      }
    }
    return true;
  }
  virtual Real Length() const { return space->Distance(a,b); }
  virtual void Eval(double u,Config& x) const
  {
    return space->Interpolate(a,b,u,x);
  }

  virtual const Config& Start() const { return a; }
  virtual const Config& End() const { return b; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlannerPtr Copy() const {
    return make_shared<PyEdgePlanner>(space,a,b,obstacle); 
  }
  virtual EdgePlannerPtr ReverseCopy() const {
    return make_shared<PyEdgePlanner>(space,b,a,obstacle);
  }
};


EdgePlannerPtr PyCSpace::PathChecker(const Config& a,const Config& b)
{
  if(visibleTests.empty()) {
    return make_shared<PyUpdateEdgePlanner>(this,make_shared<BisectionEpsilonEdgePlanner>(this,a,b,edgeResolution)); 
  }
  else {
    return make_shared<PyUpdateEdgePlanner>(this,make_shared<PyEdgePlanner>(this,a,b));
  }
}

EdgePlannerPtr PyCSpace::PathChecker(const Config& a,const Config& b,int obstacle)
{
  if(visibleTests.empty()) {
    return make_shared<PyUpdateEdgePlanner>(this,MakeSingleConstraintBisectionPlanner(this,a,b,obstacle,edgeResolution)); 
  }
  else {
    return make_shared<PyUpdateEdgePlanner>(this,make_shared<PyEdgePlanner>(this,a,b,obstacle));
  }
}

class PyConstraintSet : public CSet
{
public:
  PyObject* test,*sampler;
  PyConstraintSet(PyObject* _test,PyObject* _sampler=NULL)
    :test(_test),sampler(_sampler)
  {
    Assert(test != NULL);
    Py_INCREF(test);
    if(sampler)
      Py_INCREF(sampler);
  }
  ~PyConstraintSet() {
    Py_DECREF(test);
    if(sampler)
      Py_DECREF(sampler);
  }
  virtual void Sample(Config& x) {
    if(sampler) {
      //sample using python
      PyObject* result = PyObject_CallFunctionObjArgs(sampler,NULL);
      if(result == NULL) {
  if(!PyErr_Occurred()) {
    throw PyException("Error calling goal sampler provided to setEndpoints, must accept 0 arguments");
  }
  else {
    throw PyPyErrorException();
  }
      }
      PyListToConfig(result,x);
      Py_DECREF(result);
    }
    else CSet::Sample(x);
  }
  virtual bool Contains(const Config& q) {
    PyObject* pyq = ToPy(q);
    PyObject* result = PyObject_CallFunctionObjArgs(test,pyq,NULL);
    Py_DECREF(pyq);
    if(result == NULL) {
      if(!PyErr_Occurred()) {
  throw PyException("Error calling goal sampler provided to setEndpoints, must accept 1 argument");
      }
      else {
  throw PyPyErrorException();
      }
    }
    if(!PyBool_Check(result) && !PyInt_Check(result)) {
      Py_DECREF(result);
      throw PyException("Python visible test didn't return bool");
    }
    int res=PyObject_IsTrue(result);
    Py_DECREF(result);
    return res == 1;
  }
};


class PyGoalSet : public CSet
{
public:
  PyObject* goalTest,*sampler;
  PyGoalSet(PyObject* _goalTest,PyObject* _sampler=NULL)
    :goalTest(_goalTest),sampler(_sampler)
  {
    Py_INCREF(goalTest);
    if(sampler)
      Py_INCREF(sampler);
  }
  ~PyGoalSet() {
    Py_DECREF(goalTest);
    if(sampler)
      Py_DECREF(sampler);
  }
  virtual bool IsSampleable() const { return sampler != NULL; }
  virtual void Sample(Config& x) {
    if(sampler) {
      //sample using python
      PyObject* result = PyObject_CallFunctionObjArgs(sampler,NULL);
      if(result == NULL) {
        if(!PyErr_Occurred()) {
          throw PyException("Error calling goal sampler provided to setEndpoints, must accept 0 arguments");
        }
        else {
          throw PyPyErrorException();
        }
      }
      PyListToConfig(result,x);
      Py_DECREF(result);
    }
    else CSet::Sample(x);
  }
  virtual bool Contains(const Config& q) {
    PyObject* pyq = ToPy(q);
    PyObject* result = PyObject_CallFunctionObjArgs(goalTest,pyq,NULL);
    Py_DECREF(pyq);
    if(result == NULL) {
      if(!PyErr_Occurred()) {
        throw PyException("Error calling goal sampler provided to setEndpoints, must accept 1 argument");
      }
      else {
        throw PyPyErrorException();
      }
    }
    if(!PyBool_Check(result) && !PyInt_Check(result)) {
      Py_DECREF(result);
      throw PyException("Python visible test didn't return bool");
    }
    int res=PyObject_IsTrue(result);
    Py_DECREF(result);
    return res == 1;
  }
};

class PyObjectiveFunction : public ObjectiveFunctionalBase
{
public:
  PyObject* edgeCost,*terminalCost;
  PyObjectiveFunction(PyObject* _edgeCost,PyObject* _terminalCost)
    :edgeCost(_edgeCost),terminalCost(_terminalCost)
  {
    if(edgeCost)
      Py_INCREF(edgeCost);
    if(terminalCost)
      Py_INCREF(terminalCost);
  }
  ~PyObjectiveFunction() {
    if(edgeCost)
      Py_DECREF(edgeCost);
    if(terminalCost)
      Py_DECREF(terminalCost);
  }
  virtual const char* TypeString() { return "Python objective function"; }
  virtual Real IncrementalCost(const Interpolator* path) { 
    if(!edgeCost) return 0.0; 
    PyObject* pya = ToPy(path->Start());
    PyObject* pyb = ToPy(path->End());
    PyObject* result = PyObject_CallFunctionObjArgs(edgeCost,pya,pyb,NULL);
    Py_DECREF(pya);
    Py_DECREF(pyb);
    if(result == NULL) {
      if(!PyErr_Occurred()) {
        throw PyException("Error calling edge cost provided to setObjective, must accept 2 arguments");
      }
      else {
        throw PyPyErrorException();
      }
    }
    if(!PyFloat_Check(result) && !PyInt_Check(result)) {
      Py_DECREF(result);
      throw PyException("Edge cost function didn't return float/int");
    }
    double res=PyFloat_AsDouble(result);
    Py_DECREF(result);
    return res;
  }
  virtual Real TerminalCost(const Config& qend) {
    if(!terminalCost) return 0.0; 
    PyObject* pyq = ToPy(qend);
    PyObject* result = PyObject_CallFunctionObjArgs(terminalCost,pyq,NULL);
    Py_DECREF(pyq);
    if(result == NULL) {
      if(!PyErr_Occurred()) {
        throw PyException("Error calling terminal cost provided to setObjective, must accept 1 argument");
      }
      else {
        throw PyPyErrorException();
      }
    }
    if(!PyFloat_Check(result) && !PyInt_Check(result)) {
      Py_DECREF(result);
      throw PyException("Terminal cost didn't return float/int");
    }
    double res=PyFloat_AsDouble(result);
    Py_DECREF(result);
    return res;
  }
  virtual bool PathInvariant() const { return edgeCost == NULL; }
};


struct PyCSpaceData
{
  CSpaceInterface* interface;
  shared_ptr<PyCSpace> space;
  shared_ptr<AdaptiveCSpace> adaptiveSpace;
};

struct PyMotionPlannerData
{
  PlannerInterface* interface;
  shared_ptr<MotionPlannerInterface> planner;
  shared_ptr<PyGoalSet> goalSet;
  shared_ptr<PyObjectiveFunction> objective;
};


static vector<PyCSpaceData> spaces;
static vector<PyMotionPlannerData> plans;
static MotionPlannerFactory factory;
static list<int> spacesDeleteList;
static list<int> plansDeleteList;

int makeNewCSpace(CSpaceInterface* iface)
{
  if(spacesDeleteList.empty()) {
    spaces.resize(spaces.size()+1);
    spaces.back().interface = iface;
    spaces.back().space = make_shared<PyCSpace>();
    return (int)(spaces.size()-1);
  }
  else {
    int index = spacesDeleteList.front();
    spacesDeleteList.erase(spacesDeleteList.begin());
    spaces[index].interface = iface;
    spaces[index].space.reset(new PyCSpace);
    return index;
  }
}

void destroyCSpace(int cspace)
{
  if(cspace < 0 || cspace >= (int)spaces.size()) 
    throw PyException("Invalid cspace index");
  spaces[cspace].interface = NULL;
  spaces[cspace].space.reset();
  spaces[cspace].adaptiveSpace.reset();
  spacesDeleteList.push_back(cspace);
}

CSpace* getPreferredSpace(int index)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  if(spaces[index].adaptiveSpace)
    return spaces[index].adaptiveSpace.get();
  return spaces[index].space.get();
}

CSpaceInterface::CSpaceInterface()
{
  index = makeNewCSpace(this);
}

CSpaceInterface::CSpaceInterface(const CSpaceInterface& space)
{
  index = makeNewCSpace(this);
  spaces[index].space = spaces[space.index].space;
  spaces[index].adaptiveSpace = spaces[space.index].adaptiveSpace;
}

CSpaceInterface::~CSpaceInterface()
{
  this->destroy();
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
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  spaces[index].space->constraintNames.resize(1);
  spaces[index].space->constraintNames[0] = "feasible";
  spaces[index].space->constraints.resize(1);
  spaces[index].space->constraints[0] = make_shared<PyConstraintSet>(pyFeas);
}


void CSpaceInterface::addFeasibilityTest(const char* name,PyObject* pyFeas)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  int cindex = spaces[index].space->ConstraintIndex(name);
  spaces[index].space->constraints.resize(spaces[index].space->constraintNames.size(),shared_ptr<PyConstraintSet>());
  if(cindex < 0) {
    spaces[index].space->constraintNames.push_back(name);
    spaces[index].space->constraints.push_back(make_shared<PyConstraintSet>(pyFeas));
  }
  else {
    spaces[index].space->constraints[cindex] = make_shared<PyConstraintSet>(pyFeas);
  }
}

void CSpaceInterface::setVisibility(PyObject* pyVisible)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  for(size_t i=0;i<spaces[index].space->visibleTests.size();i++)
    Py_XDECREF(spaces[index].space->visibleTests[i]);
  Py_XINCREF(pyVisible);
  spaces[index].space->visibleTests.resize(1);
  spaces[index].space->visibleTests[0] = pyVisible;
}

void CSpaceInterface::addVisibilityTest(const char* name,PyObject* pyVis)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  int cindex = spaces[index].space->ConstraintIndex(name);
  spaces[index].space->visibleTests.resize(spaces[index].space->constraintNames.size(),NULL);
  if(cindex < 0) {
    cindex = (int)spaces[index].space->visibleTests.size();
    Py_XINCREF(pyVis);
    spaces[index].space->visibleTests.push_back(pyVis);
    spaces[index].space->constraintNames.push_back(name);
    spaces[index].space->constraints.push_back(std::shared_ptr<CSet>());
  }
  else {
    Py_DECREF(spaces[index].space->visibleTests[cindex]);
    Py_XINCREF(pyVis);
    spaces[index].space->visibleTests[cindex] = pyVis;
  }
}

void CSpaceInterface::setVisibilityEpsilon(double eps)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  if(eps <= 0) 
    throw PyException("Invalid epsilon");
  for(size_t i=0;i<spaces[index].space->visibleTests.size();i++)
    Py_XDECREF(spaces[index].space->visibleTests[i]);
  spaces[index].space->visibleTests.resize(0);
  spaces[index].space->edgeResolution = eps;
}

void CSpaceInterface::setSampler(PyObject* pySamp)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  Py_XDECREF(spaces[index].space->sample);
  Py_XINCREF(pySamp);
  spaces[index].space->sample = pySamp;
}

void CSpaceInterface::setNeighborhoodSampler(PyObject* pySamp)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  Py_XDECREF(spaces[index].space->sampleNeighborhood);
  Py_XINCREF(pySamp);
  spaces[index].space->sampleNeighborhood = pySamp;
}

void CSpaceInterface::setDistance(PyObject* pyDist)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  Py_XDECREF(spaces[index].space->distance);
  Py_XINCREF(pyDist);
  spaces[index].space->distance = pyDist;
}

void CSpaceInterface::setInterpolate(PyObject* pyInterp)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  Py_XDECREF(spaces[index].space->interpolate);
  Py_XINCREF(pyInterp);
  spaces[index].space->interpolate = pyInterp;
}

void CSpaceInterface::setProperty(const char* key,const char* value)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  spaces[index].space->properties[key] = value;
}

const char* CSpaceInterface::getProperty(const char* key)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  if(spaces[index].space->properties.count(key)==0) 
    throw PyException("Invalid property");
  return spaces[index].space->properties[key].c_str();
}


///queries
bool CSpaceInterface::isFeasible(PyObject* q)
{
  Config vq;
  if(!PyListToConfig(q,vq)) {
    throw PyException("Invalid configuration (must be list)");
  }
  return getPreferredSpace(index)->IsFeasible(vq);
}

bool CSpaceInterface::isVisible(PyObject* a,PyObject* b)
{
  Config va,vb;
  if(!PyListToConfig(a,va)) {
    throw PyException("Invalid configuration a (must be list)");
  }
  if(!PyListToConfig(b,vb)) {
    throw PyException("Invalid configuration b (must be list)");
  }
  CSpace* s = getPreferredSpace(index);
  return s->PathChecker(va,vb)->IsVisible();
}

bool CSpaceInterface::testFeasibility(const char* name,PyObject* q)
{
  Config vq;
  if(!PyListToConfig(q,vq)) {
    throw PyException("Invalid configuration (must be list)");
  }
  CSpace* s=getPreferredSpace(index);
  int cindex = spaces[index].space->ConstraintIndex(name);
  if(cindex < 0)
     throw PyException("Invalid constraint name");
  return s->IsFeasible(vq,cindex);
}

bool CSpaceInterface::testVisibility(const char* name,PyObject* a,PyObject* b)
{
  Config va,vb;
  if(!PyListToConfig(a,va)) {
    throw PyException("Invalid configuration a (must be list)");
  }
  if(!PyListToConfig(b,vb)) {
    throw PyException("Invalid configuration b (must be list)");
  }
  CSpace* s=getPreferredSpace(index);
  int cindex = spaces[index].space->ConstraintIndex(name);
  if(cindex < 0)
     throw PyException("Invalid constraint name");
  return s->PathChecker(va,vb,cindex)->IsVisible();
}

PyObject* CSpaceInterface::feasibilityFailures(PyObject* q)
{
  Config vq;
  if(!PyListToConfig(q,vq)) {
    throw PyException("Invalid configuration (must be list)");    
  }
  vector<string> infeasible;
  getPreferredSpace(index)->GetInfeasibleNames(vq,infeasible);
  return ToPy(infeasible);
}

PyObject* CSpaceInterface::visibilityFailures(PyObject* a,PyObject* b)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  Config va,vb;
  if(!PyListToConfig(a,va)) {
    throw PyException("Invalid configuration a (must be list)");
  }
  if(!PyListToConfig(b,vb)) {
    throw PyException("Invalid configuration b (must be list)");
  }
  vector<string> notVisible;
  for(size_t i=0;i<spaces[index].space->constraints.size();i++)
    if(!spaces[index].space->IsVisible(va,vb,i)) notVisible.push_back(spaces[index].space->constraintNames[i]);
  return ToPy(notVisible);
}

PyObject* CSpaceInterface::sample()
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  Config q;
  spaces[index].space->Sample(q);
  return ToPy(q);
}

double CSpaceInterface::distance(PyObject* a,PyObject* b)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  Config va,vb;
  if(!PyListToConfig(a,va)) {
    throw PyException("Invalid configuration a (must be list)");
  }
  if(!PyListToConfig(b,vb)) {
    throw PyException("Invalid configuration b (must be list)");
  }
  return spaces[index].space->Distance(va,vb);
}

PyObject* CSpaceInterface::interpolate(PyObject* a,PyObject* b,double u)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  Config va,vb,vout;
  if(!PyListToConfig(a,va)) {
    throw PyException("Invalid configuration a (must be list)");
  }
  if(!PyListToConfig(b,vb)) {
    throw PyException("Invalid configuration b (must be list)");
  }
  spaces[index].space->Interpolate(va,vb,u,vout);
  return PyListFromConfig(vout);
}

bool CSpaceInterface::adaptiveQueriesEnabled()
{
  return index < (int)spaces.size() && (spaces[index].adaptiveSpace);
}
void CSpaceInterface::enableAdaptiveQueries(bool enabled)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  if(!spaces[index].adaptiveSpace)
    spaces[index].adaptiveSpace.reset(new AdaptiveCSpace(spaces[index].space.get()));
}

void CSpaceInterface::optimizeQueryOrder()
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].adaptiveSpace) 
    throw PyException("adaptive queries not enabled for this space");
  spaces[index].adaptiveSpace->OptimizeQueryOrder();
}

void CSpaceInterface::setFeasibilityDependency(const char* name,const char* precedingTest)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].adaptiveSpace) 
    throw PyException("adaptive queries not enabled for this space");
  if(!spaces[index].adaptiveSpace->AddFeasibleDependency(name,precedingTest))
    throw PyException("Invalid dependency");
}

void CSpaceInterface::setFeasibilityPrior(const char* name,double costPrior,double feasibilityProbability,double evidenceStrength)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].adaptiveSpace) 
    throw PyException("Invalid cspace index");
  int cindex = spaces[index].space->ConstraintIndex(name);
  AdaptiveCSpace::PredicateStats& stats = spaces[index].adaptiveSpace->feasibleStats[cindex];
  stats.cost = costPrior;
  stats.probability = feasibilityProbability;
  stats.count = evidenceStrength;
}

void CSpaceInterface::setVisibilityDependency(const char* name,const char* precedingTest)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].adaptiveSpace) 
    throw PyException("adaptive queries not enabled for this space");
  if(!spaces[index].adaptiveSpace->AddVisibleDependency(name,precedingTest))
    throw PyException("Invalid dependency");
}

void CSpaceInterface::setVisibilityPrior(const char* name,double costPrior,double visibilityProbability,double evidenceStrength)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].adaptiveSpace) 
    throw PyException("adaptive queries not enabled for this space");
  int cindex = spaces[index].space->ConstraintIndex(name);
  if(cindex < 0)
     throw PyException("Invalid constraint name");
  AdaptiveCSpace::PredicateStats& stats = spaces[index].adaptiveSpace->visibleStats[cindex];
  stats.cost = costPrior;
  stats.probability = visibilityProbability;
  stats.count = evidenceStrength;
}

double CSpaceInterface::feasibilityCost(const char* name)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].adaptiveSpace) 
    throw PyException("adaptive queries not enabled for this space");
  int cindex = spaces[index].space->ConstraintIndex(name);
  if(cindex < 0)
     throw PyException("Invalid constraint name");
  return spaces[index].adaptiveSpace->feasibleStats[cindex].cost;
}

double CSpaceInterface::feasibilityProbability(const char* name)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].adaptiveSpace) 
    throw PyException("adaptive queries not enabled for this space");
  int cindex = spaces[index].space->ConstraintIndex(name);
  if(cindex < 0)
     throw PyException("Invalid constraint name");
  return spaces[index].adaptiveSpace->feasibleStats[cindex].probability;
}

double CSpaceInterface::visibilityCost(const char* name)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].adaptiveSpace) 
    throw PyException("adaptive queries not enabled for this space");
  int cindex = spaces[index].space->ConstraintIndex(name);
  if(cindex < 0)
     throw PyException("Invalid constraint name");
  return spaces[index].adaptiveSpace->visibleStats[cindex].cost;
}

double CSpaceInterface::visibilityProbability(const char* name)
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].adaptiveSpace) 
    throw PyException("adaptive queries not enabled for this space");
  int cindex = spaces[index].space->ConstraintIndex(name);
  if(cindex < 0)
     throw PyException("Invalid constraint name");
  return spaces[index].adaptiveSpace->visibleStats[cindex].probability;
}

PyObject* CSpaceInterface::feasibilityQueryOrder()
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].adaptiveSpace) 
    throw PyException("adaptive queries not enabled for this space");
  PyObject* res = PyList_New(spaces[index].space->constraints.size());
  for(size_t i=0;i<spaces[index].space->constraintNames.size();i++) {
    int cindex = (spaces[index].adaptiveSpace->feasibleTestOrder.empty() ? (int)i : spaces[index].adaptiveSpace->feasibleTestOrder[i]);
    PyObject* s = PyString_FromString(spaces[index].space->constraintNames[cindex].c_str());
    PyList_SetItem(res,i,s);
  }
  return res;
}

PyObject* CSpaceInterface::visibilityQueryOrder()
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].adaptiveSpace) 
    throw PyException("adaptive queries not enabled for this space");
  PyObject* res = PyList_New(spaces[index].space->constraints.size());
  for(size_t i=0;i<spaces[index].space->constraintNames.size();i++) {
    int cindex = (spaces[index].adaptiveSpace->visibleTestOrder.empty() ? (int)i : spaces[index].adaptiveSpace->visibleTestOrder[i]);
    PyObject* s = PyString_FromString(spaces[index].space->constraintNames[cindex].c_str());
    PyList_SetItem(res,i,s);
  }
  return res;
}

PyObject* CSpaceInterface::getStats()
{
  if(index < 0 || index >= (int)spaces.size() || !spaces[index].space) 
    throw PyException("Invalid cspace index");
  PyObject* res = PyDict_New();
  PropertyMap stats;
  if(index < (int)spaces.size() && spaces[index].adaptiveSpace) {
    spaces[index].adaptiveSpace->GetStats(stats);
  }
  stats.set("feasible_count",spaces[index].space->feasibleStats.count);
  stats.set("feasible_probability",spaces[index].space->feasibleStats.probability);
  stats.set("feasible_time",spaces[index].space->feasibleStats.cost);
  stats.set("visible_count",spaces[index].space->visibleStats.count);
  stats.set("visible_probability",spaces[index].space->visibleStats.probability);
  stats.set("visible_time",spaces[index].space->visibleStats.cost);
  stats.set("average_visible_length",spaces[index].space->visibleDistance);
  stats.set("average_notvisible_length",spaces[index].space->notVisibleDistance);
  for(PropertyMap::const_iterator i=stats.begin();i!=stats.end();i++) {
    PyObject* value = PyString_FromString(i->second.c_str());
    PyDict_SetItemString(res,i->first.c_str(),value);
    Py_XDECREF(value);
  }
  return res;
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
    stringstream ss;
    ss<<"Invalid numeric setting \""<<setting<<"\""<<endl;
    ss<<"Valid keys are:"<<endl;
    ss<<"  knn, connectionThreshold, perturbationRadius, bidirectional,"<<endl;
    ss<<"  grid, gridResolution, suboptimalityFactor, randomizeFrequency,"<<endl;
    ss<<"  shortcut, restart"<<endl;
    throw PyException(ss.str());
  }
}

void setPlanSetting(const char* setting,const char* value)
{
  if(0==strcmp(setting,"pointLocation"))
    factory.pointLocation = value;
  else if(0==strcmp(setting,"restartTermCond"))
    factory.restartTermCond = value;
  else {
    stringstream ss;
    ss<<"Invalid string-valued setting \""<<setting<<"\""<<endl;
    ss<<"Valid keys are:"<<endl;
    ss<<"  pointLocation, restartTermCond"<<endl;
    throw PyException(ss.str());
  }
}


int makeNewPlan(int cspace,PlannerInterface* iface)
{
  if(cspace < 0 || cspace >= (int)spaces.size() || spaces[cspace].interface==NULL) 
    throw PyException("Invalid cspace index");
  CSpace* klSpace = getPreferredSpace(cspace);
  if(plansDeleteList.empty()) {
    plans.resize(plans.size()+1);
    plans.back().interface = iface;
    plans.back().planner = shared_ptr<MotionPlannerInterface>(factory.Create(klSpace));
    return (int)plans.size()-1;
  }
  else {
    int index = plansDeleteList.front();
    plansDeleteList.erase(plansDeleteList.begin());
    plans[index].interface = iface;
    plans[index].planner.reset(factory.Create(klSpace));
    return index;
  }
}

void destroyPlan(int plan)
{
  if(plan < 0 || plan >= (int)plans.size() || plans[plan].interface==NULL) 
    throw PyException("Invalid plan index");
  plans[plan].interface = NULL;
  plans[plan].planner.reset();
  plans[plan].goalSet.reset();
  plans[plan].objective.reset();
  plansDeleteList.push_back(plan);
}

PlannerInterface::PlannerInterface(const CSpaceInterface& cspace)
{
  index = makeNewPlan(cspace.index,this);
  spaceIndex = cspace.index;
}

PlannerInterface::~PlannerInterface()
{
  this->destroy();
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
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");
  Config qstart,qgoal;
  bool res=PyListToConfig(start,qstart);
  if(!res) 
    throw PyException("Invalid start endpoint");
  CSpace* s = getPreferredSpace(spaceIndex);
  if(!s->IsFeasible(qstart)) {
    throw PyException("Start configuration is infeasible");
  }
  int istart=plans[index].planner->AddMilestone(qstart);
  if(istart < 0) {
    throw PyException("Start configuration is infeasible");
  }
  if(istart != 0) {
    throw PyException("Plan already initialized?");
  }

  res=PyListToConfig(goal,qgoal);
  if(!res) 
    throw PyException("Invalid start endpoint");
  if(!s->IsFeasible(qgoal)) {
    throw PyException("Goal configuration is infeasible");
  }
  int igoal=plans[index].planner->AddMilestone(qgoal);
  if(igoal < 0) {
    throw PyException("Goal configuration is infeasible");
  }
  return true;
}
bool PlannerInterface::setEndpointSet(PyObject* start,PyObject* goal,PyObject* goalSample)
{
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");
  Config qstart;
  bool res=PyListToConfig(start,qstart);
  if(!res) 
    throw PyException("Invalid start endpoint");
  CSpace* s = getPreferredSpace(spaceIndex);
  if(!s->IsFeasible(qstart)) {
    throw PyException("Start configuration is infeasible");
  }
  //test if it's a goal test
  if(!PyCallable_Check(goal)) {
    throw PyException("Goal test is not callable");
  }
  if(goalSample == Py_None)
    goalSample = NULL;
  plans[index].goalSet.reset(new PyGoalSet(goal,goalSample));
  plans[index].planner.reset(factory.Create(s,qstart,plans[index].goalSet.get()));
  return true;
}

int PlannerInterface::addMilestone(PyObject* milestone)
{
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");
  if(!plans[index].planner->CanAddMilestone()) 
    throw PyException("The planner cannot accept any more milestones");
  Config q;
  bool res=PyListToConfig(milestone,q);
  if(!res) 
    throw PyException("Invalid configuration provided to addMilestone");
  int mindex=plans[index].planner->AddMilestone(q);
  return mindex;
}

int PlannerInterface::getClosestMilestone(PyObject* config)
{
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");
  Config q;
  bool res=PyListToConfig(config,q);
  if(!res) 
    throw PyException("Invalid configuration provided to addMilestone");
  int mindex = plans[index].planner->GetClosestMilestone(q);
  if(mindex < 0)
    throw PyException("The planner does not support getClosestMilestone");
  return mindex;
}

PyObject* PlannerInterface::getMilestone(int milestone)
{
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");
  Config q;
  plans[index].planner->GetMilestone(milestone,q);
  if(q.n==0)
    throw PyException("The planner does not support getMilestone");
  return PyListFromConfig(q);
}

void DumpPlan(MotionPlannerInterface* planner,const char* fn)
{
  RoadmapPlanner prm(NULL);
  planner->GetRoadmap(prm.roadmap);
  
  Graph::Graph<string,string> Gstr;
  Graph::NodesToStrings(prm.roadmap,Gstr);
  
  ofstream out(fn);
  Graph::Save_TGF(out,Gstr);
  out.close();
}

void PlannerInterface::planMore(int iterations)
{
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");
  if(plans[index].planner->IsPointToPoint() && plans[index].planner->NumMilestones() < 1) throw PyException("No start or goal set for point-to-point planner, cannot start");
  if(spaceIndex < (int)spaces.size() && spaces[spaceIndex].adaptiveSpace) spaces[spaceIndex].adaptiveSpace->OptimizeQueryOrder();
  plans[index].planner->PlanMore(iterations);
  //printf("Plan now has %d milestones, %d components\n",plans[plan]->NumMilestones(),plans[plan]->NumComponents());
  //DumpPlan(plans[plan],"plan.tgf");
}

void PlannerInterface::setCostFunction(PyObject* edgeCost,PyObject* terminalCost)
{
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");  
  if(!plans[index].planner->CanUseObjective()) 
    throw PyException("That motion planner cannot accept objective functions");
  if(edgeCost == Py_None)
    edgeCost = NULL;
  if(terminalCost == Py_None)
    terminalCost = NULL;
  plans[index].objective.reset(new PyObjectiveFunction(edgeCost,terminalCost));
  plans[index].planner->SetObjective(plans[index].objective);
}

PyObject* PlannerInterface::getSolutionPath()
{
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");  
  if(!plans[index].planner->IsSolved()) 
    Py_RETURN_NONE;
  MilestonePath path;
  plans[index].planner->GetSolution(path);
  PyObject* pypath = PyList_New(path.NumMilestones());
  for(int i=0;i<path.NumMilestones();i++)
    PyList_SetItem(pypath,(Py_ssize_t)i,PyListFromConfig(path.GetMilestone(i)));
  return pypath;
}

PyObject* PlannerInterface::getPath(int milestone1,int milestone2)
{
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");  
  if(!plans[index].planner->IsConnected(milestone1,milestone2)) {
    Py_RETURN_NONE;
  }
  MilestonePath path;
  plans[index].planner->GetPath(milestone1,milestone2,path);
  PyObject* pypath = PyList_New(path.NumMilestones());
  for(int i=0;i<path.NumMilestones();i++)
    PyList_SetItem(pypath,(Py_ssize_t)i,PyListFromConfig(path.GetMilestone(i)));
  return pypath;
}

PyObject* PlannerInterface::getPath(int milestone1,const std::vector<int>& goalMilestones)
{
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");  
  MilestonePath path;
  Real cost = plans[index].planner->GetOptimalPath(milestone1,goalMilestones,path);
  if(path.edges.empty())
    Py_RETURN_NONE;
  PyObject* pypath = PyList_New(path.NumMilestones());
  for(int i=0;i<path.NumMilestones();i++)
    PyList_SetItem(pypath,(Py_ssize_t)i,PyListFromConfig(path.GetMilestone(i)));
  return pypath;
}

double PlannerInterface::getData(const char* setting)
{
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");  
  if(0==strcmp(setting,"iterations")) {
    return plans[index].planner->NumIterations();
  }
  else if(0==strcmp(setting,"milestones")) {
    return plans[index].planner->NumMilestones();
  }
  else if(0==strcmp(setting,"components")) {
    return plans[index].planner->NumComponents();
  }
  else {
    throw PyException("Invalid plan option");
    return 0;
  }
}

PyObject* PlannerInterface::getStats()
{
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");  
  PropertyMap stats;
  plans[index].planner->GetStats(stats);
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
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");
  RoadmapPlanner prm(NULL);
  plans[index].planner->GetRoadmap(prm.roadmap);
  PyObject* pyV = PyList_New(prm.roadmap.nodes.size());
  for(size_t i=0;i<prm.roadmap.nodes.size();i++)
    PyList_SetItem(pyV,(Py_ssize_t)i,PyListFromConfig(prm.roadmap.nodes[i]));
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
  if(index < 0 || index >= (int)plans.size() || !plans[index].planner) 
    throw PyException("Invalid plan index");
  DumpPlan(plans[index].planner.get(),fn);
}

void destroy()
{
  for(size_t i=0;i<spaces.size();i++) {
    if(spaces[i].interface)
      spaces[i].interface->index = -1;
  }
  for(size_t i=0;i<plans.size();i++) {
    if(plans[i].interface)
      plans[i].interface->index = -1;
  }
  spaces.clear();
  spacesDeleteList.clear();
  plans.clear();
  plansDeleteList.clear();
}
