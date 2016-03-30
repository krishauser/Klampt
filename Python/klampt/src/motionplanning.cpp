#include "motionplanning.h"
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <KrisLibrary/planning/ExplicitCSpace.h>
#include <KrisLibrary/planning/CSpaceHelpers.h>
#include "pyerr.h"
#include <KrisLibrary/graph/IO.h>
#include <KrisLibrary/math/random.h>
#include <Python.h>
#include <iostream>
#include <fstream>
#include <exception>
#include <vector>
#include <map>
using namespace std;

void setRandomSeed(int seed)
{
  Math::Srand(seed);
}

PyObject* ToPy(int x) { return PyInt_FromLong(x); }
PyObject* ToPy(double x) { return PyFloat_FromDouble(x); }
PyObject* ToPy(const string& x) { return PyString_FromString(x.c_str()); }

template <class T>
PyObject* ToPy(const std::vector<T>& x)
{
  PyObject* ls = PyList_New(x.size());
  PyObject* pItem;
  if(ls == NULL) {
    goto fail;
  }
	
  for(Py_ssize_t i = 0; i < PySequence_Size(ls); i++) {
    pItem = ::ToPy(x[i]);
    if(pItem == NULL)
      goto fail;
    PyList_SetItem(ls, i, pItem);
  }
  
  return ls;
  
 fail:
  Py_XDECREF(ls);
  throw PyException("Failure during ToPy");
  return NULL;
}

PyObject* ToPy(const Config& x) {
  PyObject* ls = PyList_New(x.n);
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
  throw PyException("Failure during ToPy");
  return NULL;
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

PyObject* PyListFromConfig(const Config& x)
{
  PyObject* ls = PyList_New(x.n);
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
  throw PyException("Failure during PyListFromConfig");
  return NULL;
}


bool PyListToConfig(PyObject* seq,Config& x)
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
class PyCSpace : public ExplicitCSpace
{
public:
  PyCSpace()
    :sample(NULL),sampleNeighborhood(NULL),
     distance(NULL),interpolate(NULL),edgeResolution(0.001)
  {}

  virtual ~PyCSpace() {
    Py_XDECREF(sample);
    Py_XDECREF(sampleNeighborhood);
    for(size_t i=0;i<feasibleTests.size();i++)
      Py_XDECREF(feasibleTests[i]);
    for(size_t i=0;i<visibleTests.size();i++)
      Py_XDECREF(visibleTests[i]);
    Py_XDECREF(distance);
    Py_XDECREF(interpolate);
  }

  void operator = (const PyCSpace& rhs)
  {
    sample = rhs.sample;
    sampleNeighborhood = rhs.sampleNeighborhood;
    feasibleTests = rhs.feasibleTests;
    visibleTests = rhs.visibleTests;
    constraintNames = rhs.constraintNames;
    constraintMap = rhs.constraintMap;
    distance = rhs.distance;
    interpolate = rhs.interpolate;
    edgeResolution = rhs.edgeResolution;
    Py_XINCREF(sample);
    Py_XINCREF(sampleNeighborhood);
    for(size_t i=0;i<feasibleTests.size();i++)
      Py_XINCREF(feasibleTests[i]);
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
      PyObject* pyc=PyListFromConfig(c);
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
      bool res=PyListToConfig(result,x);
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


  virtual int NumObstacles() { return feasibleTests.size(); }
  virtual std::string ObstacleName(int obstacle) {
    if(obstacle < 0 || obstacle >= (int)feasibleTests.size()) return "";
    return constraintNames[obstacle];
  }
  virtual bool IsFeasible(const Config& x,int obstacle) {
    if(obstacle < 0 || obstacle >= (int)feasibleTests.size()) return false;

    if(feasibleTests[obstacle] == NULL) {
      stringstream ss;
      ss<<"Python feasible test for constraint "<<constraintNames[obstacle]<<"not defined"<<endl;
      throw PyException(ss.str().c_str());
    }

    PyObject* pyx = PyListFromConfig(x);
    PyObject* result = PyObject_CallFunctionObjArgs(feasibleTests[obstacle],pyx,NULL);
    Py_DECREF(pyx);
    if(result == NULL) {
      if(!PyErr_Occurred()) {
	throw PyException("An error occurred when calling feasible");
      }
      else {
	throw PyPyErrorException();
      }
    }
    if(!PyBool_Check(result)) {
      Py_DECREF(result);
      throw PyException("Python feasible test method didn't return bool");
    }
    bool res=(result == Py_True);
    if(!res) {
      Py_DECREF(result);
      return false;
    }
    return true;
  }

  virtual bool IsFeasible(const Config& x)
  {
    if(feasibleTests.empty()) {
      throw PyException("Python feasible method not defined");
    }
    PyObject* pyx = PyListFromConfig(x);
    for(size_t i=0;i<feasibleTests.size();i++) {
      if(feasibleTests[i] == NULL) {
	stringstream ss;
	ss<<"Python feasible test for constraint "<<constraintNames[i]<<"not defined"<<endl;
	Py_DECREF(pyx);
	throw PyException(ss.str().c_str());
      }

      PyObject* result = PyObject_CallFunctionObjArgs(feasibleTests[i],pyx,NULL);
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
	throw PyException("Python feasible test method didn't return bool");
      }
      bool res=(result == Py_True);
      if(!res) {
	Py_DECREF(pyx);
	Py_DECREF(result);
	return false;
      }
    }
    Py_DECREF(pyx);
    return true;
  }

  virtual bool IsVisible(const Config& a,const Config& b) {
    EdgePlanner* e = LocalPlanner(a,b);
    bool res = e->IsVisible();
    delete e;
    return res;
  }

  virtual bool IsVisible(const Config& a,const Config& b,int obstacle) {
    EdgePlanner* e = LocalPlanner(a,b,obstacle);
    bool res = e->IsVisible();
    delete e;
    return res;
  }

  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b,int obstacle);

  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b);

  virtual double Distance(const Config& x, const Config& y)
  {
    if(!distance) {
      return CSpace::Distance(x,y);
    }
    else {
      PyObject* args = PyTuple_New(2);
      PyTuple_SetItem(args, 0, PyListFromConfig(x));
      PyTuple_SetItem(args, 1, PyListFromConfig(y));
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
      PyTuple_SetItem(args, 0, PyListFromConfig(x));
      PyTuple_SetItem(args, 1, PyListFromConfig(y));
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
      bool res=PyListToConfig(result,out);
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
    *distance,
    *interpolate;
  vector<PyObject*> feasibleTests;
  vector<PyObject*> visibleTests;
  vector<string> constraintNames;
  map<string,int> constraintMap;
  double edgeResolution;
  PropertyMap properties;
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
    assert(space->visibleTests.size() == space->feasibleTests.size());
    PyObject* args = PyTuple_New(2);
    PyTuple_SetItem(args, 0, PyListFromConfig(a));
    PyTuple_SetItem(args, 1, PyListFromConfig(b));
    if(obstacle < 0) { //test all obstacles
      for(size_t i=0;i<space->visibleTests.size();i++) {
	if(space->visibleTests[i] == NULL) {
	  stringstream ss;
	  ss<<"Python visible test for constraint "<<space->constraintNames[i]<<"not defined"<<endl;
	  Py_DECREF(args);
	  throw PyException(ss.str().c_str());
	}
	
	PyObject* result = PyObject_CallObject(space->visibleTests[i],args);
	if(!result) {
	  Py_DECREF(args);
	  if(!PyErr_Occurred()) {
	    throw PyException("Python visible method failed");
	  }
	  else {
	    throw PyPyErrorException();
	  }
	}
	if(!PyBool_Check(result) && !PyInt_Check(result)) {
	  Py_DECREF(args);
	  Py_DECREF(result);
	  throw PyException("Python visible test didn't return bool");
	}
	int res=PyObject_IsTrue(result);
	Py_DECREF(result);
	if(res != 1) {
	  Py_DECREF(args);
	  return false;
	}
      }
      Py_DECREF(args);    
    }
    else {
      //call visibility test for one obstacle
      if(space->visibleTests[obstacle] == NULL) {
	stringstream ss;
	ss<<"Python visible test for constraint "<<space->constraintNames[obstacle]<<"not defined"<<endl;
	Py_DECREF(args);
	throw PyException(ss.str().c_str());
      }
	
      PyObject* result = PyObject_CallObject(space->visibleTests[obstacle],args);
      Py_DECREF(args);
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
  virtual void Eval(double u,Config& x) const
  {
    return space->Interpolate(a,b,u,x);
  }
  virtual const Config& Start() const { return a; }
  virtual const Config& Goal() const { return b; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const { return new PyEdgePlanner(space,a,b,obstacle); }
  virtual EdgePlanner* ReverseCopy() const { return new PyEdgePlanner(space,b,a,obstacle); }
};


EdgePlanner* PyCSpace::LocalPlanner(const Config& a,const Config& b)
{
  if(visibleTests.empty()) {
    return new StraightLineEpsilonPlanner(this,a,b,edgeResolution); 
  }
  else {
    return new PyEdgePlanner(this,a,b);
  }
}

EdgePlanner* PyCSpace::LocalPlanner(const Config& a,const Config& b,int obstacle)
{
  if(visibleTests.empty()) {
    return MakeSingleObstacleBisectionPlanner(this,a,b,obstacle,edgeResolution); 
  }
  else {
    return new PyEdgePlanner(this,a,b,obstacle);
  }
}

class PyGoalSet : public PiggybackCSpace
{
public:
  PyObject* goalTest,*sampler;
  PyGoalSet(CSpace* baseSpace,PyObject* _goalTest,PyObject* _sampler=NULL)
    :PiggybackCSpace(baseSpace),goalTest(_goalTest),sampler(_sampler)
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
    else PiggybackCSpace::Sample(x);
  }
  virtual bool IsFeasible(const Config& q) {
    //TODO: return goal test
    PyObject* pyq = ToPy(q);
    PyObject* result = PyObject_CallFunctionObjArgs(goalTest,pyq);
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




static vector<SmartPointer<PyCSpace> > spaces;
static vector<SmartPointer<MotionPlannerInterface> > plans;
static vector<SmartPointer<PyGoalSet> > goalSets;
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
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  for(size_t i=0;i<spaces[index]->feasibleTests.size();i++)
    Py_XDECREF(spaces[index]->feasibleTests[i]);
  Py_XINCREF(pyFeas);
  spaces[index]->feasibleTests.resize(1);
  spaces[index]->constraintNames.resize(1);
  spaces[index]->constraintNames[0] = "feasible";
  spaces[index]->constraintMap.clear();
  spaces[index]->constraintMap["feasible"]=0;
  spaces[index]->feasibleTests[0] = pyFeas;
}


void CSpaceInterface::addFeasibilityTest(const char* name,PyObject* pyFeas)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  int cindex = -1;
  if(spaces[index]->constraintMap.count(name) > 0)
    cindex = spaces[index]->constraintMap[name];
  spaces[index]->feasibleTests.resize(spaces[index]->constraintNames.size(),NULL);
  if(cindex < 0) {
    Py_XINCREF(pyFeas);
    spaces[index]->feasibleTests.push_back(pyFeas);
    spaces[index]->constraintNames.push_back(name);
  }
  else {
    Py_DECREF(spaces[index]->feasibleTests[cindex]);
    Py_XINCREF(pyFeas);
    spaces[index]->feasibleTests[cindex] = pyFeas;
  }
}

void CSpaceInterface::setVisibility(PyObject* pyVisible)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  for(size_t i=0;i<spaces[index]->visibleTests.size();i++)
    Py_XDECREF(spaces[index]->visibleTests[i]);
  Py_XINCREF(pyVisible);
  spaces[index]->visibleTests.resize(1);
  spaces[index]->visibleTests[0] = pyVisible;
}

void CSpaceInterface::addVisibilityTest(const char* name,PyObject* pyVis)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  int cindex = -1;
  if(spaces[index]->constraintMap.count(name) > 0)
    cindex = spaces[index]->constraintMap[name];
  spaces[index]->visibleTests.resize(spaces[index]->constraintNames.size(),NULL);
  if(cindex < 0) {
    Py_XINCREF(pyVis);
    spaces[index]->visibleTests.push_back(pyVis);
    spaces[index]->constraintNames.push_back(name);
  }
  else {
    Py_DECREF(spaces[index]->visibleTests[cindex]);
    Py_XINCREF(pyVis);
    spaces[index]->visibleTests[cindex] = pyVis;
  }
}

void CSpaceInterface::setVisibilityEpsilon(double eps)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  if(eps <= 0) 
    throw PyException("Invalid epsilon");
  for(size_t i=0;i<spaces[index]->visibleTests.size();i++)
    Py_XDECREF(spaces[index]->visibleTests[i]);
  spaces[index]->visibleTests.resize(0);
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

const char* CSpaceInterface::getProperty(const char* key)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  if(spaces[index]->properties.count(key)==0) 
    throw PyException("Invalid property");
  return spaces[index]->properties[key].c_str();
}


///queries
bool CSpaceInterface::isFeasible(PyObject* q)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Config vq;
  if(!PyListToConfig(q,vq)) {
    throw PyException("Invalid configuration (must be list)");
  }
  return spaces[index]->IsFeasible(vq);
}

bool CSpaceInterface::isVisible(PyObject* a,PyObject* b)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Config va,vb;
  if(!PyListToConfig(a,va)) {
    throw PyException("Invalid configuration a (must be list)");
  }
  if(!PyListToConfig(b,vb)) {
    throw PyException("Invalid configuration b (must be list)");
  }
  return spaces[index]->IsVisible(va,vb);
}

bool CSpaceInterface::testFeasibility(const char* name,PyObject* q)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Config vq;
  if(!PyListToConfig(q,vq)) {
    throw PyException("Invalid configuration (must be list)");
  }
  int index = -1;
  if(spaces[index]->constraintMap.count(name)==0)
     throw PyException("Invalid constraint name");
  index = spaces[index]->constraintMap[name];
  return spaces[index]->IsFeasible(vq,index);

}
bool CSpaceInterface::testVisibility(const char* name,PyObject* a,PyObject* b)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Config va,vb;
  if(!PyListToConfig(a,va)) {
    throw PyException("Invalid configuration a (must be list)");
  }
  if(!PyListToConfig(b,vb)) {
    throw PyException("Invalid configuration b (must be list)");
  }
  int index = -1;
  if(spaces[index]->constraintMap.count(name)==0)
     throw PyException("Invalid constraint name");
  index = spaces[index]->constraintMap[name];
  return spaces[index]->IsVisible(va,vb,index);
}

PyObject* CSpaceInterface::feasibilityFailures(PyObject* q)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) {
    printf("CSpace index %d is out of range [%d,%d) or was previously destroyed\n",index,0,spaces.size());
    throw PyException("Invalid cspace index");
  }
  Config vq;
  if(!PyListToConfig(q,vq)) {
    throw PyException("Invalid configuration (must be list)");    
  }
  vector<string> infeasible;
  spaces[index]->GetInfeasibleNames(vq,infeasible);
  return ToPy(infeasible);
}

PyObject* CSpaceInterface::visibilityFailures(PyObject* a,PyObject* b)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Config va,vb;
  if(!PyListToConfig(a,va)) {
    throw PyException("Invalid configuration a (must be list)");
  }
  if(!PyListToConfig(b,vb)) {
    throw PyException("Invalid configuration b (must be list)");
  }
  vector<string> notVisible;
  for(size_t i=0;i<spaces[index]->feasibleTests.size();i++)
    if(!spaces[index]->IsVisible(va,vb,i)) notVisible.push_back(spaces[index]->constraintNames[i]);
  return ToPy(notVisible);
}

PyObject* CSpaceInterface::sample()
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Config q;
  spaces[index]->Sample(q);
  return ToPy(q);
}

double CSpaceInterface::distance(PyObject* a,PyObject* b)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Config va,vb;
  if(!PyListToConfig(a,va)) {
    throw PyException("Invalid configuration a (must be list)");
  }
  if(!PyListToConfig(b,vb)) {
    throw PyException("Invalid configuration b (must be list)");
  }
  return spaces[index]->Distance(va,vb);
}

PyObject* CSpaceInterface::interpolate(PyObject* a,PyObject* b,double u)
{
  if(index < 0 || index >= (int)spaces.size() || spaces[index]==NULL) 
    throw PyException("Invalid cspace index");
  Config va,vb,vout;
  if(!PyListToConfig(a,va)) {
    throw PyException("Invalid configuration a (must be list)");
  }
  if(!PyListToConfig(b,vb)) {
    throw PyException("Invalid configuration b (must be list)");
  }
  spaces[index]->Interpolate(va,vb,u,vout);
  return PyListFromConfig(vout);
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
  if(plan < (int)goalSets.size())
    goalSets[plan] = NULL;
  plansDeleteList.push_back(plan);
}

PlannerInterface::PlannerInterface(const CSpaceInterface& cspace)
{
  index = makeNewPlan(cspace.index);
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
  if(index < 0 || index >= (int)plans.size() || plans[index]==NULL) 
    throw PyException("Invalid plan index");
  Config qstart,qgoal;
  bool res=PyListToConfig(start,qstart);
  if(!res) 
    throw PyException("Invalid start endpoint");
  if(!spaces[spaceIndex]->IsFeasible(qstart)) {
    throw PyException("Start configuration is infeasible");
  }
  int istart=plans[index]->AddMilestone(qstart);
  if(istart < 0) {
    throw PyException("Start configuration is infeasible");
  }
  if(istart != 0) {
    throw PyException("Plan already initialized?");
  }

  res=PyListToConfig(goal,qgoal);
  if(res) {
    if(!spaces[spaceIndex]->IsFeasible(qgoal)) {
      throw PyException("Goal configuration is infeasible");
    }
    int igoal=plans[index]->AddMilestone(qgoal);
    if(igoal < 0) {
      throw PyException("Goal configuration is infeasible");
    }
  }
  else {
    //test if it's a goal test
    if(PyCallable_Check(goal)) {
      goalSets.resize(plans.size());
      goalSets[index] = new PyGoalSet(spaces[spaceIndex],goal);
      plans[index]=factory.Create(spaces[spaceIndex],qstart,goalSets[index]);
    }
    else {
      throw PyException("Invalid goal endpoint");
    }
  }
  return true;
}

int PlannerInterface::addMilestone(PyObject* milestone)
{
  if(index < 0 || index >= (int)plans.size() || plans[index]==NULL) 
    throw PyException("Invalid plan index");
  Config q;
  bool res=PyListToConfig(milestone,q);
  if(!res) 
    throw PyException("Invalid milestone provided to addMilestone");
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
    PyList_SetItem(pypath,(Py_ssize_t)i,PyListFromConfig(path.GetMilestone(i)));
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
    PyList_SetItem(pypath,(Py_ssize_t)i,PyListFromConfig(path.GetMilestone(i)));
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
