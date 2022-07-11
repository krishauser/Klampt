#include "PyController.h"
#include <KrisLibrary/errors.h>

namespace Klampt {

#if HAVE_PYTHON

PyObject* PyListFromVector(const Vector& x) {
	// HACK: gcc doesn't recognize the existence of x.size() (?!)
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
		return NULL;
}

Vector PyListToVector(PyObject* list)
{
  Vector res;
  if(!PySequence_Check(list)) {
    return res;
  }
  res.resize(PySequence_Size(list));
  for(Py_ssize_t i = 0; i < PySequence_Size(list); i++) {
    res[(int)i] = PyFloat_AsDouble(PySequence_GetItem(list, i));
  }
  return res;
}


PyController::PyController(RobotModel& robot)
  :RobotController(robot)
{
  module = updateFunc = resetFunc = getStateFunc = setStateFunc = getSettingsFunc = setSettingsFunc = NULL;
}

bool PyController::Load(const string& _moduleName)
{
  if(module) Unload();

  moduleName = _moduleName;
  PyObject* pName = PyString_FromString(moduleName.c_str());
  module = PyImport_Import(pName);
  Py_DECREF(pName);
  if(!module) {
    fprintf(stderr,"PyController: Couldn't load module %s\n",moduleName.c_str());
    return false;
  }
  else {
    resetFunc = PyObject_GetAttrString(module,"reset");
    updateFunc = PyObject_GetAttrString(module,"update");
    getStateFunc = PyObject_GetAttrString(module,"getState");
    setStateFunc = PyObject_GetAttrString(module,"setState");
    getSettingsFunc = PyObject_GetAttrString(module,"getSettings");
    setSettingsFunc = PyObject_GetAttrString(module,"setSettings");
    if(resetFunc && !PyCallable_Check(resetFunc)) {
      fprintf(stderr,"PyController: %s.reset is not callable\n",moduleName.c_str());
      Py_DECREF(resetFunc);
    }
    if(updateFunc && !PyCallable_Check(updateFunc)) {
      fprintf(stderr,"PyController: %s.update is not callable\n",moduleName.c_str());
      Py_DECREF(updateFunc);
    }
    if(getStateFunc && !PyCallable_Check(getStateFunc)) {
      fprintf(stderr,"PyController: %s.getState is not callable\n",moduleName.c_str());
      Py_DECREF(getStateFunc);
    }
    if(setStateFunc && !PyCallable_Check(setStateFunc)) {
      fprintf(stderr,"PyController: %s.setState is not callable\n",moduleName.c_str());
      Py_DECREF(setStateFunc);
    }
    if(getSettingsFunc && !PyCallable_Check(getSettingsFunc)) {
      fprintf(stderr,"PyController: %s.getSettings is not callable\n",moduleName.c_str());
      Py_DECREF(getSettingsFunc);
    }
    if(setSettingsFunc && !PyCallable_Check(setSettingsFunc)) {
      fprintf(stderr,"PyController: %s.setSettings is not callable\n",moduleName.c_str());
      Py_DECREF(setSettingsFunc);
    }

    //get commands
    PyObject* commandsFunc = PyObject_GetAttrString(module,"commands");
    if(commandsFunc && PyCallable_Check(commandsFunc)) {
      PyObject* res = PyObject_CallFunction(commandsFunc,NULL);
      if(!res) {
	fprintf(stderr,"PyController: %s.commands() failed\n",moduleName.c_str());
      }
      else {
	if(PySequence_Check(res)) {
	  Py_ssize_t n=PySequence_Size(res);
	  commandFuncs.resize(n);
	  commandFuncNames.resize(n);
	  for(Py_ssize_t i=0;i<n;i++) {
	    PyObject* elem = PySequence_GetItem(res,i);
	    if(!elem || !PyString_Check(elem)) {
	      fprintf(stderr,"PyController: element %d of %s.commands() not a string\n",(int)i,moduleName.c_str());
	      commandFuncs[i] = NULL;
	    }
	    else {
	      char* c=PyString_AsString(elem);
	      Assert(c != NULL);
	      commandFuncNames[i] = c;
	      commandFuncs[i] = PyObject_GetAttr(module,elem);
	      if(!commandFuncs[i]) {
		fprintf(stderr,"PyController: command of %s.%s not available",moduleName.c_str(),c);
	      }
	      else if(!PyCallable_Check(commandFuncs[i])) {
		fprintf(stderr,"PyController: command %s.%s is not a callable object",moduleName.c_str(),c);
		Py_DECREF(commandFuncs[i]);
		commandFuncs[i] = NULL;
	      }
	    }
	  }
	}
	else {
	  fprintf(stderr,"PyController: %s.commands() did not return a sequence\n",moduleName.c_str());
	}
      }
      Py_XDECREF(res);
    }
    Py_XDECREF(commandsFunc);
    return true;
  }
}

PyController::~PyController()
{
  Unload();
}

void PyController::Unload()
{
  Py_XDECREF(resetFunc);
  Py_XDECREF(updateFunc);
  Py_XDECREF(getStateFunc);
  Py_XDECREF(setStateFunc);
  Py_XDECREF(getSettingsFunc);
  Py_XDECREF(setSettingsFunc);
  for(size_t i=0;i<commandFuncs.size();i++)
    Py_XDECREF(commandFuncs[i]);
  Py_XDECREF(module);
  commandFuncs.resize(0);
  commandFuncNames.resize(0);
  module = updateFunc = resetFunc = getStateFunc = setStateFunc = getSettingsFunc = setSettingsFunc = NULL;
  moduleName = "";
}

void PyController::Update(Real dt)
{
  if(updateFunc) {
    PyObject* dict = PyDict_New();

    //time
    PyObject* key = PyString_FromString("t");
    PyObject* value = PyFloat_FromDouble(time);
    PyDict_SetItem(dict,key,value);
    Py_DECREF(key);
    Py_DECREF(value);

    //dt
    key = PyString_FromString("dt");
    value = PyFloat_FromDouble(dt);
    PyDict_SetItem(dict,key,value);
    Py_DECREF(key);
    Py_DECREF(value);

    //qcmd
    for(size_t i=0;i<command->actuators.size();i++) {
      if(command->actuators[i].mode == ActuatorCommand::PID)
	robot.SetDriverValue(i,command->actuators[i].qdes);
      else {
	//FatalError("Can't get commanded config for non-config drivers");
      }
    }
    Config qcmd = robot.q;
    key = PyString_FromString("qcmd");
    value = PyListFromVector(qcmd);
    PyDict_SetItem(dict,key,value);
    Py_DECREF(key);
    Py_DECREF(value);

    //sensor data
    for(size_t i=0;i<sensors->sensors.size();i++) {
      key = PyString_FromString(sensors->sensors[i]->name.c_str());
      vector<double> measurements;
      sensors->sensors[i]->GetMeasurements(measurements);
      value = PyListFromVector(measurements);
      PyDict_SetItem(dict,key,value);
      Py_DECREF(key);
      Py_DECREF(value);
    }

    //call the update function
    PyObject* args = PyTuple_Pack(1,dict);
    PyObject* res=PyObject_CallObject(updateFunc,args);

    //parse the result
    if(PyDict_Check(res)) {
      PyObject* qcmd = PyDict_GetItemString(res,"qcmd");
      PyObject* dqcmd = PyDict_GetItemString(res,"dqcmd");
      //PyObject* tcmd = PyDict_GetItemString(res,"tcmd");
      PyObject* torquecmd = PyDict_GetItemString(res,"torquecmd");
      Vector vqcmd,vdqcmd,vtorquecmd;
      //Real dt = 0;
      if(qcmd) 
	vqcmd = PyListToVector(qcmd);
      if(dqcmd) 
	vdqcmd = PyListToVector(dqcmd);
      if(torquecmd) 
	vtorquecmd = PyListToVector(torquecmd);
      /*
	if(tcmd) {
	if(!PyNumber_Check(tcmd)) {
	  fprintf(stderr,"Python module %s.update didn't return 'tcmd' as a number\n",moduleName);
	  dt = PyFloat_AsDouble(tcmd);
	}
      }
      */
      if(!qcmd && !dqcmd && !torquecmd) {
	fprintf(stderr,"Python module %s.update doesn't return valid command item\n",moduleName.c_str());
      }
      if(qcmd) {
	robot.NormalizeAngles(vqcmd);
	robot.q = vqcmd;
	if(vdqcmd) 
	  robot.dq = vdqcmd;
	else
	  robot.dq.setZero();
      }
      for(size_t i=0;i<robot.drivers.size();i++) {
	if(qcmd) {
	  command->actuators[i].SetPID(robot.GetDriverValue(i),robot.GetDriverVelocity(i),command->actuators[i].iterm);
	  if(torquecmd)
	    command->actuators[i].torque = vtorquecmd[i];
	}
	else if(torquecmd) {
	  command->actuators[i].SetTorque(vtorquecmd[i]);
	}
      }
    }
    else {
      fprintf(stderr,"Python module %s.update doesn't return dictionary\n",moduleName.c_str());
    }

    Py_DECREF(res);
    Py_DECREF(args);
    Py_DECREF(dict);
    Py_DECREF(res);
  }
  RobotController::Update(dt);
}

void PyController::Reset()
{
  if(resetFunc) {
    PyObject* res=PyObject_CallFunction(resetFunc,NULL);
    Py_DECREF(res);
  }
  RobotController::Reset();
}

bool PyController::ReadState(File& f)
{
  if(!RobotController::ReadState(f)) return false;
  if(setStateFunc && getStateFunc) {
    int size;
    if(!ReadFile(f,size)) return false;
    char* buf = new char[size+1];
    if(!f.ReadString(buf,size)) {
      delete [] buf;
      return false;
    }
    buf[size]=0;
    PyObject* res=PyObject_CallFunction(setStateFunc,"s",buf);
    delete [] buf;

    if(res == NULL) return false;
    bool retVal = true;
    if(res == Py_False) retVal = false;
    Py_DECREF(res);
    return retVal;
  }
  return true;
}

bool PyController::WriteState(File& f) const
{
  if(!RobotController::WriteState(f)) return false;
  if(setStateFunc && getStateFunc) {
    PyObject* pData = PyObject_CallFunction(getStateFunc,"");
    if(!pData) return false;
    char* buf = PyString_AsString(pData);
    int n = strlen(buf);
    if(!WriteFile(f,n)) {
      Py_DECREF(pData);
      return false;
    }
    if(!f.WriteString(buf)) {
      Py_DECREF(pData);
      return false;
    }
    Py_DECREF(pData);
    return true;
  }
  return true;
}

map<string,string> PyController::Settings() const 
{
  map<string,string> settings;
  settings["module"]=moduleName;
  if(getSettingsFunc) {
    PyObject* pMap = PyObject_CallFunction(getSettingsFunc,"");
    if(pMap) {
      if(PyMapping_Check(pMap)) {
	PyObject* keys = PyMapping_Keys(pMap);
	PyObject* values = PyMapping_Values(pMap);
	Py_ssize_t n=PySequence_Length(keys);
	for(Py_ssize_t i=0;i<n;i++) {
	  PyObject* key = PySequence_GetItem(keys,i);
	  PyObject* value = PySequence_GetItem(values,i);
	  settings[PyString_AsString(key)] = PyString_AsString(value);
	  Py_DECREF(key);
	  Py_DECREF(values);
	}
	Py_DECREF(keys);
	Py_DECREF(values);
	Py_DECREF(pMap);
      }
      else {
	fprintf(stderr,"PyController: %s.getSettings failed to return map type\n",moduleName.c_str());
      }
      Py_DECREF(pMap);
    }
  }
  return RobotController::Settings();
}

bool PyController::GetSetting(const string& name,string& str) const
{
  if(name=="module") { str=moduleName; return true; }
  else {
    map<string,string> settings = Settings();
    if(settings.count(name)>0) {
      str = settings[name];
      return true;
    }
  }
  return RobotController::GetSetting(name,str);
}

bool PyController::SetSetting(const string& name,const string& str)
{
  if(name=="module") { return Load(str); }
  else if(setSettingsFunc && getSettingsFunc) {
    map<string,string> settings = Settings();
    if(settings.count(name) > 0) {
      settings[name] = str;
      PyObject* dict = PyDict_New();
      for(map<string,string>::const_iterator i=settings.begin();i!=settings.end();i++) {
	PyObject* key = PyString_FromString(i->first.c_str());
	PyObject* value = PyString_FromString(i->second.c_str());
	PyDict_SetItem(dict,key,value);
	Py_DECREF(key);
	Py_DECREF(value);
      }
      PyObject* args = PyTuple_Pack(1,dict);
      PyObject* res = PyObject_CallObject(setSettingsFunc,args);
      Py_DECREF(res);
      Py_DECREF(args);
      Py_DECREF(dict);
    }
  }
  return RobotController::SetSetting(name,str);
}

vector<string> PyController::Commands() const
{
  return commandFuncNames;
}

bool PyController::SendCommand(const string& name,const string& str)
{
  for(size_t i=0;i<commandFuncNames.size();i++) {
    if(commandFuncNames[i] == name) {
      PyObject* res = PyObject_CallFunction(commandFuncs[i],"s",str.c_str());
      bool retVal = (res != Py_False);
      Py_DECREF(res);
      return retVal;
    }
  }
  return RobotController::SendCommand(name,str);
}

#else

PyController::PyController(RobotModel& robot)
  :RobotController(robot)
{
  fprintf(stderr,"Python not enabled, cannot instantiate PyControllers\n");
}

bool PyController::Load(const string& _moduleName) { return false; }
void  PyController::Unload() {}
PyController::~PyController() {}

void PyController::Update(Real dt)
{
  RobotController::Update(dt);
}

void PyController::Reset()
{
  RobotController::Reset();
}

bool PyController::ReadState(File& f)
{
  return RobotController::ReadState(f);
}

bool PyController::WriteState(File& f) const
{
  return RobotController::WriteState(f);
}

map<string,string> PyController::Settings() const 
{
  return RobotController::Settings();
}

bool PyController::GetSetting(const string& name,string& str) const
{
  return RobotController::GetSetting(name,str);
}

bool PyController::SetSetting(const string& name,const string& str)
{
  return RobotController::SetSetting(name,str);
}

vector<string> PyController::Commands() const
{
  return RobotController::Commands();
}

bool PyController::SendCommand(const string& name,const string& str)
{
  return RobotController::SendCommand(name,str);
}

} //namespace Klampt

#endif
