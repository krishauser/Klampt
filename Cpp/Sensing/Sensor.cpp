#include "Sensor.h"
#include "JointSensors.h"
#include "ForceSensors.h"
#include "InertialSensors.h"
#include "VisualSensors.h"
#include "OtherSensors.h"
#include "Common_Internal.h"
#include "Simulation/Simulator.h"
#include <KrisLibrary/utils/PropertyMap.h>
#include <tinyxml.h>

DECLARE_LOGGER(XmlParser);
DEFINE_LOGGER(Sensing);

using namespace Klampt;
using namespace GLDraw;




SensorBase::SensorBase()
  :name("Unnamed sensor"),rate(0),enabled(true)
{}

bool SensorBase::ReadState(File& f)
{
  vector<double> values;
  if(!ReadFile(f,values)) {
    LOG4CXX_WARN(GET_LOGGER(Sensing),"SensorBase::ReadState: Unable to read values");
    return false;
  }
  SetMeasurements(values);
  vector<double> state;
  if(!ReadFile(f,state)) {
    LOG4CXX_WARN(GET_LOGGER(Sensing),"SensorBase::ReadState: Unable to read internal state");
    return false;
  }
  SetInternalState(state);
  size_t n;
  if(!ReadFile(f,n)) {
    LOG4CXX_WARN(GET_LOGGER(Sensing),"SensorBase::ReadState: Unable to read property size");
    return false;
  }
  for(size_t i=0;i<n;i++) {
    string key,value;
    if(!ReadFile(f,key)) {
      LOG4CXX_WARN(GET_LOGGER(Sensing),"SensorBase::ReadState: Unable to read property key "<<i);
      return false;
    }
    if(!ReadFile(f,value)) {
      LOG4CXX_WARN(GET_LOGGER(Sensing),"SensorBase::ReadState: Unable to read property value "<<i);
      return false;
    }
    SetSetting(key,value);
  }
  return true;
}

bool SensorBase::WriteState(File& f) const
{
  vector<double> values;
  GetMeasurements(values);
  if(!WriteFile(f,values)) return false;
  vector<double> state;
  GetInternalState(state);
  if(!WriteFile(f,state)) return false;
  map<string,string> settings;
  size_t n=settings.size();
  if(!WriteFile(f,n)) return false;
  for(map<string,string>::const_iterator i=settings.begin();i!=settings.end();i++) {
    if(!WriteFile(f,i->first)) return false;
    if(!WriteFile(f,i->second)) return false;
  }
  return true;
}

map<string,string> SensorBase::Settings() const
{
  map<string,string> settings;
  FILL_SENSOR_SETTING(settings,rate);
  FILL_SENSOR_SETTING(settings,enabled);
  return settings;
}
bool SensorBase::GetSetting(const string& name,string& str) const
{
  GET_SENSOR_SETTING(rate);
  GET_SENSOR_SETTING(enabled);
  return false;
}

bool SensorBase::SetSetting(const string& name,const string& str)
{
  SET_SENSOR_SETTING(rate);
  SET_SENSOR_SETTING(enabled);
  return false;
}



























bool RobotSensors::LoadSettings(const char* fn)
{
  TiXmlDocument doc;
  if(!doc.LoadFile(fn)) return false;
  return LoadSettings(doc.RootElement());
}

bool RobotSensors::SaveSettings(const char* fn)
{
  TiXmlDocument doc;
  SaveSettings(doc.RootElement());
  return doc.SaveFile(fn);
}

shared_ptr<SensorBase> RobotSensors::CreateByType(const char* type) const
{
  if(0==strcmp(type,"JointPositionSensor")) {
    return make_shared<JointPositionSensor>();
  }
  else if(0==strcmp(type,"JointVelocitySensor")) {
    return make_shared<JointVelocitySensor>();
  }
  else if(0==strcmp(type,"DriverTorqueSensor")) {
    return make_shared<DriverTorqueSensor>();
  }
  else if(0==strcmp(type,"GyroSensor")) {
    return make_shared<GyroSensor>();
  }
  else if(0==strcmp(type,"Accelerometer")) {
    return make_shared<Accelerometer>();
  }
  else if(0==strcmp(type,"TiltSensor")) {
    return make_shared<TiltSensor>();
  }
  else if(0==strcmp(type,"IMUSensor")) {
    return make_shared<IMUSensor>();
  }
  else if(0==strcmp(type,"ContactSensor")) {
    return make_shared<ContactSensor>();
  }
  else if(0==strcmp(type,"ForceTorqueSensor")) {
    return make_shared<ForceTorqueSensor>();
  }
  else if(0==strcmp(type,"LaserRangeSensor")) {
    return make_shared<LaserRangeSensor>();
  }
  else if(0==strcmp(type,"CameraSensor")) {
    return make_shared<CameraSensor>();
  }
  else if(0==strcmp(type,"TransformedSensor")) {
    return make_shared<TransformedSensor>();
  }
  else if(0==strcmp(type,"CorruptedSensor")) {
    return make_shared<CorruptedSensor>();
  }
  else if(0==strcmp(type,"FilteredSensor")) {
    return make_shared<FilteredSensor>();
  }
  else if(0==strcmp(type,"TimeDelayedSensor")) {
    return make_shared<TimeDelayedSensor>();
  }
  return shared_ptr<SensorBase>();

}

bool RobotSensors::LoadSettings(TiXmlElement* node)
{
  if(0!=strcmp(node->Value(),"sensors")){
    LOG4CXX_ERROR(GET_LOGGER(XmlParser),"RobotSensors::LoadSettings: unable to load from xml file, no <sensors> tag");
    return false;
  }
  TiXmlElement* e=node->FirstChildElement();
  sensors.resize(0);
  while(e != NULL) {
    shared_ptr<SensorBase> sensor = CreateByType(e->Value());
    if(!sensor) {
      LOG4CXX_ERROR(GET_LOGGER(XmlParser),"RobotSensors::LoadSettings: Unknown sensor type "<<e->Value());
      return false;
    }
    sensors.push_back(sensor);
    set<string> processedAttributes;
    if(0==strcmp(e->Value(),"TransformedSensor")) {
      auto fs = dynamic_cast<TransformedSensor*>(sensor.get());
      if(!e->Attribute("sensor")) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Transformed sensor doesn't have a \"sensor\" attribute");
        return false;
      }
      fs->sensor = GetNamedSensor(e->Attribute("sensor"));
      if(fs->sensor == NULL) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Transformed sensor has unknown sensor named \""<<e->Attribute("sensor"));
        return false;
      }
      processedAttributes.insert("sensor");
    }
    else if(0==strcmp(e->Value(),"CorruptedSensor")) {
      auto fs = dynamic_cast<CorruptedSensor*>(sensor.get());
      if(!e->Attribute("sensor")) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Corrupted sensor doesn't have a \"sensor\" attribute");
        return false;
      }
      fs->sensor = GetNamedSensor(e->Attribute("sensor"));
      if(fs->sensor == NULL) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Corrupted sensor has unknown sensor named \""<<e->Attribute("sensor"));
        return false;
      }
      processedAttributes.insert("sensor");
    }
    else if(0==strcmp(e->Value(),"FilteredSensor")) {
      auto fs = dynamic_cast<FilteredSensor*>(sensor.get());
      if(!e->Attribute("sensor")) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Filtered sensor doesn't have a \"sensor\" attribute");
        return false;
      }
      fs->sensor = GetNamedSensor(e->Attribute("sensor"));
      if(fs->sensor == NULL) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Filtered sensor has unknown sensor named \""<<e->Attribute("sensor")<<"\"");
        return false;
      }
      processedAttributes.insert("sensor");
    }
    else if(0==strcmp(e->Value(),"TimeDelayedSensor")) {
      auto fs = dynamic_cast<TimeDelayedSensor*>(sensor.get());
      if(!e->Attribute("sensor")) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Time-delayed sensor doesn't have a \"sensor\" attribute");
        return false;
      }
      fs->sensor = GetNamedSensor(e->Attribute("sensor"));
      if(fs->sensor == NULL) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Time-delayed sensor has unknown sensor named \""<<e->Attribute("sensor")<<"\"");
        return false;
      }
      processedAttributes.insert("sensor");
    }
    TiXmlAttribute* attr = e->FirstAttribute();
    while(attr != NULL) {
      if(processedAttributes.count(attr->Name())!=0) {
        //pass
      }
      else if(0==strcmp(attr->Name(),"name")) {
        sensor->name = attr->Value();
      }
      else if(!sensor->SetSetting(attr->Name(),attr->Value())) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Error setting sensor "<<e->Value()<<" attribute "<<attr->Name()<<", doesn't exist?");
        map<string,string> s = sensor->Settings();
        LOG4CXX_INFO(GET_LOGGER(XmlParser),"Candidates:");
        for(map<string,string>::const_iterator i=s.begin();i!=s.end();i++)
          LOG4CXX_INFO(GET_LOGGER(XmlParser),"  "<<i->first<<" : "<<i->second<<" by default");
        return false;
      }
      attr = attr->Next();
    }
    e = e->NextSiblingElement();
  }
  LOG4CXX_INFO(GET_LOGGER(XmlParser),"RobotSensors::LoadSettings: loaded "<<sensors.size()<<" sensors from XML");
  return true;
}

void RobotSensors::SaveSettings(TiXmlElement* node)
{
  node->SetValue("sensors");
  for(size_t i=0;i<sensors.size();i++) {
    TiXmlElement c(sensors[i]->Type());
    node->SetAttribute("name",sensors[i]->name.c_str());
    PropertyMap settings = sensors[i]->Settings();
    settings.Save(&c);
    node->InsertEndChild(c);
  }
}

bool RobotSensors::LoadMeasurements(TiXmlElement* node)
{
  if(0!=strcmp(node->Value(),"measurement")) return false;
  TiXmlElement* e=node->FirstChildElement();
  while(e != NULL) {
    SensorBase* s = GetNamedSensor(e->Value()).get();
    if(!s) {
      LOG4CXX_ERROR(GET_LOGGER(XmlParser),"RobotSensors::LoadMeasurements: No sensor named"<<e->Value());
      return false;
    }
    vector<string> measurementNames;
    s->MeasurementNames(measurementNames);
    vector<double> measurementValues(measurementNames.size(),0);
    PropertyMap attrs;
    attrs.Load(e);
    for(size_t i=0;i<measurementNames.size();i++) {
      if(attrs.count(measurementNames[i]) == 0) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"RobotSensors::LoadMeasurements: No measurement of"<<e->Value()<<" of name "<<measurementNames[i]);
        return false;
      }
      stringstream ss(attrs[measurementNames[i]]);
      ss >> measurementValues[i];
    }
    s->SetMeasurements(measurementValues);

    e = e->NextSiblingElement();
  }
  return true;
}

void RobotSensors::SaveMeasurements(TiXmlElement* node)
{
  node->SetValue("measurement");
  for(size_t i=0;i<sensors.size();i++) {
    TiXmlElement c(sensors[i]->name.c_str());
    vector<string> measurementNames;
    sensors[i]->MeasurementNames(measurementNames);
    vector<double> measurementValues(measurementNames.size());
    sensors[i]->GetMeasurements(measurementValues);
    for(size_t i=0;i<measurementNames.size();i++) {
      stringstream ss; ss << measurementValues[i];
      c.SetAttribute(measurementNames[i].c_str(),ss.str().c_str());
    }
    node->InsertEndChild(c);
  }
}

bool RobotSensors::ReadState(File& f)
{
  for(size_t i=0;i<sensors.size();i++)
    if(!sensors[i]->ReadState(f)) return false;
  return true;
}

bool RobotSensors::WriteState(File& f) const
{
  for(size_t i=0;i<sensors.size();i++)
    if(!sensors[i]->WriteState(f)) return false;
  return true;
}

shared_ptr<SensorBase> RobotSensors::GetNamedSensor(const string& name)
{
  for(size_t i=0;i<sensors.size();i++)
    if(name == sensors[i]->name) return sensors[i];
  return NULL;
}



void RobotSensors::MakeDefault(RobotModel* robot)
{
  sensors.resize(0);
  string sensorXml;
  if(robot->properties.get("sensors",sensorXml)) {
    TiXmlElement n("sensors");
    stringstream ss(sensorXml);
    ss>>n;
    if(ss) {
      //for any named links, convert them to indices
      TiXmlElement* c = n.FirstChildElement();
      while(c) {
        if(c->Attribute("link")) {
          int ind = robot->LinkIndex(c->Attribute("link"));
          if(ind >= 0) {
            c->SetAttribute("link",ind);
          }
        }
        c = c->NextSiblingElement();
      }
      if(LoadSettings(&n)) {
        //be sure to resize any empty JointPositionSensor's and 
        //JointVelocitySensor's 
        vector<JointPositionSensor*> jps;
        GetTypedSensors(jps);
        for(size_t i=0;i<jps.size();i++)
          if(jps[i]->indices.empty())
            jps[i]->q.resize(robot->q.n,Zero);
        vector<JointVelocitySensor*> jvs;
        GetTypedSensors(jvs);
        for(size_t i=0;i<jvs.size();i++)
          if(jvs[i]->indices.empty())
            jvs[i]->dq.resize(robot->q.n,Zero);
        return;
      }
    }
    LOG4CXX_WARN(GET_LOGGER(Sensing),"RobotSensors::MakeDefault: invalid sensor data format "<<sensorXml);
    LOG4CXX_WARN(GET_LOGGER(Sensing),"   Making the standard sensors instead");
    sensors.resize(0);
  }
  auto jp = make_shared<JointPositionSensor>();
  auto jv = make_shared<JointVelocitySensor>();
  jp->name = "q";
  jv->name = "dq";
  jp->q.resize(robot->q.n,Zero);
  jv->dq.resize(robot->q.n,Zero);
  sensors.push_back(jp);
  sensors.push_back(jv);
}
