#include "Sensor.h"
#include "JointSensors.h"
#include "ForceSensors.h"
#include "InertialSensors.h"
#include "VisualSensors.h"
#include "OtherSensors.h"
#include "Common_Internal.h"
#include "Simulation/WorldSimulation.h"
#include <KrisLibrary/utils/PropertyMap.h>
#include <tinyxml.h>

DECLARE_LOGGER(XmlParser);
DEFINE_LOGGER(Sensing);

using namespace GLDraw;




SensorBase::SensorBase()
  :name("Unnamed sensor"),rate(0)
{}

bool SensorBase::ReadState(File& f)
{
  vector<double> values;
  if(!ReadFile(f,values)) return false;
  SetMeasurements(values);
  vector<double> state;
  if(!ReadFile(f,state)) return false;
  SetInternalState(state);
  size_t n;
  if(!ReadFile(f,n)) return false;
  for(size_t i=0;i<n;i++) {
    string key,value;
    if(!ReadFile(f,key)) return false;
    if(!ReadFile(f,value)) return false;
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
  return settings;
}
bool SensorBase::GetSetting(const string& name,string& str) const
{
  GET_SENSOR_SETTING(rate);
  return false;
}

bool SensorBase::SetSetting(const string& name,const string& str)
{
  SET_SENSOR_SETTING(rate);
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

bool RobotSensors::LoadSettings(TiXmlElement* node)
{
  if(0!=strcmp(node->Value(),"sensors")){
    LOG4CXX_ERROR(GET_LOGGER(XmlParser),"RobotSensors::LoadSettings: unable to load from xml file, no <sensors> tag");
    return false;
  }
  TiXmlElement* e=node->FirstChildElement();
  sensors.resize(0);
  while(e != NULL) {
    shared_ptr<SensorBase> sensor;
    set<string> processedAttributes;
    if(0==strcmp(e->Value(),"JointPositionSensor")) {
      sensor = make_shared<JointPositionSensor>();
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"JointVelocitySensor")) {
      sensor = make_shared<JointVelocitySensor>();
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"DriverTorqueSensor")) {
      sensor = make_shared<DriverTorqueSensor>();
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"GyroSensor")) {
      sensor = make_shared<GyroSensor>();
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"Accelerometer")) {
      sensor = make_shared<Accelerometer>();
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"TiltSensor")) {
      sensor = make_shared<TiltSensor>();
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"IMUSensor")) {
      sensor = make_shared<IMUSensor>();
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"ContactSensor")) {
      sensor = make_shared<ContactSensor>();
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"ForceTorqueSensor")) {
      sensor = make_shared<ForceTorqueSensor>();
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"LaserRangeSensor")) {
      sensor = make_shared<LaserRangeSensor>();
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"CameraSensor")) {
      sensor = make_shared<CameraSensor>();
      sensors.push_back(sensor);
    }
    else if(0==strcmp(e->Value(),"TransformedSensor")) {
      auto fs = make_shared<TransformedSensor>();
      if(!e->Attribute("sensor")) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Transformed sensor doesn't have a \"sensor\" attribute");
        return false;
      }
      fs->sensor = GetNamedSensor(e->Attribute("sensor"));
      if(fs->sensor == NULL) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Transformed sensor has unknown sensor named \""<<e->Attribute("sensor"));
        return false;
      }
      sensor = fs;
      sensors.push_back(sensor);
      processedAttributes.insert("sensor");
    }
    else if(0==strcmp(e->Value(),"CorruptedSensor")) {
      auto fs = make_shared<CorruptedSensor>();
      if(!e->Attribute("sensor")) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Corrupted sensor doesn't have a \"sensor\" attribute");
        return false;
      }
      fs->sensor = GetNamedSensor(e->Attribute("sensor"));
      if(fs->sensor == NULL) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Corrupted sensor has unknown sensor named \""<<e->Attribute("sensor"));
        return false;
      }
      sensor = fs;
      sensors.push_back(sensor);
      processedAttributes.insert("sensor");
    }
    else if(0==strcmp(e->Value(),"FilteredSensor")) {
      auto fs = make_shared<FilteredSensor>();
      if(!e->Attribute("sensor")) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Filtered sensor doesn't have a \"sensor\" attribute");
        return false;
      }
      fs->sensor = GetNamedSensor(e->Attribute("sensor"));
      if(fs->sensor == NULL) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Filtered sensor has unknown sensor named \""<<e->Attribute("sensor")<<"\"");
        return false;
      }
      sensor = fs;
      sensors.push_back(sensor);
      processedAttributes.insert("sensor");
    }
    else if(0==strcmp(e->Value(),"TimeDelayedSensor")) {
      auto fs = make_shared<TimeDelayedSensor>();
      if(!e->Attribute("sensor")) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Time-delayed sensor doesn't have a \"sensor\" attribute");
        return false;
      }
      fs->sensor = GetNamedSensor(e->Attribute("sensor"));
      if(fs->sensor == NULL) {
        LOG4CXX_ERROR(GET_LOGGER(XmlParser),"Time-delayed sensor has unknown sensor named \""<<e->Attribute("sensor")<<"\"");
        return false;
      }
      sensor = fs;
      sensors.push_back(sensor);
      processedAttributes.insert("sensor");
    }
    else {
      LOG4CXX_ERROR(GET_LOGGER(XmlParser),"RobotSensors::LoadSettings: Unknown sensor type "<<e->Value());
      return false;
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



void RobotSensors::MakeDefault(Robot* robot)
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
