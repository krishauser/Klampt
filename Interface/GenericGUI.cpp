#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "GenericGUI.h"
#include <KrisLibrary/utils/AnyMapper.h>
#include <fstream>
#include <iostream>

#define DEBUG_GUI 0


template <class T>
size_t ReplaceValues(AnyCollection& coll,const T& val,const AnyValue& replacement)
{
  if(!coll.collection()) {
    if((const AnyValue&)coll == val) {
      coll = replacement;
      return 1;
    }
    return 0;
  }
  vector<SmartPointer<AnyCollection> > subelements;
  coll.enumerate(subelements);
  size_t n=0;
  for(size_t i=0;i<subelements.size();i++) {
    n += ReplaceValues(*subelements[i],val,replacement);
  }
  return n;
} 



GenericGUIBase::GenericGUIBase(GenericBackendBase* _backend)
  :backend(_backend)
{
  if(backend)
    backend->gui = this;
}

void GenericGUIBase::Run()
{
  backend->Start();
  backend->Stop();
}


bool GenericGUIBase::LoadRules(const char* fn)
{
  ifstream in(fn);
  if(!in) return false;
  return LoadRules(in);
}

bool GenericGUIBase::LoadRules(istream& in)
{
  AnyCollection rules;
  if(!rules.read(in)) return false;
  for(size_t i=0;i<rules.size();i++) {
    if(rules[i].size() != 2) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"Format error: Rule "<<i<<" isn't a pair"<<"\n");
      return false;
    }
    //LOG4CXX_INFO(KrisLibrary::logger(),"Rule: "<<rules[i][0]<<" -> "<<rules[i][1]<<"\n");
    AddRule(rules[i][0],rules[i][1]);
  }
  return true;
}

void GenericGUIBase::AddCommandRule(const AnyCollection& inschema,const string& cmd,const string& args,bool mapWildcardStrings)
{
  AnyCollection outschema;
  outschema["type"]=string("command");
  outschema["cmd"]=cmd;
  outschema["args"]=args;
  AddRule(inschema,outschema,mapWildcardStrings);
}

void GenericGUIBase::AddRule(const AnyCollection& inschema,const AnyCollection& outschema,bool mapWildcardStrings)
{
  pair<AnyCollection,AnyCollection> rule(inschema,outschema);
  if(mapWildcardStrings) {
    ReplaceValues(rule.first,string("_Any"),_Any);
    ReplaceValues(rule.first,string("_0"),_0);
    ReplaceValues(rule.first,string("_1"),_1);
    ReplaceValues(rule.first,string("_2"),_2);
    ReplaceValues(rule.first,string("_3"),_3);
    ReplaceValues(rule.first,string("_4"),_4);
    ReplaceValues(rule.first,string("_5"),_5);
    ReplaceValues(rule.first,string("_6"),_6);
    ReplaceValues(rule.first,string("_7"),_7);
    ReplaceValues(rule.first,string("_8"),_8);
    ReplaceValues(rule.first,string("_9"),_9);
    ReplaceValues(rule.first,string("_10"),_10);
    ReplaceValues(rule.first,string("_Any"),_Any);
    ReplaceValues(rule.second,string("_0"),_0);
    ReplaceValues(rule.second,string("_1"),_1);
    ReplaceValues(rule.second,string("_2"),_2);
    ReplaceValues(rule.second,string("_3"),_3);
    ReplaceValues(rule.second,string("_4"),_4);
    ReplaceValues(rule.second,string("_5"),_5);
    ReplaceValues(rule.second,string("_6"),_6);
    ReplaceValues(rule.second,string("_7"),_7);
    ReplaceValues(rule.second,string("_8"),_8);
    ReplaceValues(rule.second,string("_9"),_9);
    ReplaceValues(rule.second,string("_10"),_10);
  }
  rules.push_back(rule);
}

bool GenericGUIBase::ProcessMessage(const AnyCollection& msg)
{
  DISPATCHBEGIN()
  DISPATCH0("quit",OnQuit)
  DISPATCH2("command",cmd,args,OnCommand)
  DISPATCH2("notify",text,msglevel,OnNotify)
  DISPATCH1("pause_idle",secs,OnPauseIdle)
  DISPATCH0("refresh",OnRefresh)
  DISPATCH2("resize",w,h,OnResize)
  DISPATCHEND()
}

bool GenericGUIBase::SendMessage(const AnyCollection& msg)
{
  for(size_t i=0;i<rules.size();i++) {
    AnyCollection outmsg;
    if(MatchAndFill(msg,rules[i].first,rules[i].second,outmsg)) {
      if(DEBUG_GUI)
	LOG4CXX_INFO(KrisLibrary::logger(),"GUI->Backend "<<msg<<" matched rule "<<i<<", processed to "<<outmsg<<"\n");
      if(!backend->ProcessMessage(outmsg)) {
	LOG4CXX_INFO(KrisLibrary::logger(),"Message "<<outmsg<<" not processed"<<"\n");
	return false;
      }
      return true;
    }
  }
  //LOG4CXX_INFO(KrisLibrary::logger(),"GUI->Backend "<<msg<<"\n");
  if(!backend->ProcessMessage(msg)) {
    LOG4CXX_INFO(KrisLibrary::logger(),"GUI->Backend Message "<<msg<<" not processed"<<"\n");
    return false;
  }
  return true;
}

bool GenericGUIBase::SendIdle()
{
  SEND0("idle")
}

bool GenericGUIBase::SendGLRender()
{
  SEND0("glrender")
}

bool GenericGUIBase::SendGLViewport(int x,int y,int w,int h)
{
  SEND4("glviewport",x,y,w,h)
}

bool GenericGUIBase::SendCommand(const string& cmd)
{
  return SendCommand(cmd,"");
}

bool GenericGUIBase::SendCommand(const string& cmd,const string& args)
{
  SEND2("command",cmd,args)
}

bool GenericGUIBase::SendButtonPress(const string& button)
{
  SEND1("button_press",button)
}

bool GenericGUIBase::SendButtonToggle(const string& button,int checked)
{
  SEND2("button_toggle",button,checked)
}

bool GenericGUIBase::SendWidgetValue(const string& widget,const string& value)
{
  SEND2("widget_value",widget,value)
}

bool GenericGUIBase::SendSpaceball(const Math3D::RigidTransform& T)
{
  return false;
}

bool GenericGUIBase::SendDevice(const string& name,const string& data)
{
  SEND2("device",name,data)
}

bool GenericGUIBase::SendMouseClick(int button,int state,int mx,int my)
{
  SEND4("mouse_click",button,state,mx,my)
}

bool GenericGUIBase::SendMouseMove(int mx,int my)
{
  SEND2("mouse_move",mx,my)
}

bool GenericGUIBase::SendMouseWheel(int dwheel)
{
  SEND1("mouse_wheel",dwheel)
}


bool GenericGUIBase::SendScroll(int dy)
{
  SEND1("scroll",dy)
}

bool GenericGUIBase::SendKeyUp(const string& key)
{
  SEND1("key_up",key)
}

bool GenericGUIBase::SendKeyDown(const string& key)
{
  SEND1("key_down",key)
}


GenericBackendBase::GenericBackendBase()
  :gui(NULL)
{}

bool GenericBackendBase::ProcessMessage(const AnyCollection& msg)
{
  DISPATCHBEGIN()
  DISPATCH0("idle",OnIdle)
  DISPATCH0("glrender",OnGLRender)
  DISPATCH4("glviewport",x,y,w,h,OnGLViewport)
  DISPATCH2("command",cmd,args,OnCommand)
  DISPATCH1("button_press",button,OnButtonPress)
  DISPATCH2("button_toggle",button,checked,OnButtonToggle)
  DISPATCH2("widget_value",widget,value,OnWidgetValue)
  DISPATCH4("mouse_click",button,state,mx,my,OnMouseClick)
  DISPATCH2("mouse_move",mx,my,OnMouseMove)
  DISPATCH1("mouse_wheel",dwheel,OnMouseWheel)
  DISPATCH1("scroll",dy,OnMouseWheel)
  DISPATCH1("key_down",key,OnKeyDown)
  DISPATCH1("key_up",key,OnKeyUp)
  DISPATCH2("device",name,data,OnDevice)
  DISPATCHEND()
}

bool GenericBackendBase::SendQuit()
{
  SEND0("quit")
}

bool GenericBackendBase::SendCommand(const string& cmd,const string& args)
{
  SEND2("command",cmd,args)
}

bool GenericBackendBase::SendNotify(const string& text,const string& msglevel)
{
  SEND2("notify",text,msglevel)
}

bool GenericBackendBase::SendPauseIdle(double secs)
{
  SEND1("pause_idle",secs)
}

bool GenericBackendBase::SendRefresh()
{
  SEND0("refresh")
}

bool GenericBackendBase::SendResize(int w,int h)
{
  SEND2("resize",w,h)
}

void GenericBackendBase::MapButtonPress(const string& button,int* var)
{ liveButtonPresses[button] = var; }
void GenericBackendBase::MapButtonToggle(const string& button,int* var)
{ liveButtonToggles[button] = var; }
void GenericBackendBase::MapWidgetValue(const string& widget,string* var)
{ liveWidgetValues[widget] = var; }
void GenericBackendBase::MapKeyToggle(const string& key,int* var)
{ liveKeys[key] = var; }


bool GenericBackendBase::OnButtonPress(const string& button)
{
  if(liveButtonPresses.count(button)!=0) {
    *liveButtonPresses[button] = 1;
    SendRefresh();
    return true;
  }
  return false;
}

bool GenericBackendBase::OnButtonToggle(const string& button,int checked)
{
  if(liveButtonToggles.count(button)!=0) {
    *liveButtonToggles[button] = checked;
    SendRefresh();
    return true;
  }
  return false;
}

bool GenericBackendBase::OnWidgetValue(const string& widget,const string& value)
{
 if(liveWidgetValues.count(widget)!=0) {
    *liveWidgetValues[widget] = value;
    SendRefresh();
    return true;
  }
  return false;
}

bool GenericBackendBase::OnKeyDown(const string& key)
{
  if(liveKeys.count(key)!=0) {
    *liveKeys[key] = 1;
    SendRefresh();
    return true;
  }
  return false;
}

bool GenericBackendBase::OnKeyUp(const string& key)
{
  if(liveKeys.count(key)!=0) {
    *liveKeys[key] = 0;
    SendRefresh();
    return true;
  }
  return false;
}


void CompositeBackend::Add(SmartPointer<GenericBackendBase>& backend)
{
  children.push_back(backend);
  children.back()->gui = gui;
}

void CompositeBackend::SetDefaultBroadcasts()
{
  SetBroadcast("glrender");
}

bool CompositeBackend::ProcessMessage(const AnyCollection& msg)
{
  bool res;
  string type;
  res = msg["type"].as<string>(type);
  if(!res) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Message doesn't contain type"<<"\n");
    return false;
  }

  for(size_t i=0;i<children.size();i++)
    if(children[i]->ProcessMessage(msg)) return true;
  return false;
}


