#ifndef GENERIC_GUI_H
#define GENERIC_GUI_H

#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/math3d/primitives.h>
#include <string>
#include <sstream>

namespace Klampt {
  using namespace std;

class GenericGUIBase;
class GenericBackendBase;

/** @brief A base class for a GUI frontend.  Performs message passing to the
 * backend in the easily serializable AnyCollection format.
 * 
 * Along with a GenericBackendBase, implements a GUI program.
 *
 * Empty hooks for several default GUI functions are provided.  Subclasses
 * should implement these hooks.
 *
 * Currently, only OpenGL is allowed for drawing.  Implementations might
 * pass geometry data back and forth from frontend to backend, or a
 * shared canvas object could be passed to the backend on construction.
 */
class GenericGUIBase
{
 public:
  GenericGUIBase(GenericBackendBase* backend);
  virtual ~GenericGUIBase() {}
  ///Default implementation does nothing but startup and shutdown the interface
  virtual void Run();
  ///This callback will be called by the interface. The default implementation
  ///dispatches to the On[X] callbacks depending on the value of the msg.type
  ///item 
  virtual bool ProcessMessage(const AnyCollection& msg);
  ///Send a message to the interface
  virtual bool SendMessage(const AnyCollection& msg);

  ///Message mappings: translate one message to another via wildcard-matched rules
  ///(see AnyMapping.h for more details).  The constants _Any, _0, _1, ..., _10
  ///can be used to capture wildcards.
  void AddRule(const AnyCollection& inschema,const AnyCollection& outschema,bool mapWildcardStrings=true);
  ///Convenience function to map a wildcard rule to a command (emulating RPC)
  void AddCommandRule(const AnyCollection& inschema,const string& cmd,const string& args,bool mapWildcardStrings=true);
  bool LoadRules(const char* fn);
  bool LoadRules(istream& in);

  virtual bool OnQuit() { return false; }
  virtual bool OnCommand(const string& cmd,const string& args) { return false; }
  virtual bool OnNotify(const string& text,const string& msglevel) { return false; }
  virtual bool OnPauseIdle(double secs) { return false; }
  virtual bool OnRefresh() { return false; }
  virtual bool OnResize(int w,int h) { return false; }
  virtual bool OnDrawText(double x, double y, double z, const std::string &text, int height) { return false; }
  virtual bool OnDrawText(int x,int y, const std::string &text, int height) { return false; }

  bool SendIdle();
  bool SendGLRender();
  bool SendGLViewport(int x,int y,int w,int h);
  bool SendCommand(const string& cmd,const string& args);
  bool SendCommand(const string& cmd);
  template <class T>
  bool SendCommand(const string& cmd,const T& arg1);
  template <class T1,class T2>
  bool SendCommand(const string& cmd,const T1& arg1,const T2& arg2);
  template <class T1,class T2,class T3>
  bool SendCommand(const string& cmd,const T1& arg1,const T2& arg2,const T3& arg3);
  bool SendButtonPress(const string& widget);
  bool SendButtonToggle(const string& widget,int checked);
  bool SendWidgetValue(const string& widget,const string& value);
  bool SendMouseClick(int button,int state,int mx,int my);
  bool SendMouseMove(int mx,int my);
  bool SendMouseWheel(int dwheel);
  bool SendScroll(int dy);
  bool SendKeyDown(const string& key);
  bool SendKeyUp(const string& key);
  bool SendSpaceball(const Math3D::RigidTransform& T);
  bool SendDevice(const string& name,const string& data);

  GenericBackendBase* backend;
  vector<pair<AnyCollection,AnyCollection> > rules;
};

class GenericBackendBase
{
 public:
  GenericBackendBase();
  virtual ~GenericBackendBase() {}
  ///Default implementation of following do nothing
  virtual void Start() {}
  virtual void Stop() {}
  ///This callback will be called by the gui.  The default implementation
  ///dispatches to the On[X] callbacks depending on the msg.type string.
  virtual bool ProcessMessage(const AnyCollection& msg);
  ///Send a message to the gui
  virtual bool SendMessage(const AnyCollection& msg) { return gui->ProcessMessage(msg); }

  //"live" button mappings
  void MapButtonPress(const string& button,int* var);
  void MapButtonToggle(const string& button,int* var);
  void MapWidgetValue(const string& button,string* var);
  void MapKeyToggle(const string& key,int* var);

  virtual bool OnIdle() { SendPauseIdle(); return true; }
  virtual bool OnGLRender() { return false; }
  virtual bool OnGLViewport(int x,int y,int w, int h) { return false; }
  virtual bool OnCommand(const string& cmd,const string& args) { return false; }
  virtual bool OnButtonPress(const string& button);
  virtual bool OnButtonToggle(const string& button,int checked);
  virtual bool OnWidgetValue(const string& widget,const string& value);
  virtual bool OnMouseClick(int button,int state,int mx,int my) { return false; }
  virtual bool OnMouseMove(int mx,int my) { return false; }
  virtual bool OnMouseWheel(int dwheel) { return false; }
  virtual bool OnScroll(int dy) { return false; }
  virtual bool OnKeyDown(const string& key);
  virtual bool OnKeyUp(const string& key);
  virtual bool OnSpaceball(const Math3D::RigidTransform& T) { return false; }
  virtual bool OnDevice(const string& name,const string& data) { return false; }

  bool SendQuit();
  bool SendCommand(const string& cmd,const string& args);
  bool SendNotify(const string& text,const string& msglevel="");
  bool SendError(const string& text) { return SendNotify(text,"error"); }
  bool SendWarning(const string& text) { return SendNotify(text,"warning"); }
  bool SendPauseIdle(double secs=1e300);
  bool SendRefresh();
  bool SendResize(int w,int h);
  bool SendDrawText(double x, double y, double z, const std::string &text, int height = 10);
  bool SendDrawText(int x, int y, const std::string &text, int height = 10);

  GenericGUIBase* gui;
  map<string,int*> liveButtonPresses;
  map<string,int*> liveButtonToggles;
  map<string,string*> liveWidgetValues;
  map<string,int*> liveKeys;
};

/** Composes a bunch of backends together. 
 *
 * On ProcessMessage, the message type is checked for whether it's in the
 * list of targets. If not, the list of children are called in order until
 * one successfully processes the message.  If target[type]=="all" then all
 * children are sent the message.  If it's an integer, then only that child
 * is sent that message.
 *
 * This method is bit more modular than manual subclassing but manual
 * subclassing gives more control over which messages get sent where.
 *
 * Default broadcasts for now include:
 * - GLRender
 */
class CompositeBackend : public GenericBackendBase
{
 public:
  void Add(std::shared_ptr<GenericBackendBase>& backend);
  void SetDefaultBroadcasts();
  void SetBroadcast(const string& msgtype) { targets[msgtype]="all"; }
  virtual bool ProcessMessage(const AnyCollection& msg);

  vector<std::shared_ptr<GenericBackendBase> > children;
  map<string,string> targets;
};


/** @brief The DISPATCH macros let you easily dispatch messages to callbacks
 * in the ProcessMessage function.
 */

#define DISPATCHBEGIN() \
  bool res; \
  string type; \
  res = msg["type"].as<string>(type); \
  if(!res) { \
    cerr<<"Message doesn't contain type"<<endl; \
    return false; \
  } \

#define DISPATCHEND() \
  else { \
    cerr<<"Unhandled message type "<<type<<endl; \
    return false; \
  }


#define DISPATCH0(msgtype,func)		\
  else if(type == msgtype)	\
    return func();  

#define DISPATCH1(msgtype,arg1,func) 		\
  else if(type == msgtype) \
    return func(msg[#arg1]);			


#define DISPATCH2(msgtype,arg1,arg2,func) 		\
  else if(type == msgtype) \
    return func(msg[#arg1],msg[#arg2]);	     

#define DISPATCH3(msgtype,arg1,arg2,arg3,func)	\
  else if(type == msgtype) \
    return func(msg[#arg1],msg[#arg2],msg[#arg3]);	

#define DISPATCH4(msgtype,arg1,arg2,arg3,arg4,func)	\
  else if(type == msgtype) \
    return func(msg[#arg1],msg[#arg2],msg[#arg3],msg[#arg4]);  

#define DISPATCH5(msgtype,arg1,arg2,arg3,arg4,arg5,func)	\
  else if(type == msgtype) \
    return func(msg[#arg1],msg[#arg2],msg[#arg3],msg[#arg4],msg[#arg5]);  


/** @brief The SEND macros let you easily package messages to be sent to the
 * frontend/backend.
 */

#define SEND0(msgtype)\
  AnyCollection msg; \
  msg["type"] = string(msgtype); \
  return SendMessage(msg);

#define SEND1(msgtype,arg1)			\
  AnyCollection msg; \
  msg["type"] = string(msgtype); \
  msg[#arg1] = arg1; \
  return SendMessage(msg);

#define SEND2(msgtype,arg1,arg2)			\
  AnyCollection msg; \
  msg["type"] = string(msgtype); \
  msg[#arg1] = arg1; \
  msg[#arg2] = arg2; \
  return SendMessage(msg);

#define SEND3(msgtype,arg1,arg2,arg3)			\
  AnyCollection msg; \
  msg["type"] = string(msgtype); \
  msg[#arg1] = arg1; \
  msg[#arg2] = arg2; \
  msg[#arg3] = arg3; \
  return SendMessage(msg);

#define SEND4(msgtype,arg1,arg2,arg3,arg4)		\
  AnyCollection msg; \
  msg["type"] = string(msgtype); \
  msg[#arg1] = arg1; \
  msg[#arg2] = arg2; \
  msg[#arg3] = arg3; \
  msg[#arg4] = arg4; \
  return SendMessage(msg);

#define SEND5(msgtype,arg1,arg2,arg3,arg4,arg5)		\
  AnyCollection msg; \
  msg["type"] = string(msgtype); \
  msg[#arg1] = arg1; \
  msg[#arg2] = arg2; \
  msg[#arg3] = arg3; \
  msg[#arg4] = arg4; \
  msg[#arg5] = arg5; \
  return SendMessage(msg);


template <class T>
bool GenericGUIBase::SendCommand(const string& cmd,const T& arg1)
{
  stringstream ss;
  ss<<arg1;
  return this->SendCommand(cmd,ss.str());
}

template <class T1,class T2>
bool GenericGUIBase::SendCommand(const string& cmd,const T1& arg1,const T2& arg2)
{
  stringstream ss;
  ss<<arg1<<" "<<arg2;
  return this->SendCommand(cmd,ss.str());
}

template <class T1,class T2,class T3>
bool GenericGUIBase::SendCommand(const string& cmd,const T1& arg1,const T2& arg2,const T3& arg3)
{
  stringstream ss;
  ss<<arg1<<" "<<arg2<<" "<<arg3<<endl;
  return this->SendCommand(cmd,ss.str());
}

} //namespace Klampt

#endif 
