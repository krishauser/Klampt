#include "ROS.h"

#if HAVE_ROS 

#include "Modeling/World.h"
#include "Modeling/Paths.h"
#include "Simulation/Simulator.h"
#include <KrisLibrary/meshing/PointCloud.h>
#include "Simulation/SimRobotController.h"
#include "Sensing/Sensor.h"
#include "Sensing/VisualSensors.h"
#include "Sensing/ForceSensors.h"
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/Logger.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace Klampt {

bool IsBigEndian() {
  int n = 1;
  // little endian if true
  if(*(char *)&n == 1) return false;
  return true;
}

bool ROSToKlampt(const geometry_msgs::Point& pt,Vector3& kp)
{
  kp.x = pt.x;
  kp.y = pt.y;
  kp.z = pt.z;
  return true;
}


bool KlamptToROS(const Vector3& kp,geometry_msgs::Point& p)
{
  p.x = kp.x;
  p.y = kp.y;
  p.z = kp.z;
  return true;
}

bool ROSToKlampt(const geometry_msgs::Quaternion& q,Matrix3& kR)
{
  QuaternionRotation kq;
  kq.x = q.x;
  kq.y = q.y;
  kq.z = q.z;
  kq.w = q.w;
  kq.getMatrix(kR);
  return true;
}

bool KlamptToROS(const Matrix3& kR,geometry_msgs::Quaternion& q)
{
  QuaternionRotation kq;
  if(!kq.setMatrix(kR)) return false;
  q.x = kq.x;
  q.y = kq.y;
  q.z = kq.z;
  q.w = kq.w;
  return true;
}

bool ROSToKlampt(const geometry_msgs::Pose& pose,RigidTransform& kT)
{
  ROSToKlampt(pose.position,kT.t);
  ROSToKlampt(pose.orientation,kT.R);
  return true;
}

bool ROSToKlampt(const geometry_msgs::PoseStamped& pose,RigidTransform& kT)
{
  ROSToKlampt(pose.pose.position,kT.t);
  ROSToKlampt(pose.pose.orientation,kT.R);
  return true;
}

bool KlamptToROS(const RigidTransform& kT,geometry_msgs::Pose& pose)
{
  if(!KlamptToROS(kT.t,pose.position)) return false;
  if(!KlamptToROS(kT.R,pose.orientation)) return false;
  return true;
}

bool KlamptToROS(const RigidTransform& kT,geometry_msgs::PoseStamped& pose)
{
  if(!KlamptToROS(kT.t,pose.pose.position)) return false;
  if(!KlamptToROS(kT.R,pose.pose.orientation)) return false;
  return true;
}

bool ROSToKlampt(const sensor_msgs::JointState& js,RobotModel& krobot)
{
  map<string,int> indices;
  for(size_t i=0;i<krobot.linkNames.size();i++)
    indices[krobot.linkNames[i]] = (int)i;
  for(size_t i=0;i<js.name.size();i++) {
    if(indices.count(js.name[i])==0) {
      fprintf(stderr,"ROS JointState message has incorrect name %s\n",js.name[i].c_str());
      return false;
    }
    if(!js.position.empty()) krobot.q[indices[js.name[i]]] = js.position[i];
    if(!js.velocity.empty()) krobot.dq[indices[js.name[i]]] = js.velocity[i];
  }
  krobot.UpdateFrames();
  return true;
}

bool KlamptToROS(const RobotModel& krobot,sensor_msgs::JointState& js)
{
  js.name = krobot.linkNames;
  js.position.resize(krobot.linkNames.size());
  js.velocity.resize(krobot.linkNames.size());
  for(size_t i=0;i<krobot.linkNames.size();i++) {
    js.position[i] = krobot.q[i];
    js.velocity[i] = krobot.dq[i];
  }
  return true;
}

bool ROSToKlampt(const std_msgs::Float32MultiArray& msg,vector<double>& kvec)
{
  kvec.resize(msg.data.size());
  for(size_t i=0;i<kvec.size();i++)
    kvec[i] = msg.data[i];
  return true;
}

bool KlamptToROS(const vector<double>& kvec,std_msgs::Float32MultiArray& msg)
{
  msg.data.resize(kvec.size());
  for(size_t i=0;i<kvec.size();i++)
    msg.data[i] = kvec[i];
  return true;
}


bool CommandedKlamptToROS(SimRobotController& kcontroller,sensor_msgs::JointState& js)
{
  Config qcmd,vcmd,t;
  kcontroller.GetCommandedConfig(qcmd);
  kcontroller.GetCommandedVelocity(vcmd);
  kcontroller.GetLinkTorques(t);
  RobotModel& krobot = *kcontroller.robot;
  js.name = krobot.linkNames;
  js.position.resize(krobot.linkNames.size());
  js.velocity.resize(krobot.linkNames.size());
  js.effort.resize(krobot.linkNames.size(),0.0);
  for(size_t i=0;i<krobot.linkNames.size();i++) {
    js.position[i] = qcmd[i];
    js.velocity[i] = vcmd[i];
    js.effort[i] = t[i];
  }
  return true;
}

bool SensedKlamptToROS(SimRobotController& kcontroller,sensor_msgs::JointState& js)
{
  Config qcmd,vcmd,t;
  kcontroller.GetSensedConfig(qcmd);
  kcontroller.GetSensedVelocity(vcmd);
  kcontroller.GetLinkTorques(t);
  RobotModel& krobot = *kcontroller.robot;
  js.name = krobot.linkNames;
  js.position.resize(krobot.linkNames.size());
  js.velocity.resize(krobot.linkNames.size());
  js.effort.resize(krobot.linkNames.size(),0.0);
  for(size_t i=0;i<krobot.linkNames.size();i++) {
    js.position[i] = qcmd[i];
    js.velocity[i] = vcmd[i];
    js.effort[i] = t[i];
  }
  return true;
}

bool ROSToKlampt(const trajectory_msgs::JointTrajectory& traj,LinearPath& kpath)
{
  kpath.times.resize(traj.points.size());
  kpath.milestones.resize(traj.points.size());
  for(size_t i=0;i<traj.points.size();i++) {
    kpath.times[i] = traj.points[i].time_from_start.toSec();
    kpath.milestones[i] = traj.points[i].positions;
  }
  return true;
}

bool KlamptToROS(const LinearPath& kpath,trajectory_msgs::JointTrajectory& traj)
{
  if(kpath.milestones.empty()) {
    traj.joint_names.clear();
    traj.points.clear();
    return true;
  }
  traj.joint_names.resize(kpath.milestones[0].n);
  for(int i=0;i<kpath.milestones[0].n;i++)
    traj.joint_names[i] = '0'+i;
  traj.points.resize(kpath.milestones.size());
  for(size_t i=0;i<kpath.milestones.size();i++) {
    traj.points[i].time_from_start = ros::Duration(kpath.times[i]);
    traj.points[i].positions = kpath.milestones[i];
  }
  return true;
}

bool KlamptToROS(const RobotModel& robot,const LinearPath& kpath,trajectory_msgs::JointTrajectory& traj)
{
  if(kpath.milestones.empty()) {
    traj.joint_names.clear();
    traj.points.clear();
    return true;
  }
  if((int)robot.linkNames.size() != kpath.milestones[0].size()) {
    fprintf(stderr,"KlamptToROS (LinearPath): path doesn't have same number of milestones as the robot\n");
    return false;
  }
  traj.joint_names = robot.linkNames;
  traj.points.resize(kpath.milestones.size());
  for(size_t i=0;i<kpath.milestones.size();i++) {
    traj.points[i].time_from_start = ros::Duration(kpath.times[i]);
    traj.points[i].positions = kpath.milestones[i];
  }
  return true;
}

bool KlamptToROS(const RobotModel& robot,const vector<int>& indices,const LinearPath& kpath,trajectory_msgs::JointTrajectory& traj)
{
  if(kpath.milestones.empty()) {
    traj.joint_names.clear();
    traj.points.clear();
    return true;
  }
  if((int)indices.size() != kpath.milestones[0].size()) {
    fprintf(stderr,"KlamptToROS (LinearPath): path doesn't have same number of milestones as the indices\n");
    return false;
  }
  traj.joint_names.resize(indices.size());
  for(size_t i=0;i<indices.size();i++) {
    if(indices[i] < 0 || indices[i] > robot.q.n)  {
      fprintf(stderr,"KlamptToROS (LinearPath): invalid index\n");
      return false;
    }
    traj.joint_names[i] = robot.linkNames[indices[i]];
  }
  traj.points.resize(kpath.milestones.size());
  for(size_t i=0;i<kpath.milestones.size();i++) {
    traj.points[i].time_from_start = ros::Duration(kpath.times[i]);
    traj.points[i].positions = kpath.milestones[i];
  }
  return true;
}

// Swap 2 byte, 16 bit values:
#define Swap2Bytes(val) \
 ( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) )
// Swap 4 byte, 32 bit values:

#define Swap4Bytes(val) \
 ( (((val) >> 24) & 0x000000FF) | (((val) >>  8) & 0x0000FF00) | \
   (((val) <<  8) & 0x00FF0000) | (((val) << 24) & 0xFF000000) )

// Swap 8 byte, 64 bit values:
#define Swap8Bytes(val) \
 ( (((val) >> 56) & 0x00000000000000FF) | (((val) >> 40) & 0x000000000000FF00) | \
   (((val) >> 24) & 0x0000000000FF0000) | (((val) >>  8) & 0x00000000FF000000) | \
   (((val) <<  8) & 0x000000FF00000000) | (((val) << 24) & 0x0000FF0000000000) | \
   (((val) << 40) & 0x00FF000000000000) | (((val) << 56) & 0xFF00000000000000) )

#define Swap2If(val,cond) (cond ? Swap2Bytes(val) : val)
#define Swap4If(val,cond) (cond ? Swap4Bytes(val) : val)
#define Swap8If(val,cond) (cond ? Swap8Bytes(val) : val)


template <class T> 
void UNPACK(const sensor_msgs::PointField& field,const unsigned char* data,T* out,bool swap_bigendian)
{
  static const int datasizes[] = {1,1,2,2,4,4,4,8};
  int stride = datasizes[field.datatype];
  data += field.offset;
  for(size_t i=0;i<field.count;i++) {
    switch(field.datatype) {
    case sensor_msgs::PointField::INT8:
      out[i] = T(*(char*)data);
      break;
    case sensor_msgs::PointField::UINT8:
      out[i] = T(*data);
      break;
    case sensor_msgs::PointField::INT16:
      out[i] = T(Swap2If(*(short*)data,swap_bigendian));
      break;
    case sensor_msgs::PointField::UINT16:
      out[i] = T(Swap2If(*(unsigned short*)data,swap_bigendian));
      break;
    case sensor_msgs::PointField::INT32:
      out[i] = T(Swap4If(*(int*)data,swap_bigendian));
      break;
    case sensor_msgs::PointField::UINT32:
      out[i] = T(Swap4If(*(unsigned int*)data,swap_bigendian));
      break;
    case sensor_msgs::PointField::FLOAT32:
    {
      int bytes = Swap4If(*(int*)data,swap_bigendian);
      unsigned char* bptr=((unsigned char*)&bytes);
      out[i] = T(*(float*)bptr);
      break;
    }
    case sensor_msgs::PointField::FLOAT64:
    {
      int64_t bytes = Swap8If(*(int64_t*)data,swap_bigendian);
      unsigned char* bptr=((unsigned char*)&bytes);
      out[i] = T(*(double*)bptr);
      break;
    }
    }
    data += stride;
  }
}

bool ROSToKlampt(const sensor_msgs::PointCloud2& pc,Meshing::PointCloud3D& kpc)
{
  int xfield=-1,yfield=-1,zfield=-1;
  int rgbfloat_field=-1;
  int rgbproperty=-1;
  vector<int> fieldmap(pc.fields.size(),-1);
  bool structured = false;
  if(pc.height > 1) {
    structured = true;
    kpc.settings.set("width",pc.width);
    kpc.settings.set("height",pc.height);
  }
  kpc.points.resize(0);
  kpc.propertyNames.resize(0);
  kpc.properties.resize(0);
  bool swap_bigendian = (IsBigEndian() != pc.is_bigendian);
  for(size_t i=0;i<pc.fields.size();i++) {
    if(pc.fields[i].name == "x") { xfield=(int)i; Assert(pc.fields[i].count==1); }
    else if(pc.fields[i].name == "y") { yfield=(int)i; Assert(pc.fields[i].count==1); }
    else if(pc.fields[i].name == "z") { zfield=(int)i; Assert(pc.fields[i].count==1); }
    else {
      fieldmap[i] = (int)kpc.propertyNames.size();
      if((pc.fields[i].name == "rgb" || pc.fields[i].name == "rgba" ) && pc.fields[i].datatype == sensor_msgs::PointField::FLOAT32) {
        //custom crap for Kinect2 bridge sending UINTs in float format
        rgbfloat_field = (int)i;
        rgbproperty = kpc.propertyNames.size();
        Assert(pc.fields[i].count == 1);
        fieldmap[i] = -1;
      }
      if(pc.fields[i].count==1) kpc.propertyNames.push_back(pc.fields[i].name);
      else {
        for(size_t j=0;j<pc.fields[i].count;j++) {
          char suffix = '0'+j;
          kpc.propertyNames.push_back(pc.fields[i].name+suffix);
        }
      }
    }
  }
  Assert(pc.data.size() >= pc.row_step*pc.height);
  int ofs = 0;
  Vector3 pt(0.0);
  Vector propertyTemp(kpc.propertyNames.size());
  for(unsigned int i=0;i<pc.height;i++) {
    int vofs = ofs;
    for(unsigned int j=0;j<pc.width;j++) {
      if(xfield >=0) UNPACK<Real>(pc.fields[xfield],&pc.data[vofs],&pt.x,swap_bigendian);
      if(yfield >=0) UNPACK<Real>(pc.fields[yfield],&pc.data[vofs],&pt.y,swap_bigendian);
      if(zfield >=0) UNPACK<Real>(pc.fields[zfield],&pc.data[vofs],&pt.z,swap_bigendian);
      if(structured || (IsFinite(pt.x) && IsFinite(pt.y) && IsFinite(pt.z))) {
        kpc.points.push_back(pt);
        if(rgbfloat_field >= 0) {
          //hack
          const unsigned char* data = &pc.data[vofs]+pc.fields[rgbfloat_field].offset;
          unsigned int rgb = Swap4If(*((unsigned int*)data),swap_bigendian);
          Real* out = &propertyTemp[rgbproperty];
          *out = Real(rgb);
        }
        int pofs = 0;
        for(size_t k=0;k<pc.fields.size();k++) {
          if(fieldmap[k] < 0) continue;
          UNPACK<Real>(pc.fields[k],&pc.data[vofs],&propertyTemp[pofs],swap_bigendian);
          pofs +=pc.fields[k].count;
        }
        kpc.properties.push_back(propertyTemp);
      }
      vofs += pc.point_step;
    }
    ofs += pc.row_step;
  }
  //printf("Read %d points from ROS\n",kpc.points.size());
  return true;
}

bool KlamptToROS(const Meshing::PointCloud3D& kpc,sensor_msgs::PointCloud2& pc)
{
  pc.is_bigendian = IsBigEndian();
  if(kpc.IsStructured()) {
    pc.height = kpc.GetStructuredHeight();
    pc.width = kpc.GetStructuredWidth();
  }
  else {
    pc.height = 1;
    pc.width = kpc.points.size();
  }
  pc.point_step = 4*(3+kpc.propertyNames.size());
  pc.row_step = pc.width*pc.point_step;
  pc.fields.resize(3+kpc.propertyNames.size());
  pc.fields[0].name = "x";
  pc.fields[1].name = "y";
  pc.fields[2].name = "z";
  for(size_t i=0;i<kpc.propertyNames.size();i++)
    pc.fields[3+i].name = kpc.propertyNames[i];
  for(size_t i=0;i<pc.fields.size();i++) {
    pc.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    pc.fields[i].offset = i*4;
    pc.fields[i].count = 1;
  }
  int ofs = 0;
  pc.data.resize(pc.row_step);
  for(size_t i=0;i<kpc.points.size();i++) {
    *(float*)&pc.data[ofs] = kpc.points[i].x; ofs += 4;
    *(float*)&pc.data[ofs] = kpc.points[i].y; ofs += 4;
    *(float*)&pc.data[ofs] = kpc.points[i].z; ofs += 4;
    for(size_t j=0;j<kpc.propertyNames.size();j++) {
      *(float*)&pc.data[ofs] = kpc.properties[i][j]; ofs += 4;
    }
  }
  return true;
}

bool ROSToKlampt(const tf::Transform& T,RigidTransform& kT)
{
  kT.t.set(T.getOrigin().x(),T.getOrigin().y(),T.getOrigin().z());
  Vector3 row1(T.getBasis()[0].x(),T.getBasis()[0].y(),T.getBasis()[0].z());
  Vector3 row2(T.getBasis()[1].x(),T.getBasis()[1].y(),T.getBasis()[1].z());
  Vector3 row3(T.getBasis()[2].x(),T.getBasis()[2].y(),T.getBasis()[2].z());
  kT.R.setRow1(row1);
  kT.R.setRow2(row2);
  kT.R.setRow3(row3);
  return true;
}

bool KlamptToROS(const RigidTransform& kT,tf::Transform& T)
{
  T.getOrigin().setValue(kT.t.x,kT.t.y,kT.t.z);
  Vector3 row;
  kT.R.getRow1(row);
  T.getBasis()[0].setValue(row.x,row.y,row.z);
  kT.R.getRow2(row);
  T.getBasis()[1].setValue(row.x,row.y,row.z);
  kT.R.getRow3(row);
  T.getBasis()[2].setValue(row.x,row.y,row.z);
  return true;
}



unique_ptr<ros::NodeHandle> gRosNh;
int gRosQueueSize = 1;
bool gRosSubscribeError = false;
string gRosSubscribeErrorWhere;

class ROSSubscriberBase
{
public:
  ROSSubscriberBase() : error(false),numMessages(0) {}
  virtual ~ROSSubscriberBase() { unsubscribe(); }
  void unsubscribe() {
    this->topic = "";
    this->numMessages = 0;
    sub = ros::Subscriber();
  }
  virtual void endUpdate() {}

  ros::Subscriber sub;
  string topic;
  bool error;
  std_msgs::Header header;
  int numMessages;
};

class ROSPublisherBase
{
public:
  virtual ~ROSPublisherBase() {}
  ros::Publisher pub;
  string topic;
};

typedef map<string,shared_ptr<ROSSubscriberBase> > SubscriberList;
typedef map<string,shared_ptr<ROSPublisherBase> > PublisherList;
SubscriberList gSubscribers;
PublisherList gPublishers;




template <class Type,class Msg>
class ROSSubscriber : public ROSSubscriberBase
{
public:
  Type& obj;
  Msg msg;
  ROSSubscriber(Type& _obj,const std::string& _topic):obj(_obj) {
    this->topic = _topic;
    sub = gRosNh->subscribe(_topic,gRosQueueSize,&ROSSubscriber<Type,Msg>::callback, this);
  }
  void callback(const Msg& msg) {
    numMessages++;
    header = msg.header;
    //this->msg = msg;
    error = (!ROSToKlampt(msg,obj));
    if(error) {
      gRosSubscribeError = true;
      gRosSubscribeErrorWhere = this->topic;
    }
  }
  virtual void endUpdate() {
    /*
    error = (!ROSToKlampt(msg,obj));
    if(error) {
      gRosSubscribeError = true;
      gRosSubscribeErrorWhere = this->topic;
    }
    */
  }
};


class ROSTfSubscriber : public ROSSubscriberBase
{
public:
  tf::TransformListener listener;
  map<string,RigidTransform*> transforms;
  ROSTfSubscriber():listener(*gRosNh) { this->topic = "tf"; }
  void update() {
    tf::StampedTransform transform;
    try{
      for(map<string,RigidTransform*>::iterator i=transforms.begin();i!=transforms.end();i++) {
	listener.lookupTransform(i->first, "world",  
				 ros::Time::now(), transform);
	ROSToKlampt(transform,*i->second);
      }
    }
    catch (tf::TransformException ex){
      gRosSubscribeError = true;
      gRosSubscribeErrorWhere = "tf";
    }
  }
};


template <class Msg>
class ROSPublisher : public ROSPublisherBase
{
public:
  Msg msg;
  ROSPublisher(const std::string& topic) {
    this->topic = topic;
    pub = gRosNh->advertise<Msg>(topic,gRosQueueSize,false);
    msg.header.seq = 0;
    msg.header.frame_id = "0";
  }
  template <class Type>
  void publish(const Type& obj) {
    //ignore if no subscribers
    if(pub.getNumSubscribers () == 0) return;
    //do conversion if there's a subscriber
    msg.header.stamp = ros::Time::now();
    msg.header.seq++;
    KlamptToROS(obj,msg);
    pub.publish(msg);
  }
  ///assumes that you've updated the meaningful parts of msg
  void publish_current() {
    //ignore if no subscribers
    if(pub.getNumSubscribers () == 0) return;
    //do conversion if there's a subscriber
    msg.header.stamp = ros::Time::now();
    msg.header.seq++;
    pub.publish(msg);
  }
};

template <class Msg>
class ROSRawPublisher : public ROSPublisherBase
{
public:
  Msg msg;
  ROSRawPublisher(const std::string& topic) {
    this->topic = topic;
    pub = gRosNh->advertise<Msg>(topic,gRosQueueSize,false);
  }
  template <class Type>
  void publish(const Type& obj) {
    //ignore if no subscribers
    if(pub.getNumSubscribers () == 0) return;
    KlamptToROS(obj,msg);
    pub.publish(msg);
  }
  ///assumes that you've updated the meaningful parts of msg
  void publish_current() {
    //ignore if no subscribers
    if(pub.getNumSubscribers () == 0) return;
    //do conversion if there's a subscriber
    pub.publish(msg);
  }

};


class ROSTfPublisher : public ROSPublisherBase
{
public:
  tf::TransformBroadcaster broadcaster;
  ROSTfPublisher() { this->topic = "tf"; }
  void send(const string& name,const RigidTransform& T,const char* parent="world") {
    tf::Transform transform;
    bool valid = KlamptToROS(T,transform);
    if(!valid) {
      printf("RosPublishTransforms: transform of %s is not valid?\n",name.c_str());
      return;
    }
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent, name));
  }
};


bool ROSInit(const char* nodeName)
{
  if(gRosNh) return true;
  int argc = 1;
  char* argv [1]={(char*)"klampt"}; 
  ros::init(argc, &argv[0], nodeName);
  gRosNh.reset(new ros::NodeHandle);
  return true;
}

bool ROSInitialized()
{
  return (gRosNh != NULL);
}

bool ROSShutdown()
{
  if(gRosNh) {
    ros::shutdown();
    gRosNh = NULL;
  }
  return true;
}

bool ROSSetQueueSize(int size)
{
  if(size <= 0) return false;
  gRosQueueSize = size;
  return true;
}

template<class Type,class Msg>
bool RosSubscribe(Type& obj,const string& topic)
{
  if(!ROSInit()) return false;
  SubscriberList::iterator i=gSubscribers.find(topic); 
  if(i!=gSubscribers.end()) { 
    printf("ROSSubscribe: Unsubscribing old subscriber to topic %s\n",topic.c_str());
    i->second->unsubscribe();
    i->second = NULL;
  }
  auto sub = make_shared<ROSSubscriber<Type,Msg> >(obj,topic); 
  if(!sub->sub) {
    fprintf(stderr,"ROSSubscribe: Unable to subscribe to topic %s, maybe wrong type\n",topic.c_str());
    return false;
  }
  gSubscribers[topic] = sub;
  return true; 
}

template<class Type,class PubType,class Msg>
bool RosPublish2(const Type& obj,const string& topic)
{
  if(!ROSInit()) return false;
  PublisherList::iterator i=gPublishers.find(topic); 
  PubType* pub;
  if(i==gPublishers.end()) { 
    pub = new PubType(topic); 
    gPublishers[topic].reset(pub); 
  } 
  else { 
    pub = dynamic_cast<PubType*>(i->second.get());
    if(!pub) return false;
  }
  pub->publish(obj); 
  return true; 
}



template <class Msg>
ROSPublisher<Msg>* GetPublisher(const char* topic)
{
  if(!ROSInit()) return NULL;
  PublisherList::iterator i=gPublishers.find(topic); 
  ROSPublisher<Msg>* pub;
  if(i==gPublishers.end()) { 
    pub = new ROSPublisher<Msg>(topic); 
    gPublishers[topic].reset(pub); 
  } 
  else { 
    pub = dynamic_cast<ROSPublisher<Msg>*>(i->second.get());
    if(!pub) return NULL;
  }
  return pub;
}

template<class Type,class Msg>
bool RosPublish(const Type& obj,const string& topic)
{
  return RosPublish2<Type,ROSPublisher<Msg>,Msg>(obj,topic);
}

bool ROSPublishTransforms(const WorldModel& world,const char* frameprefix)
{
  if(!ROSInit()) return false;
  string prefix = frameprefix;
  ROSTfPublisher* tf;
  if(gPublishers.count("tf")==0) {
    tf = new ROSTfPublisher();
    gPublishers["tf"].reset(tf);
  }
  else {
    tf = dynamic_cast<ROSTfPublisher*>(gPublishers["tf"].get());
    if(tf==NULL) return false;
  }
  for(size_t i=0;i<world.rigidObjects.size();i++)
    tf->send(prefix+"/"+world.rigidObjects[i]->name,world.rigidObjects[i]->T);
  for(size_t i=0;i<world.robots.size();i++) {
    string rprefix = prefix+"/"+world.robots[i]->name;
    for(size_t j=0;j<world.robots[i]->links.size();j++) {
      int p = world.robots[i]->parents[j];
      if(p < 0) {
        tf->send(rprefix+"/"+world.robots[i]->linkNames[j],world.robots[i]->links[j].T_World);
      }
      else {
        RigidTransform Tparent;
        Tparent.mulInverseA(world.robots[i]->links[p].T_World,world.robots[i]->links[j].T_World);
        tf->send(rprefix+"/"+world.robots[i]->linkNames[j],Tparent,(rprefix+"/"+world.robots[i]->linkNames[j]).c_str());
      }
    }
  }
  return true;
}

bool ROSPublishTransforms(const Simulator& sim,const char* frameprefix)
{
  if(!ROSInit()) return false;
  string prefix = frameprefix;
  ROSTfPublisher* tf;
  if(gPublishers.count("tf")==0) {
    tf = new ROSTfPublisher();
    gPublishers["tf"].reset(tf);
  }
  else {
    tf = dynamic_cast<ROSTfPublisher*>(gPublishers["tf"].get());
    if(tf==NULL) return false;
  }
  RigidTransform T,Tp;
  for(size_t i=0;i<sim.world->rigidObjects.size();i++) {
    sim.odesim.object(i)->GetTransform(T);
    tf->send(prefix+"/"+sim.world->rigidObjects[i]->name,T);
  }
  for(size_t i=0;i<sim.world->robots.size();i++) {
    string rprefix = prefix+"/"+sim.world->robots[i]->name;
    for(size_t j=0;j<sim.world->robots[i]->links.size();j++) {
      sim.odesim.robot(i)->GetLinkTransform(j,T);
      int p = sim.world->robots[i]->parents[j];
      if(p < 0)
        tf->send(rprefix+"/"+sim.world->robots[i]->linkNames[j],T);
      else {
        sim.odesim.robot(i)->GetLinkTransform(p,Tp);
        RigidTransform Trel;
        Trel.mulInverseA(Tp,T);
        tf->send(rprefix+"/"+sim.world->robots[i]->linkNames[j],Trel,(rprefix+"/"+sim.world->robots[i]->linkNames[p]).c_str());
      }
    }
  }
  return true;
}

bool ROSPublishTransforms(const RobotModel& robot,const char* frameprefix)
{
  if(!ROSInit()) return false;
  string prefix = frameprefix;
  ROSTfPublisher* tf;
  if(gPublishers.count("tf")==0) {
    tf = new ROSTfPublisher();
    gPublishers["tf"].reset(tf);
  }
  else {
    tf = dynamic_cast<ROSTfPublisher*>(gPublishers["tf"].get());
    if(tf==NULL) return false;
  }
  for(size_t j=0;j<robot.links.size();j++)  {
    int p = robot.parents[j];
    if(p < 0) {
      tf->send(prefix+"/"+robot.linkNames[j],robot.links[j].T_World);
    }
    else {
      RigidTransform Tparent;
      Tparent.mulInverseA(robot.links[p].T_World,robot.links[j].T_World);
      tf->send(prefix+"/"+robot.linkNames[j],Tparent,(prefix+"/"+robot.linkNames[j]).c_str());
    }
  }
  return true;
}

bool ROSPublishTransform(const RigidTransform& T,const char* frame)
{
  ROSTfPublisher* tf;
  if(gPublishers.count("tf")==0) {
    tf = new ROSTfPublisher();
    gPublishers["tf"].reset(tf);
  }
  else {
    tf = dynamic_cast<ROSTfPublisher*>(gPublishers["tf"].get());
    if(tf==NULL) return false;
  }
  tf->send(frame,T);
  return true;
}

bool ROSPublishPose(const RigidTransform& T,const char* topic)
{
  return RosPublish<RigidTransform,geometry_msgs::PoseStamped>(T,topic);
}

bool ROSPublishJointState(const RobotModel& robot,const char* topic)
{
  return RosPublish<RobotModel,sensor_msgs::JointState>(robot,topic);
}

bool ROSPublishPointCloud(const Meshing::PointCloud3D& pc,const char* topic)
{
  return RosPublish<Meshing::PointCloud3D,sensor_msgs::PointCloud2>(pc,topic);
}

bool ROSPublishTrajectory(const LinearPath& path,const char* topic) 
{
  return RosPublish<LinearPath,trajectory_msgs::JointTrajectory>(path,topic);
}

bool ROSPublishTrajectory(const RobotModel& robot,const LinearPath& path,const char* topic)
{
  if(!ROSInit()) return false;
  ROSPublisher<trajectory_msgs::JointTrajectory>* pub = GetPublisher<trajectory_msgs::JointTrajectory>(topic);
  if(!pub) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error getting JointTrajectory publisher on "<<topic);
    return false;
  }
  //ignore if no subscribers
  if(pub->pub.getNumSubscribers () == 0) return true;
  //do conversion if there's a subscriber
  KlamptToROS(robot,path,pub->msg);
  pub->publish_current();
  return true; 
}

bool ROSPublishTrajectory(const RobotModel& robot,const vector<int>& indices,const LinearPath& path,const char* topic)
{
  if(!ROSInit()) return false;
  ROSPublisher<trajectory_msgs::JointTrajectory>* pub = GetPublisher<trajectory_msgs::JointTrajectory>(topic);
  if(!pub) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error getting JointTrajectory publisher on "<<topic);
    return false;
  }
  //ignore if no subscribers
  if(pub->pub.getNumSubscribers () == 0) return true;
  //do conversion if there's a subscriber
  KlamptToROS(robot,indices,path,pub->msg);
  pub->publish_current();
  return true; 
}

void KlamptToROSCameraInfo(const CameraSensor& cam,sensor_msgs::CameraInfo& msg)
{
  msg.width = cam.xres;
  msg.height = cam.yres;
  msg.distortion_model = "plumb_bob";
  msg.D.resize(5,0.0);
  Real fx = 0.5*cam.xres/Tan(cam.xfov*0.5);
  Real fy = 0.5*cam.yres/Tan(cam.yfov*0.5);
  Real cx = 0.5*cam.xres;
  Real cy = 0.5*cam.yres;
  msg.K[0] = fx;
  msg.K[4] = fy;
  msg.K[8] = 1;
  msg.K[3] = cx;
  msg.K[7] = cy;
  msg.R[0] = 1;
  msg.R[4] = 1;
  msg.R[8] = 1;
  msg.P[0] = fx;
  msg.P[5] = fy;
  msg.P[10] = 1;
  msg.P[3] = cx;
  msg.P[8] = cy;
}


bool ROSPublishSensorMeasurement(const SensorBase* sensor,const char* topic)
{
  RobotModel blank;
  return ROSPublishSensorMeasurement(sensor,blank,topic,NULL);
}
bool ROSPublishSensorMeasurement(const SensorBase* sensor,const RobotModel& robot,const char* topic,const char* frameprefix)
{
  if(!ROSInit()) return false;
  if(0 == strcmp(sensor->Type(),"CameraSensor")) {
    const CameraSensor* camera = dynamic_cast<const CameraSensor*>(sensor);
    string frame = "0";
    if(frameprefix) 
      frame = string(frameprefix) + "/" + robot.name + "/" + robot.linkNames[camera->link];
    
    vector<double> measurements;
    camera->GetMeasurements(measurements);
    if(measurements.empty()) return false;
    if(camera->rgb) {
      ROSPublisher<sensor_msgs::CameraInfo>* pubinfo = GetPublisher<sensor_msgs::CameraInfo>((string(topic)+"/rgb/camera_info").c_str());
      if(!pubinfo) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Error getting CameraInfo publisher on "<<topic<<"/rgb/camera_info");
        return false;
      }
      KlamptToROSCameraInfo(*camera,pubinfo->msg);
      pubinfo->msg.header.frame_id = frame;
      pubinfo->publish_current();
      ROSPublisher<sensor_msgs::Image>* pub = GetPublisher<sensor_msgs::Image>((string(topic)+"/rgb/image_rect_color").c_str());
      if(!pubinfo) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Error getting Image publisher on "<<topic<<"/rgb/image_rect_color");
        return false;
      }
      pub->msg.header.frame_id = frame;
      pub->msg.width = camera->xres;
      pub->msg.height = camera->yres;
      pub->msg.encoding = "rgb8";
      pub->msg.is_bigendian = IsBigEndian();
      pub->msg.step = pub->msg.width*3;
      pub->msg.data.resize(camera->xres*camera->yres*3);
      for(int i=0;i<camera->xres*camera->yres;i++) {
        unsigned int abgr = (unsigned int)measurements[i];
        unsigned char b = (abgr >> 16) & 0xff;
        unsigned char g = (abgr >> 8) & 0xff;
        unsigned char r = (abgr) & 0xff;
        pub->msg.data[i*3]=r;
        pub->msg.data[i*3+1]=g;
        pub->msg.data[i*3+2]=b;
      }
      pub->publish_current();
    }
    if(camera->depth) {
      ROSPublisher<sensor_msgs::CameraInfo>* pubinfo = GetPublisher<sensor_msgs::CameraInfo>((string(topic)+"/depth_registered/camera_info").c_str());
      if(!pubinfo) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Error getting CameraInfo publisher on "<<topic<<"/depth_registered/camera_info");
        return false;
      }
      KlamptToROSCameraInfo(*camera,pubinfo->msg);
      pubinfo->publish_current();
      int ofs = 0;
      if(camera->rgb) ofs = camera->xres*camera->yres;
      ROSPublisher<sensor_msgs::Image>* pub = GetPublisher<sensor_msgs::Image>((string(topic)+"/depth_registered/image_rect").c_str());
      if(!pubinfo) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Error getting Image publisher on "<<topic<<"/depth_registered/image_rect");
        return false;
      }
      pub->msg.width = camera->xres;
      pub->msg.height = camera->yres;
      pub->msg.encoding = "32FC1";
      pub->msg.is_bigendian = IsBigEndian();
      pub->msg.step = pub->msg.width*4;
      pub->msg.data.resize(camera->xres*camera->yres*4);
      for(int i=0;i<camera->xres*camera->yres;i++) {
        double d = measurements[ofs+i];
        *((float*)&pub->msg.data[i*4])=float(d);
      }
      pub->publish_current();
    }
  }
  else if(0 == strcmp(sensor->Type(),"ForceTorqueSensor")) {
    const ForceTorqueSensor* ft = dynamic_cast<const ForceTorqueSensor*>(sensor);
    string frame = "0";
    if(frameprefix) 
      frame = string(frameprefix) + "/" + robot.name + "/" + robot.linkNames[ft->link];

    vector<double> measurements;
    sensor->GetMeasurements(measurements);
    if(measurements.size() != 6) return false;
    ROSPublisher<geometry_msgs::WrenchStamped>* pub = GetPublisher<geometry_msgs::WrenchStamped>(topic);
    pub->msg.header.frame_id = frame;
    pub->msg.wrench.force.x = measurements[0];
    pub->msg.wrench.force.y = measurements[1];
    pub->msg.wrench.force.z = measurements[2];
    pub->msg.wrench.torque.x = measurements[3];
    pub->msg.wrench.torque.y = measurements[4];
    pub->msg.wrench.torque.z = measurements[5];
    pub->publish_current();
  }
  else {
    vector<double> measurements;
    sensor->GetMeasurements(measurements);
    RosPublish2<vector<double>,ROSRawPublisher<std_msgs::Float32MultiArray>,std_msgs::Float32MultiArray>(measurements,topic);
  }
  return true;
}

bool ROSPublishCommandedJointState(SimRobotController& robot,const char* topic)
{
  ROSPublisher<sensor_msgs::JointState>* pub = GetPublisher<sensor_msgs::JointState>(topic);
  if(!pub) return false;
  CommandedKlamptToROS(robot,pub->msg);
  pub->publish_current();
  return true;
}

bool ROSPublishSensedJointState(SimRobotController& robot,const char* topic)
{
  ROSPublisher<sensor_msgs::JointState>* pub = GetPublisher<sensor_msgs::JointState>(topic);
  if(!pub) return false;
  SensedKlamptToROS(robot,pub->msg);
  pub->publish_current();
  return true;
}


bool ROSSubscribeTransforms(WorldModel& world,const char* frameprefix);
bool ROSSubscribeTransforms(RobotModel& robot,const char* frameprefix);
bool ROSSubscribeTransform(RigidTransform& T,const char* frameprefix);
bool ROSSubscribePose(RigidTransform& T,const char* topic)
{
  //TODO: handle un-stamped messages?
  return RosSubscribe<RigidTransform,geometry_msgs::PoseStamped>(T,topic);
}
bool ROSSubscribeJointState(RobotModel& robot,const char* topic)
{
  return RosSubscribe<RobotModel,sensor_msgs::JointState>(robot,topic);
}
bool ROSSubscribePointCloud(Meshing::PointCloud3D& pc,const char* topic)
{
  //printf("ROSSubscribePointCloud %s\n",topic);
  return RosSubscribe<Meshing::PointCloud3D,sensor_msgs::PointCloud2>(pc,topic);
}

bool ROSSubscribeTrajectory(LinearPath& path,const char* topic) 
{
  return RosSubscribe<LinearPath,trajectory_msgs::JointTrajectory>(path,topic);
}

bool ROSSubscribeUpdate()
{
  if(gSubscribers.empty() && gPublishers.empty()) return false; 
  //Timer timer;
  for(SubscriberList::iterator i=gSubscribers.begin();i!=gSubscribers.end();i++)
    i->second->numMessages = 0;
  gRosSubscribeError = false;
  ros::spinOnce();
  //TODO: tf listener is running in background, do we want a delay?
  if(gSubscribers.count("tf") != 0) {
    ROSTfSubscriber* tf=dynamic_cast<ROSTfSubscriber*>(gSubscribers["tf"].get());
    if(tf != NULL) tf->update();
  }
  bool updated = false;
  for(SubscriberList::iterator i=gSubscribers.begin();i!=gSubscribers.end();i++)
    if(i->second->numMessages > 0) {
      //printf("%d updates to %s\n",i->second->numMessages,i->second->topic.c_str());
      updated = true;
      i->second->endUpdate();
    }
  //printf("ROS Update in time %gs\n",timer.ElapsedTime());
  if(gRosSubscribeError) {
    fprintf(stderr,"ROS: Error converting topic %s to Klampt format\n",gRosSubscribeErrorWhere.c_str());
    return false;
  }
  return updated;
}

bool ROSDetach(const char* topic)
{
  if(gSubscribers.count(topic) != 0) {
    gSubscribers[topic] = NULL;
    gSubscribers.erase(gSubscribers.find(topic));
    return true;
  }
  fprintf(stderr,"ROSDetach: topic %s not published/subscribed\n",topic);
  return false;
}

int ROSNumSubscribedTopics() { return (int)gSubscribers.size(); }
int ROSNumPublishedTopics() { return (int)gPublishers.size(); }

bool ROSIsConnected(const char* topic) {
  if(gSubscribers.count(topic) != 0) {
    return gSubscribers[topic]->sub.getNumPublishers() >  0;
  }
  else if(gPublishers.count(topic) != 0) {
    return gPublishers[topic]->pub.getNumSubscribers() >  0;
  }
  return false;
}

std::string ROSFrame(const char* topic)
{
  if(gSubscribers.count(topic) != 0) {
    return gSubscribers[topic]->header.frame_id;
  }
  return "";
}

bool ROSWaitForUpdate(const char* topic,double timeout)
{
  if(gSubscribers.count(topic) == 0) return false;
  auto s = gSubscribers[topic];
  int oldNumMessages = s->numMessages;
  Timer timer;
  while(timer.ElapsedTime() < timeout) {
    ros::spinOnce();
    ros::Duration(Min(timeout-timer.ElapsedTime(),0.001)).sleep();
    if(s->numMessages > oldNumMessages) return true;
  }
  return false;
}

bool ROSHadUpdate(const char* topic)
{
  if(gSubscribers.count(topic) == 0) {
    printf("No subscribers on topic %s\n",topic);
    printf("Valid topics:\n");
    for(auto i = gSubscribers.begin();i!=gSubscribers.end();i++)
      printf("  %s\n",i->first.c_str());
    return false;
  }
  auto s = gSubscribers[topic];
  return s->numMessages > 0;
}


} //namespace Klampt


#else

#include "Modeling/World.h"

namespace Klampt {

bool ROSInit(const char* nodename) { fprintf(stderr,"ROSInit(): Klamp't was not built with ROS support\n"); return false; }
bool ROSShutdown() { return false; }
bool ROSInitialized() { return false; }
bool ROSSetQueueSize(int size) { return false; }
bool ROSPublishTransforms(const WorldModel& world,const char* frameprefix) { return false; }
bool ROSPublishTransforms(const Simulator& sim,const char* frameprefix) { return false; }
bool ROSPublishTransforms(const RobotModel& robot,const char* frameprefix) { return false; }
bool ROSPublishTransform(const RigidTransform& T,const char* frame) { return false; }
bool ROSPublishPose(const RigidTransform& T,const char* topic) { return false; }
bool ROSPublishJointState(const RobotModel& robot,const char* topic) { return false; }
bool ROSPublishPointCloud(const Meshing::PointCloud3D& pc,const char* topic) { return false; }
bool ROSPublishTrajectory(const LinearPath& T,const char* topic) { return false; }
bool ROSPublishTrajectory(const RobotModel& robot,const LinearPath& path,const char* topic) { return false; }
bool ROSPublishTrajectory(const RobotModel& robot,const vector<int>& indices,const LinearPath& path,const char* topic) { return false; }
bool ROSPublishCommandedJointState(SimRobotController& robot,const char* topic) { return false; }
bool ROSPublishSensedJointState(SimRobotController& robot,const char* topic) { return false; }
bool ROSPublishSensorMeasurement(const SensorBase* sensor,const char* topic) { return false; }
bool ROSPublishSensorMeasurement(const SensorBase* sensor,const RobotModel& robot,const char* topic,const char* frameprefix) { return false; }
bool ROSSubscribeTransforms(WorldModel& world,const char* frameprefix) { return false; }
bool ROSSubscribeTransforms(RobotModel& robot,const char* frameprefix) { return false; }
bool ROSSubscribeTransform(RigidTransform& T,const char* frameprefix) { return false; }
bool ROSSubscribePose(RigidTransform& T,const char* topic) { return false; }
bool ROSSubscribeJointState(RobotModel& robot,const char* topic) { return false; }
bool ROSSubscribePointCloud(Meshing::PointCloud3D& pc,const char* topic)  { return false; }
bool ROSSubscribeTrajectory(LinearPath& T,const char* topic) { return false; }
bool ROSSubscribeTrajectory(RobotModel& robot,LinearPath& path,const char* topic) { return false; }
bool ROSSubscribeUpdate() { return false; }
bool ROSDetach(const char* topic) { return false; }
int ROSNumSubscribedTopics() { return 0; }
int ROSNumPublishedTopics() { return 0; }
bool ROSIsConnected(const char* topic) { return false; }
std::string ROSFrame(const char* topic) { return ""; }
bool ROSWaitForUpdate(const char* topic,double timeout) { return false; }
bool ROSHadUpdate(const char* topic) { return false; }

} //namespace Klampt

#endif //HAVE_ROS
