#ifndef ROBOT_RESOURCES_H
#define ROBOT_RESOURCES_H

#include <utils/ResourceLibrary.h>
#include <utils/AnyCollection.h>
#include <math3d/geometry3d.h>
#include "World.h"
#include "MultiPath.h"
#include "Contact/Stance.h"
#include "Contact/Grasp.h"
using namespace std;

typedef BasicResource<Config> ConfigResource;
typedef BasicResource<Vector3> Vector3Resource;
typedef BasicResource<Matrix3> Matrix3Resource;
typedef BasicResource<Matrix> MatrixResource;
typedef BasicResource<RigidTransform> RigidTransformResource;
typedef BasicResource<GeometricPrimitive3D> GeometricPrimitive3DResource;
typedef BasicResource<Meshing::TriMesh> TriMeshResource;
typedef BasicResource<Camera::Viewport> ViewportResource;

class ConfigsResource : public ResourceBase
{
 public:
  virtual bool Load(istream& in);
  virtual bool Save(ostream& out);
  virtual bool Load(const std::string& fn) { return ResourceBase::Load(fn); }
  virtual bool Save(const std::string& fn) { return ResourceBase::Save(fn); }
  virtual bool Load() { return ResourceBase::Load(); }
  virtual bool Save() { return ResourceBase::Save(); }
  virtual bool Save(AnyCollection& c) { c["configs"]=configs; return true; }
  virtual bool Load(AnyCollection& c) { return c["configs"].asvector(configs); }
  virtual const char* Type() const { return "Configs"; }
  virtual ResourceBase* Make() { return new ConfigsResource; }

  vector<Vector> configs;
};

class PointCloudResource : public ResourceBase
{
 public:
  virtual bool Load(istream& in);
  virtual bool Save(ostream& out);
  virtual bool Load(const std::string& fn) { return ResourceBase::Load(fn); }
  virtual bool Load() { return ResourceBase::Load(); }
  virtual bool Save(const std::string& fn) { return ResourceBase::Save(fn); }
  virtual bool Save() { return ResourceBase::Save(); }
  virtual const char* Type() const { return "PointCloud"; }
  virtual ResourceBase* Make() { return new PointCloudResource; }

  Meshing::PointCloud3D pointCloud;
};


class RobotResource : public ResourceBase
{
 public:
  virtual bool Load(const string& fn);
  virtual bool Save(const string& fn);
  virtual const char* Type() const { return "Robot"; }
  virtual ResourceBase* Make() { return new RobotResource; }

  Robot robot;
};

class RigidObjectResource : public ResourceBase
{
 public:
  virtual bool Load(const string& fn);
  virtual bool Save(const string& fn);
  virtual const char* Type() const { return "RigidObject"; }
  virtual ResourceBase* Make() { return new RigidObjectResource; }

  RigidObject object;
};

class WorldResource : public ResourceBase
{
 public:
  virtual bool Load(const string& fn);
  virtual bool Save(const string& fn);
  virtual const char* Type() const { return "World"; }
  virtual ResourceBase* Make() { return new WorldResource; }

  RobotWorld world;
};

class LinearPathResource : public ResourceBase
{
 public:
  virtual bool Load(istream& in);
  virtual bool Save(ostream& out);
  virtual bool Load(const std::string& fn) { return ResourceBase::Load(fn); }
  virtual bool Load() { return ResourceBase::Load(); }
  virtual bool Save(const std::string& fn) { return ResourceBase::Save(fn); }
  virtual bool Save() { return ResourceBase::Save(); }
  virtual bool Save(AnyCollection& c) { c["times"]=times; c["milestones"]=milestones; return true; }
  virtual bool Load(AnyCollection& c) { return c["times"].asvector(times) && c["milestones"].asvector(milestones); }
  virtual const char* Type() const { return "LinearPath"; }
  virtual ResourceBase* Make() { return new LinearPathResource; }

  vector<Real> times;
  vector<Vector> milestones;
};

class MultiPathResource : public ResourceBase
{
 public:
  virtual bool Load(const std::string& fn);
  virtual bool Save(const std::string& fn);
  virtual bool Load(TiXmlElement* in);
  virtual bool Save(TiXmlElement* out);
  virtual bool Load() { return ResourceBase::Load(); }
  virtual bool Save() { return ResourceBase::Save(); }
  virtual const char* Type() const { return "MultiPath"; }
  virtual ResourceBase* Make() { return new MultiPathResource; }

  MultiPath path;
};

class IKGoalResource : public BasicResource<IKGoal> 
{
 public:
  IKGoalResource() {}
  IKGoalResource(const IKGoal& val) : BasicResource<IKGoal>(val) {}
  IKGoalResource(const IKGoal& val,const std::string& name) : BasicResource<IKGoal>(val,name) {}
  virtual bool Load(AnyCollection& c);
  virtual bool Save(AnyCollection& c);
};

class HoldResource : public BasicResource<Hold> 
{
 public:
  HoldResource() {}
  HoldResource(const Hold& val) : BasicResource<Hold>(val) {}
  HoldResource(const Hold& val,const std::string& name) : BasicResource<Hold>(val,name) {}
  virtual bool Load(AnyCollection& c);
  virtual bool Save(AnyCollection& c);
};

class StanceResource : public ResourceBase
{
 public:
  StanceResource() {}
  StanceResource(const Stance& val);
  virtual bool Load(istream& in);
  virtual bool Save(ostream& out);
  virtual bool Load(TiXmlElement* in);
  virtual bool Save(TiXmlElement* out);
  virtual bool Load() { return ResourceBase::Load(); }
  virtual bool Save() { return ResourceBase::Save(); }
  virtual bool Load(const std::string& fn) { return ResourceBase::Load(fn); }
  virtual bool Load(AnyCollection& c);
  virtual bool Save(AnyCollection& c);
  virtual const char* Type() const { return "Stance"; }
  virtual ResourceBase* Make() { return new StanceResource; }

  Stance stance;
};

class GraspResource : public ResourceBase
{
 public:
  GraspResource() {}
  GraspResource(const Grasp& val);
  virtual bool Load(TiXmlElement* in);
  virtual bool Save(TiXmlElement* out);
  virtual bool Load() { return ResourceBase::Load(); }
  virtual bool Save() { return ResourceBase::Save(); }
  virtual bool Load(AnyCollection& c);
  virtual bool Save(AnyCollection& c);
  virtual const char* Type() const { return "Grasp"; }
  virtual ResourceBase* Make() { return new GraspResource; }

  Grasp grasp;
};

void Convert(const IKGoal& g,AnyCollection& c);
void Convert(const Hold& h,AnyCollection& c);
bool Convert(const AnyCollection& c,IKGoal& g);
bool Convert(const AnyCollection& c,Hold& h);

/** @ingroup Modeling
 * @brief Initializes a ResourceLibrary so that it accepts standard RobotSim
 * file types.
 */
void MakeRobotResourceLibrary(ResourceLibrary& library);

ResourcePtr MakeResource(const string& name,const vector<int>& vals);
ResourcePtr MakeResource(const string& name,const vector<double>& vals);
ResourcePtr MakeResource(const string& name,const Config& q);
ResourcePtr MakeResource(const string& name,const vector<Config>& qs);
ResourcePtr MakeResource(const string& name,const vector<Real>& ts,const vector<Config>& qs);
ResourcePtr MakeResource(const string& name,const MultiPath& path);
ResourcePtr MakeResource(const string& name,const Vector3& pt);
ResourcePtr MakeResource(const string& name,const Matrix3& R);
ResourcePtr MakeResource(const string& name,const RigidTransform& T);
ResourcePtr MakeResource(const string& name,const GeometricPrimitive3D& geom);
ResourcePtr MakeResource(const string& name,const Meshing::TriMesh& mesh);
ResourcePtr MakeResource(const string& name,const IKGoal& goal);
ResourcePtr MakeResource(const string& name,const Hold& hold);
ResourcePtr MakeResource(const string& name,const Stance& stance);
ResourcePtr MakeResource(const string& name,const Grasp& grasp);


ResourcePtr CastResource(const ResourcePtr& item,const char* type);
vector<ResourcePtr> ExtractResources(const ResourcePtr& item,const char* type);

template <class T>
ResourcePtr CastResource(const ResourcePtr& item)
{
  T temp;
  return CastResource(item,temp.Type());
}

template <class T>
vector<ResourcePtr> ExtractResources(const ResourcePtr& item)
{
  T temp;
  return ExtractResources(item,temp.Type());
}



#endif
