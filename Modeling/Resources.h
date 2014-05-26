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

/** @brief Resource for multiple Config's.
 */
class ConfigsResource : public CompoundResourceBase
{
 public:
  virtual bool Load(istream& in);
  virtual bool Save(ostream& out);
  virtual bool Save(AnyCollection& c) { c["configs"]=configs; return true; }
  virtual bool Load(AnyCollection& c) { return c["configs"].asvector(configs); }
  virtual const char* Type() const { return "Configs"; }
  virtual ResourceBase* Make() { return new ConfigsResource; }
  virtual ResourceBase* Copy();
  virtual vector<string> CastTypes() const;
  virtual vector<string> SubTypes() const;
  virtual ResourcePtr Cast(const char* subtype);
  virtual bool Extract(const char* subtype,vector<ResourcePtr>&);
  virtual bool Pack(vector<ResourcePtr>& subobjects,string* errorMessage=NULL);
  virtual bool Unpack(vector<ResourcePtr>& subobjects,bool* incomplete=NULL);

  vector<Vector> configs;
};

/** @brief Resource for a PointCloud3D.
 */
class PointCloudResource : public ResourceBase
{
 public:
  virtual bool Load(istream& in);
  virtual bool Save(ostream& out);
  virtual const char* Type() const { return "PointCloud"; }
  virtual ResourceBase* Make() { return new PointCloudResource; }
  virtual ResourceBase* Copy();

  Meshing::PointCloud3D pointCloud;
};

/** @brief Resource for a Robot.
 */
class RobotResource : public ResourceBase
{
 public:
  virtual bool Load(const string& fn);
  virtual bool Save(const string& fn);
  virtual const char* Type() const { return "Robot"; }
  virtual ResourceBase* Make() { return new RobotResource; }
  virtual ResourceBase* Copy();

  Robot robot;
};

/** @brief Resource for a RigidObject.
 */
class RigidObjectResource : public ResourceBase
{
 public:
  virtual bool Load(const string& fn);
  virtual bool Save(const string& fn);
  virtual const char* Type() const { return "RigidObject"; }
  virtual ResourceBase* Make() { return new RigidObjectResource; }
  virtual ResourceBase* Copy();

  RigidObject object;
};

/** @brief Resource for a RobotWorld.
 *
 * Implementation status: Incomplete. (Saving is not done yet, visualization
 * info are not implemented in hierarchy)
 */
class WorldResource : public CompoundResourceBase
{
 public:
  virtual bool Load(const string& fn);
  virtual bool Save(const string& fn);
  virtual const char* Type() const { return "World"; }
  virtual ResourceBase* Make() { return new WorldResource; }
  virtual ResourceBase* Copy();
  virtual vector<string> SubTypes() const;
  virtual bool Extract(const char* subtype,vector<ResourcePtr>&);
  virtual bool Pack(vector<ResourcePtr>& subobjects,string* errorMessage=NULL);
  virtual bool Unpack(vector<ResourcePtr>& subobjects,bool* incomplete=NULL);

  RobotWorld world;
};

/** @brief Resource for a LinearPath. */
class LinearPathResource : public CompoundResourceBase
{
 public:
  virtual bool Load(istream& in);
  virtual bool Save(ostream& out);
  virtual bool Save(AnyCollection& c) { c["times"]=times; c["milestones"]=milestones; return true; }
  virtual bool Load(AnyCollection& c) { return c["times"].asvector(times) && c["milestones"].asvector(milestones); }
  virtual const char* Type() const { return "LinearPath"; }
  virtual ResourceBase* Make() { return new LinearPathResource; }
  virtual ResourceBase* Copy();
  virtual vector<string> CastTypes() const;
  virtual vector<string> SubTypes() const;
  virtual vector<string> ExtractTypes() const;
  virtual ResourcePtr Cast(const char* subtype);
  virtual bool Extract(const char* subtype,vector<ResourcePtr>&);
  virtual bool Pack(vector<ResourcePtr>& subobjects,string* errorMessage=NULL);
  virtual bool Unpack(vector<ResourcePtr>& subobjects,bool* incomplete=NULL);

  vector<Real> times;
  vector<Vector> milestones;
};

/** @brief Resource for a MultiPath.
 * 
 * Implementation status: mostly complete. Hierarchy for properties and
 * global holds are not complete.
 */
class MultiPathResource : public CompoundResourceBase
{
 public:
  virtual bool Load(const string& fn);
  virtual bool Save(const string& fn);
  virtual bool Load(TiXmlElement* in);
  virtual bool Save(TiXmlElement* out);
  virtual bool Load() { return ResourceBase::Load(); }
  virtual bool Save() { return ResourceBase::Save(); }
  virtual const char* Type() const { return "MultiPath"; }
  virtual ResourceBase* Make() { return new MultiPathResource; }
  virtual ResourceBase* Copy();
  virtual vector<string> CastTypes() const;
  virtual vector<string> SubTypes() const;
  virtual vector<string> ExtractTypes() const;
  virtual ResourcePtr Cast(const char* subtype);
  virtual bool Extract(const char* subtype,vector<ResourcePtr>&);
  virtual bool Pack(vector<ResourcePtr>& subobjects,string* errorMessage=NULL);
  virtual bool Unpack(vector<ResourcePtr>& subobjects,bool* incomplete=NULL);

  MultiPath path;
};

/** @brief Resource for an IKGoal.
 * 
 * Implementation status: hierarchy is incomplete.
 */
class IKGoalResource : public CompoundResourceBase
{
 public:
  IKGoalResource() {}
  IKGoalResource(const IKGoal& val) : goal(val) {}
  IKGoalResource(const IKGoal& val,const string& name) : CompoundResourceBase(name),goal(val) {}
  virtual bool Load(AnyCollection& c);
  virtual bool Save(AnyCollection& c);
  virtual bool Load(std::istream& in) {
    in>>goal;
    if(in.bad()) return false;
    return true;
  }
  virtual bool Save(std::ostream& out) {
    out<<goal<<std::endl;
    return true;
  }
  virtual const char* Type() const { return "IKGoal"; }
  virtual ResourceBase* Make() { return new IKGoalResource; }
  virtual ResourceBase* Copy() { return new IKGoalResource(goal,name); }
  virtual vector<string> CastTypes() const;
  virtual ResourcePtr Cast(const char* subtype) const;

  IKGoal goal;
};

/** @brief Resource for a Hold.
 */
class HoldResource : public CompoundResourceBase
{
 public:
  HoldResource() {}
  HoldResource(const Hold& val) : hold(val) {}
  HoldResource(const Hold& val,const string& name) : CompoundResourceBase(name),hold(val) {}
  virtual bool Load(AnyCollection& c);
  virtual bool Save(AnyCollection& c);
  virtual bool Load(std::istream& in) {
    in>>hold;
    if(in.bad()) return false;
    return true;
  }
  virtual bool Save(std::ostream& out) {
    out<<hold<<std::endl;
    return true;
  }
  virtual const char* Type() const { return "Hold"; }
  virtual ResourceBase* Make() { return new HoldResource; }
  virtual ResourceBase* Copy() { return new HoldResource(hold,name); }
  virtual vector<string> CastTypes() const;
  virtual vector<string> SubTypes() const;
  virtual ResourcePtr Cast(const char* subtype) const;
  virtual bool Pack(vector<ResourcePtr>& subobjects,string* errorMessage=NULL);
  virtual bool Unpack(vector<ResourcePtr>& subobjects,bool* incomplete=NULL);

  Hold hold;
};

/** @brief Resource for a Stance.
 */
class StanceResource : public CompoundResourceBase
{
 public:
  StanceResource() {}
  StanceResource(const Stance& val);
  virtual bool Load(istream& in);
  virtual bool Save(ostream& out);
  virtual bool Load(TiXmlElement* in);
  virtual bool Save(TiXmlElement* out);
  virtual bool Load(AnyCollection& c);
  virtual bool Save(AnyCollection& c);
  virtual const char* Type() const { return "Stance"; }
  virtual ResourceBase* Make() { return new StanceResource; }
  virtual ResourceBase* Copy();
  virtual vector<string> CastTypes() const;
  virtual vector<string> SubTypes() const;
  virtual vector<string> ExtractTypes() const;
  virtual ResourcePtr Cast(const char* subtype);
  virtual bool Extract(const char* subtype,vector<ResourcePtr>&);
  virtual bool Pack(vector<ResourcePtr>& subobjects,string* errorMessage=NULL);
  virtual bool Unpack(vector<ResourcePtr>& subobjects,bool* incomplete=NULL);

  Stance stance;
};

/** @brief Resource for a Grasp.
 * 
 * Implementation status: incomplete.  Hierarchy items for fixed DOFs
 * are incomplete.
 */
class GraspResource : public CompoundResourceBase
{
 public:
  GraspResource() {}
  GraspResource(const Grasp& val);
  virtual bool Load(TiXmlElement* in);
  virtual bool Save(TiXmlElement* out);
  virtual bool Load(AnyCollection& c);
  virtual bool Save(AnyCollection& c);
  virtual const char* Type() const { return "Grasp"; }
  virtual ResourceBase* Make() { return new GraspResource; }
  virtual ResourceBase* Copy();
  virtual vector<string> CastTypes() const;
  virtual vector<string> SubTypes() const;
  virtual vector<string> ExtractTypes() const;
  virtual ResourcePtr Cast(const char* subtype);
  virtual bool Extract(const char* subtype,vector<ResourcePtr>&);
  virtual bool Pack(vector<ResourcePtr>& subobjects,string* errorMessage=NULL);
  virtual bool Unpack(vector<ResourcePtr>& subobjects,bool* incomplete=NULL);

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
ResourcePtr MakeResource(const string& name,const Geometry::AnyGeometry3D& geom);
ResourcePtr MakeResource(const string& name,const IKGoal& goal);
ResourcePtr MakeResource(const string& name,const Hold& hold);
ResourcePtr MakeResource(const string& name,const Stance& stance);
ResourcePtr MakeResource(const string& name,const Grasp& grasp);

///Returns true if CastResource can cast to the given type
bool CanCastResource(const ResourcePtr& item,const char* type);
///Returns the list of types which the item is castable to
vector<string> CastResourceTypes(const ResourcePtr& item);
///Convert a resource to a given type
ResourcePtr CastResource(ResourcePtr& item,const char* type);

///Returns the list of types that can be extracted from the item
vector<string> ExtractResourceTypes(const ResourcePtr& item);
///Extract all sub-resources of a given type
vector<ResourcePtr> ExtractResources(ResourcePtr& item,const char* type);

///Creates an object of the same type as the template out of the given
///resources
ResourcePtr PackResources(vector<ResourcePtr>& resources,ResourcePtr rtemplate,string* errorMessage=NULL);
///Creates an object of the given type out of the given resources
ResourcePtr PackResources(ResourceLibrary& resources,const string& type,string* errorMessage=NULL);

///Creates sub-objects from the given resource.
///If the decomposition succeeded, the flag successful is set to true.
///If the decomposition is incomplete, the flag incomplete is set to true.
///Here "incomplete" means that calling Pack on the return array will
///NOT produce a copy of r.
vector<ResourcePtr> UnpackResource(ResourcePtr r,bool* successful=NULL,bool* incomplete=NULL);

#endif
