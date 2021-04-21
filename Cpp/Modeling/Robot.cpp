#include "Robot.h"
#include "Mass.h"
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/arrayutils.h>
#include <string.h>
#include <KrisLibrary/robotics/DenavitHartenberg.h>
#include <KrisLibrary/robotics/Rotation.h>
#include <KrisLibrary/math3d/misc.h>
#include <KrisLibrary/math3d/basis.h>
#include <KrisLibrary/meshing/IO.h>
#include <KrisLibrary/meshing/VolumeGrid.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/geometry/Conversions.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/utils/fileutils.h>
#include <fstream>
#include <sstream>
#include <KrisLibrary/Timer.h>
#include "IO/urdf_parser.h"
#include <memory>
#include "IO/URDFConverter.h"
#include <map>
//using namespace urdf;

DEFINE_LOGGER(Robot)
DEFINE_LOGGER(RobParser)
DEFINE_LOGGER(URDFParser)

//defined in XmlWorld.cpp
string ResolveFileReference(const string& path,const string& fn);
string MakeURLLocal(const string& url,const char* url_resolution_path="klampt_downloads");

template <class Val>
class Voting
{
public:
  Voting() {}
  void add(const Val& v,int numVotes=1) {
    if(counts.count(v) == 0)
      counts[v] = numVotes;
    else
      counts[v] += numVotes;
  }
  void erase(const Val& v) {
    if(counts.count(v) != 0)
      counts.erase(counts.find(v));
  }
  size_t numVotes() const {
    size_t cnt = 0;
    for(auto i=counts.begin();i!=counts.end();i++)
      cnt += i->second;
    return cnt;
  }
  ///returns the winning value. This will return an empty Val if there are 0 votes.  If there are ties, this
  ///returns the one that has least value.
  Val winner() const {
    size_t imax = 0;
    Val vmax;
    for(auto i=counts.begin();i!=counts.end();i++) {
      if(i->second > imax) {
        imax = i->second;
        vmax = i->first;
      }
    }
    return vmax;
  }
  map<Val,size_t> counts;
};

Real Radius(const Geometry::AnyGeometry3D& geom)
{
  switch(geom.type) {
  case Geometry::AnyGeometry3D::Primitive:
    {
      if(geom.Empty()) return 0.0;
      Box3D box = geom.AsPrimitive().GetBB();
      Vector3 originLocal;
      box.toLocal(Vector3(0.0),originLocal);
      Real xmax = Max(Abs(originLocal.x),Abs(originLocal.x+box.dims.x));
      Real ymax = Max(Abs(originLocal.y),Abs(originLocal.y+box.dims.y));
      Real zmax = Max(Abs(originLocal.z),Abs(originLocal.z+box.dims.z));
      return Sqrt(Sqr(xmax)+Sqr(ymax)+Sqr(zmax));
    }
  case Geometry::AnyGeometry3D::TriangleMesh:
    {
      const Meshing::TriMesh& mesh = geom.AsTriangleMesh();
      Real dmax = 0;
      for(size_t i=0;i<mesh.verts.size();i++)
        dmax = Max(dmax,mesh.verts[i].normSquared());
      return Sqrt(dmax);
    }
  case Geometry::AnyGeometry3D::PointCloud:
    {
      const Meshing::PointCloud3D& mesh = geom.AsPointCloud();
      Real dmax = 0;
      for(size_t i=0;i<mesh.points.size();i++)
        dmax = Max(dmax,mesh.points[i].normSquared());
      return Sqrt(dmax);
    }
  case Geometry::AnyGeometry3D::ImplicitSurface:
    {
      Box3D box;
      box.set(geom.AsImplicitSurface().bb);
      Vector3 originLocal;
      box.toLocal(Vector3(0.0),originLocal);
      Real xmax = Max(Abs(originLocal.x),Abs(originLocal.x+box.dims.x));
      Real ymax = Max(Abs(originLocal.y),Abs(originLocal.y+box.dims.y));
      Real zmax = Max(Abs(originLocal.z),Abs(originLocal.z+box.dims.z));
      return Sqrt(Sqr(xmax)+Sqr(ymax)+Sqr(zmax));
    }
  case Geometry::AnyGeometry3D::Group:
    {
      const vector<Geometry::AnyGeometry3D>& items = geom.AsGroup();
      Real rmax = 0;
      for(size_t i=0;i<items.size();i++)
        rmax = Max(rmax,Radius(items[i]));
      return rmax;
    }
  case Geometry::AnyGeometry3D::ConvexHull:
    {
      Meshing::TriMesh mesh;
      Geometry::ConvexHullToMesh(geom.AsConvexHull(),mesh);
      Real dmax = 0;
      for(size_t i=0;i<mesh.verts.size();i++)
        dmax = Max(dmax,mesh.verts[i].normSquared());
      return Sqrt(dmax);
    }
  }
  return 0;
}

void GetAccMax(const Robot& robot, Vector& accMax) {
  accMax = robot.torqueMax;
  Real sumMass = 0;
  Real sumCom = 0;
  for (int i = accMax.n - 1; i >= 0; i--) {
    Real oldMass = sumMass;
    Real oldCom = sumCom;
    sumMass += robot.links[i].mass;
    oldCom += robot.links[i].T0_Parent.t.length();
    sumCom = (robot.links[i].com.length() * robot.links[i].mass
        + oldCom * oldMass) / (robot.links[i].mass + oldMass);
    accMax(i) = robot.torqueMax[i] / (sumCom * sumMass * 9.8);
    if (!IsFinite(accMax(i))) {
      LOG4CXX_WARN(GET_LOGGER(Robot),"Warning, infinite acceleration limit for joint "<< i);
    }
  }
}

void SetDefaultAppearance(shared_ptr<GLDraw::GeometryAppearance> app)
{
  app->faceColor.set(0.5,0.5,0.5);
  app->specularColor.set(0.2,0.2,0.2);
  app->shininess = 20;
}


int RobotJointDriver::NumControls() const {
  return 1;
}
int RobotJointDriver::NumLinks() const {
  return linkIndices.size();
}
bool RobotJointDriver::Affects(int link) const {
  for (size_t i = 0; i < linkIndices.size(); i++)
    if (linkIndices[i] == link)
      return true;
  return false;
}

bool Robot::disableGeometryLoading = false;

std::string Robot::LinkName(int i) const {
  if (linkNames.empty())
    return RobotWithGeometry::LinkName(i);
  else if (linkNames[i].empty())
    return RobotWithGeometry::LinkName(i);
  else
    return linkNames[i];
}

int Robot::LinkIndex(const char* name) const
{
  if(IsValidInteger(name)) {
    stringstream ss(name);
    int res;
    ss>>res;
    return res;
  }
  for(size_t i=0;i<linkNames.size();i++)
    if(linkNames[i] == name) return (int)i;
  return -1;
}

bool Robot::Load(const char* fn) {
  bool res = false;
  const char* ext = FileExtension(fn);
  if(ext == NULL) {
    LOG4CXX_ERROR(GET_LOGGER(Robot),"Robot::Load("<<fn <<"): no extension, file must have .rob or .urdf extension");
  }
  else if (0 == strcmp(ext, "rob")) {
    res = LoadRob(fn);
  }
  else if (0 == strcmp(ext, "urdf")) {
    res = LoadURDF(fn);
  }
  else {
    LOG4CXX_ERROR(GET_LOGGER(Robot),"Robot::Load("<<fn<<"): unknown extenion "<<ext <<", only .rob or .urdf supported");
  }

  return res;
}

bool Robot::LoadRob(const char* fn) {
  string path = GetFilePath(fn);
  string localfile = MakeURLLocal(fn);
  if(localfile.empty()) return false;
  links.resize(0);
  parents.resize(0);
  qMin.clear();
  qMax.clear();
  q.clear();
  linkNames.resize(0);
  driverNames.resize(0);
  joints.resize(0);
  drivers.resize(0);
  lipschitzMatrix.clear();
  properties.clear();
  vector<Real> massVec;
  vector<Vector3> comVec;
  vector<Matrix3> inertiaVec;
  vector<Real> qMinVec, qMaxVec, qVec;
  vector<Real> vMinVec, vMaxVec, aMaxVec, tMaxVec, pMaxVec;
  vector<Real> servoP, servoI, servoD, dryFriction, viscousFriction;
  vector<Real> alpha, a, d, theta;
  vector<RigidTransform> TParent;
  vector<Vector3> axes;
  vector<int> jointType;
  vector<string> collision, noCollision;
  vector<pair<string, string> > selfCollision;
  vector<pair<string, string> > noSelfCollision;
  RigidTransform baseTransform;
  baseTransform.setIdentity();
  Real scale = 1.0;
  vector<string> geomFn;
  vector<Real> geomscale;
  vector<Real> geommargin;
  bool autoMass = false;
  Real surfaceFraction = 1.0;
  Real autoTorque = 0;
  vector<string> mountLinks;
  vector<string> mountFiles;
  vector<RigidTransform> mountT;
  vector<string> mountNames;

  //add a new keyword: geomTransform for transform 3D geometry
  vector<int> geomTransformIndex;
  vector<Matrix4> geomTransform;

  ifstream in(localfile.c_str(), ios::in);
  if (!in) {
    LOG4CXX_INFO(GET_LOGGER(RobParser),"Unable to read robot file "<< fn<<", file does not exist or is not available for reading");
    return false;
  }
  LOG4CXX_INFO(GET_LOGGER(RobParser),"Reading robot file "<< fn <<"...");
  int lineno = 0;
  while (in) {
    //LOG4CXX_INFO(GET_LOGGER(RobParser),"Reading line "<<name<<"...");
    //read the rest of the line
    string line,name;
    char buf[1025];
    buf[1024] = 0;
    bool foundEndline = false, comment = false;
    bool foundEof = false;
    while (!foundEndline) {
      for (int i = 0; i < 1024; i++) {
        int c = in.get();
        if(!in || c == EOF) { buf[i] = 0; foundEndline=true; foundEof=true; break; }
        if (c == '\n') {
          buf[i] = 0;
          foundEndline = true;
          lineno++;
          break;
        } else if (c == '#' || comment) {
          c = 0;
          comment = true;
        } else if (c == '\\') { //read escape characters (e.g., end of line)
          c = in.get();
          if (c == '\r') //possibly windows-encoded file
            c = in.get();
          else if (c == '\n')
            lineno++;
          else {
            buf[i] = '\\';
            i++;
          }
          //Change: don't escape in file, since quoted strings might contain escaped characters
          //c = TranslateEscape(c);
        }
        buf[i] = c;
      }
      line += buf;
    }
    stringstream ss;
    ss.str(line);
    ss>>name;
    if(!ss) {
      if(foundEof) break;
      continue; //empty line
    }
    Lowercase(name);
    //LOG4CXX_INFO(GET_LOGGER(RobParser),"Reading line "<<name<<" line "<<lineno);
    string stemp;
    int itemp;
    Real ftemp;
    if (name[0] == '#') {
      continue;
    } else if (name == "parents") {
      while (ss >> itemp)
        parents.push_back(itemp);
    } else if (name == "links") {
      while (SafeInputString(ss, stemp))
        linkNames.push_back(stemp);
    } else if (name == "jointtype") {
      while (ss >> stemp) {
        if (stemp == "r")
          jointType.push_back(RobotLink3D::Revolute);
        else if (stemp == "p")
          jointType.push_back(RobotLink3D::Prismatic);
        else {
          LOG4CXX_ERROR(GET_LOGGER(RobParser),"Invalid joint type "<<stemp<<" on line "<<lineno);
          return false;
        }
      }
    } else if (name == "drivers") {
      while (SafeInputString(ss, stemp))
        driverNames.push_back(stemp);
    } else if (name == "qmin") {
      while (SafeInputFloat(ss, ftemp))
        qMinVec.push_back(ftemp);
    } else if (name == "qmindeg") {
      while (SafeInputFloat(ss, ftemp))
        qMinVec.push_back(DtoR(ftemp));
    } else if (name == "qmax") {
      while (SafeInputFloat(ss, ftemp))
        qMaxVec.push_back(ftemp);
    } else if (name == "qmaxdeg") {
      while (SafeInputFloat(ss, ftemp))
        qMaxVec.push_back(DtoR(ftemp));
    } else if (name == "q") {
      while (SafeInputFloat(ss, ftemp))
        qVec.push_back(ftemp);
    } else if (name == "qdeg") {
      while (SafeInputFloat(ss, ftemp))
        qVec.push_back(DtoR(ftemp));
    } else if (name == "velmax") {
      while (SafeInputFloat(ss, ftemp))
        vMaxVec.push_back(ftemp);
    } else if (name == "velmaxdeg") {
      while (SafeInputFloat(ss, ftemp))
        vMaxVec.push_back(DtoR(ftemp));
    } else if (name == "velmin") {
      while (SafeInputFloat(ss, ftemp))
        vMinVec.push_back(ftemp);
    } else if (name == "velmindeg") {
      while (SafeInputFloat(ss, ftemp))
        vMinVec.push_back(DtoR(ftemp));
    } else if (name == "torquemax") {
      while (SafeInputFloat(ss, ftemp))
        tMaxVec.push_back(ftemp);
    } else if (name == "accmax") {
      while (SafeInputFloat(ss, ftemp))
        aMaxVec.push_back(ftemp);
    } else if (name == "accmaxdeg") {
      while (SafeInputFloat(ss, ftemp))
        aMaxVec.push_back(DtoR(ftemp));
    } else if (name == "powermax") {
      while (SafeInputFloat(ss, ftemp))
        pMaxVec.push_back(ftemp);
    } else if (name == "servop") {
      while (ss >> ftemp)
        servoP.push_back(ftemp);
    } else if (name == "servoi") {
      while (ss >> ftemp)
        servoI.push_back(ftemp);
    } else if (name == "servod") {
      while (ss >> ftemp)
        servoD.push_back(ftemp);
    } else if (name == "dryfriction") {
      while (ss >> ftemp)
        dryFriction.push_back(ftemp);
    } else if (name == "viscousfriction") {
      while (ss >> ftemp)
        viscousFriction.push_back(ftemp);
    } else if (name == "tparent") {
      RigidTransform Ttemp;
      while (ss >> Ttemp)
        TParent.push_back(Ttemp);
    } else if (name == "axis") {
      Vector3 vtemp;
      while (ss >> vtemp)
        axes.push_back(vtemp);
    } else if (name == "alpha") {
      while (ss >> ftemp)
        alpha.push_back(ftemp);
    } else if (name == "alphadeg") {
      while (ss >> ftemp)
        alpha.push_back(DtoR(ftemp));
    } else if (name == "a") {
      while (ss >> ftemp)
        a.push_back(ftemp);
    } else if (name == "d") {
      while (ss >> ftemp)
        d.push_back(ftemp);
    } else if (name == "theta") {
      while (ss >> ftemp)
        theta.push_back(ftemp);
    } else if (name == "thetadeg") {
      while (ss >> ftemp)
        theta.push_back(DtoR(ftemp));
    } else if (name == "translation") {
      ss >> baseTransform.t;
    } else if (name == "rotation") {
      ss >> baseTransform.R;
    } else if (name == "mass") {
      while (ss >> ftemp)
        massVec.push_back(ftemp);
    } else if (name == "com") {
      Vector3 v3temp;
      while (ss >> v3temp)
        comVec.push_back(v3temp);
    } else if (name == "inertiadiag") {
      Vector3 v3temp;
      Matrix3 m3temp;
      while (ss >> v3temp) {
        m3temp.setDiagonal(v3temp);
        inertiaVec.push_back(m3temp);
      }
    } else if (name == "inertia") {
      Vector3 v1, v2, v3;
      Matrix3 m3temp;
      while (ss) {
        ss >> v1 >> v2 >> v3;
        if (ss) {
          m3temp.set(v1, v2, v3);
          inertiaVec.push_back(m3temp);
        }
      }
    } else if (name == "automass") {
      autoMass = true;
      Real temp;
      ss >> temp;
      if(ss) surfaceFraction = temp;
    } else if (name == "autotorque") {
      ss >> autoTorque;
    } else if (name == "geometry") {
      while (SafeInputString(ss, stemp)) {
        geomFn.push_back(stemp);
      }
    } else if (name == "scale") {
      ss >> scale;
    } else if (name == "geomscale") {
      while (ss >> ftemp)
        geomscale.push_back(ftemp);
    } else if (name == "geommargin") {
      while (ss >> ftemp)
        geommargin.push_back(ftemp);
    } else if (name == "collision") {
      while (SafeInputString(ss,stemp))
        collision.push_back(stemp);
    } else if (name == "nocollision") {
      while (SafeInputString(ss,stemp))
        noCollision.push_back(stemp);
    } else if (name == "selfcollision") {
      pair<string, string> ptemp;
      while (SafeInputString(ss,ptemp.first) && SafeInputString(ss,ptemp.second))
        selfCollision.push_back(ptemp);
    } else if (name == "noselfcollision") {
      pair<string, string> ptemp;
      while (SafeInputString(ss,ptemp.first) && SafeInputString(ss,ptemp.second))
        noSelfCollision.push_back(ptemp);
    } else if (name == "geomtransform") {
      int tmpindex;
      ss >> tmpindex;
      geomTransformIndex.push_back(tmpindex);
      Matrix4 m;
      if(ss){
        ss >> m(0,0) >> m(0,1) >> m(0,2) >> m(0,3);
        ss >> m(1,0) >> m(1,1) >> m(1,2) >> m(1,3);
        ss >> m(2,0) >> m(2,1) >> m(2,2) >> m(2,3);
        ss >> m(3,0) >> m(3,1) >> m(3,2) >> m(3,3);
        geomTransform.push_back(m);
      }else{
        LOG4CXX_ERROR(GET_LOGGER(RobParser),"Invalid geomTransform on line "<<lineno);
      }
    }else if (name == "joint") {
      RobotJoint tempJoint;
      tempJoint.type = RobotJoint::Normal;
      ss >> stemp;
      if (ss) {
        Lowercase(stemp);
        if (stemp == "weld") {
          tempJoint.type = RobotJoint::Weld;
          ss >> tempJoint.linkIndex;
        } else if (stemp == "normal") {
          tempJoint.type = RobotJoint::Normal;
          ss >> tempJoint.linkIndex;
        } else if (stemp == "spin") {
          tempJoint.type = RobotJoint::Spin;
          ss >> tempJoint.linkIndex;
        } else if (stemp == "floating") {
          tempJoint.type = RobotJoint::Floating;
          ss >> tempJoint.linkIndex;
          if (tempJoint.linkIndex <= 0) {
            LOG4CXX_INFO(GET_LOGGER(RobParser),"Invalid floating link "<< tempJoint.linkIndex<<" on line "<< lineno);
            return false;
          }
          if (ss) {
            ss >> tempJoint.baseIndex;
            if (!ss) {
              tempJoint.baseIndex = -1;
              ss.clear(); //clear the bad bit
            }
          }
        } else if (stemp == "ballandsocket") {
          tempJoint.type = RobotJoint::BallAndSocket;
          ss >> tempJoint.linkIndex;
          if (tempJoint.linkIndex <= 0) {
            LOG4CXX_INFO(GET_LOGGER(RobParser),"Invalid ballandsocket link "<<tempJoint.linkIndex <<" on line "<<lineno);
            return false;
          }
          if (ss) {
            ss >> tempJoint.baseIndex;
            if (!ss) {
              tempJoint.baseIndex = -1;
              ss.clear(); //clear the bad bit
            }
          }
        } else if (stemp == "floatingplanar") {
          tempJoint.type = RobotJoint::FloatingPlanar;
          ss >> tempJoint.linkIndex;
          if (tempJoint.linkIndex <= 0) {
            LOG4CXX_INFO(GET_LOGGER(RobParser),"Invalid floatingplanar link "<<tempJoint.linkIndex <<" on line "<<lineno);
            return false;
          }
          if (ss) {
            ss >> tempJoint.baseIndex;
            if (!ss) {
              tempJoint.baseIndex = -1;
              ss.clear(); //clear the bad bit
            }
          }
        } else {
          LOG4CXX_INFO(GET_LOGGER(RobParser),"Invalid joint type "<< stemp.c_str()<<" on line "<<
              lineno);
          return false;
        }
      }
      if (ss)
        joints.push_back(tempJoint);
    } else if (name == "driver") {
      RobotJointDriver tempDriver;
      tempDriver.type = RobotJointDriver::Normal;
      ss >> stemp;
      if (ss) {
        //TODO: servo parameters, dry friction
        tempDriver.servoP = 0;
        tempDriver.servoI = 0;
        tempDriver.servoD = 0;
        tempDriver.dryFriction = 0;
        tempDriver.viscousFriction = 0;
        if (stemp == "normal") {
          tempDriver.type = RobotJointDriver::Normal;
          tempDriver.linkIndices.resize(1);
          ss >> itemp;
          tempDriver.linkIndices[0] = itemp;
          //Settings are loaded below
        } else if (stemp == "affine") {
          tempDriver.type = RobotJointDriver::Affine;
          ss >> itemp;
          if (itemp <= 0) {
            LOG4CXX_ERROR(GET_LOGGER(RobParser),"Invalid number of joint indices "<<itemp);
            return false;
          }
          tempDriver.linkIndices.resize(itemp);
          tempDriver.affScaling.resize(itemp);
          tempDriver.affOffset.resize(itemp);
          for (int i = 0; i < itemp; i++)
            ss >> tempDriver.linkIndices[i];
          for (int i = 0; i < itemp; i++)
            ss >> tempDriver.affScaling[i];
          for (int i = 0; i < itemp; i++)
            ss >> tempDriver.affOffset[i];
          SafeInputFloat(ss, tempDriver.qmin);
          SafeInputFloat(ss, tempDriver.qmax);
          SafeInputFloat(ss, tempDriver.vmin);
          SafeInputFloat(ss, tempDriver.vmax);
          SafeInputFloat(ss, tempDriver.tmin);
          SafeInputFloat(ss, tempDriver.tmax);
          //TODO: driver acceleration limits?
          tempDriver.amin = -Inf;
          tempDriver.amax = Inf;
        } else if (stemp == "translation") {
          tempDriver.type = RobotJointDriver::Translation;
          tempDriver.linkIndices.resize(2);
          ss >> itemp;
          tempDriver.linkIndices[0] = itemp;
          ss >> itemp;
          tempDriver.linkIndices[1] = itemp;
          //Settings are loaded below
        } else if (stemp == "rotation") {
          tempDriver.type = RobotJointDriver::Rotation;
          tempDriver.linkIndices.resize(2);
          ss >> itemp;
          tempDriver.linkIndices[0] = itemp;
          ss >> itemp;
          tempDriver.linkIndices[1] = itemp;
          //Settings are loaded below
        } else {
          LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Invalid driver type "<< stemp.c_str()<<" on line "<<lineno);
          return false;
        }
      }
      if (ss)
        drivers.push_back(tempDriver);
      else {
        LOG4CXX_ERROR(GET_LOGGER(RobParser),"   Failure reading driver on line " << lineno);
        return false;
      }
    } else if (name == "mount") {
      //ss >> itemp;
      if (!SafeInputString(ss, stemp)) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser),"   Error reading mount link on line "<<lineno);
        return false;
      }
      mountLinks.push_back(stemp);
      if (!SafeInputString(ss, stemp)) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser),"   Error reading mount file name on line "<<lineno);
        return false;
      }
      mountFiles.push_back(stemp);
      RigidTransform Ttemp;
      Ttemp.setIdentity();
      if (ss) {
        ss >> Ttemp;
        if (!ss) {
          LOG4CXX_INFO(GET_LOGGER(RobParser),"   Note: didn't read subchain transform");
          ss.clear();
          Ttemp.setIdentity();
        }
      }
      mountT.push_back(Ttemp);
      string name;
      if (ss) {
        string op;
        ss >> op;
        if(op == "as") {
          if(!SafeInputString(ss,name))  {
            LOG4CXX_ERROR(GET_LOGGER(RobParser),"   Error reading mount alias");
            name = "";
          }
        }
      }
      if(name.empty()) {
        mountNames.push_back(string());
        /*
        string robotName = stemp;
        StripExtension(robotName);
        mountNames.push_back(robotName);
        */
      }
      else mountNames.push_back(name);
    }
    else if(name == "property") {
      string value;
      SafeInputString(ss,stemp);
      getline(ss, value);
      if(ss.fail() || ss.bad()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Explicit property on line "<< lineno<<" could not be read");
        return false;
      }
      if(stemp == "controller" || stemp == "sensors") {
        //need to escape these strings
        stringstream ss(value);
        string file;
        SafeInputString(ss,file);
        const char* ext = FileExtension(file.c_str());
        if(ext && 0==strcmp(ext,"xml")) {
          //prepend the robot path
          string fn = ResolveFileReference(path,file);
          fn = MakeURLLocal(fn);
          if(fn.empty()) {
            LOG4CXX_ERROR(GET_LOGGER(RobParser),"     error downloading "<<stemp<<" file");
          }
          if(!GetFileContents(fn.c_str(),properties[stemp])) {
            LOG4CXX_ERROR(GET_LOGGER(RobParser),"     Unable to read "<<stemp<<" property from file "<<fn);
            return false;
          }
        }
        else {
          //this property may have escapes (e.g. /n, /") which should be translated
          properties[stemp] = TranslateEscapes(value);
          stringstream ss(value);
          TiXmlElement e(stemp.c_str());
          ss >> e;
          if(!ss) 
            LOG4CXX_ERROR(GET_LOGGER(RobParser),"     Property "<<stemp<<" is not valid XML");
        }
      }
      else 
        properties[stemp] = value; 
    } else {
      LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Invalid robot property "<<name<<" on line "<<lineno<< "");
      return false;
    }
    if (ss.bad()) {
            LOG4CXX_ERROR(GET_LOGGER(RobParser),
          "   Error encountered while reading robot property "<<name.c_str()<<" on line "<<lineno);
      return false;
    }
  }

  size_t n = 0, nj = 0, nd = 0;
  nd = std::max(driverNames.size(), nd);
  nd = std::max(drivers.size(), nd);
  nj = std::max(joints.size(), nj);
  Voting<size_t> nvote;
  nvote.add(parents.size());
  nvote.add(jointType.size());
  nvote.add(linkNames.size());
  nvote.add(massVec.size());
  nvote.add(comVec.size());
  nvote.add(inertiaVec.size());
  nvote.add(a.size());
  nvote.add(d.size());
  nvote.add(alpha.size());
  nvote.add(theta.size());
  nvote.add(TParent.size());
  nvote.add(axes.size());
  nvote.add(qVec.size());
  nvote.add(qMinVec.size());
  nvote.add(qMaxVec.size());
  nvote.add(vMinVec.size());
  nvote.add(vMaxVec.size());
  nvote.add(tMaxVec.size());
  nvote.add(pMaxVec.size());
  nvote.erase(0);
  n = nvote.winner();
  //jointNames.resize(0);
  //joints.resize(0);
  bool sizeErr = false;
  if (!parents.empty() && n != parents.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of parents specified ("<<parents.size()<<")");
    sizeErr = true;
  }
  if (!linkNames.empty() && n != linkNames.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of link names specified ("<<linkNames.size()<<")");
    sizeErr = true;
  }
  if (!driverNames.empty() && nd != driverNames.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of driver names specified ("<<driverNames.size()<<")");
    sizeErr = true;
  }
  if (!drivers.empty() && nd != drivers.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of drivers specified ("<<drivers.size()<<")");
    sizeErr = true;
  }
  if (!jointType.empty() && n != jointType.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of joint types specified ("<<jointType.size()<<")");
    sizeErr = true;
  }
  if (!joints.empty() && nj != joints.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of joints specified ("<<joints.size()<<")");
    sizeErr = true;
  }
  if (!massVec.empty() && n != massVec.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of masses specified ("<<massVec.size()<<")");
    sizeErr = true;
  }
  if (!comVec.empty() && n != comVec.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of COMs specified ("<<comVec.size()<<")");
    sizeErr = true;
  }
  if (!inertiaVec.empty() && n != inertiaVec.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of inertia components specified ("<<inertiaVec.size()<<")");
    sizeErr = true;
  }
  if (!a.empty() && n != a.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of DH-parameters specified ("<<a.size()<<")");
    sizeErr = true;
  }
  if (!d.empty() && n != d.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of DH-parameters specified ("<<d.size()<<")");
    sizeErr = true;
  }
  if (!alpha.empty() && n != alpha.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of DH-parameters specified ("<<alpha.size()<<")");
    sizeErr = true;
  }
  if (!theta.empty() && n != theta.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of DH-parameters specified ("<<theta.size()<<")");
    sizeErr = true;
  }
  if (!TParent.empty() && n != TParent.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of link transforms specified ("<<TParent.size()<<")");
    sizeErr = true;
  }
  if (!axes.empty() && n != axes.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of axes specified ("<<axes.size()<<")");
    sizeErr = true;
  }
  if (!qVec.empty() && n != qVec.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of configuration variables specified ("<<qVec.size()<<")");
    sizeErr = true;
  }
  if (!qMaxVec.empty() && n != qMaxVec.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of joint limit variables specified ("<<qMaxVec.size()<<")");
    sizeErr = true;
  }
  if (!qMinVec.empty() && n != qMinVec.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of joint limit variables specified ("<<qMinVec.size()<<")");
    sizeErr = true;
  }
  if (!vMaxVec.empty() && n != vMaxVec.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of velocity limit variables specified ("<<vMaxVec.size()<<")");
    sizeErr = true;
  }
  if (!vMinVec.empty() && n != vMinVec.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of velocity limit variables specified ("<<vMinVec.size()<<")");
    sizeErr = true;
  }
  if (!tMaxVec.empty() && n != tMaxVec.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of torque limit variables specified ("<<tMaxVec.size()<<")");
    sizeErr = true;
  }
  if (!pMaxVec.empty() && n != pMaxVec.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of power limit variables specified ("<<pMaxVec.size()<<")");
    sizeErr = true;
  }
  if (!geomFn.empty() && n != geomFn.size()) {
    LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of geometry files specified ("<<geomFn.size()<<")");
    for (size_t i = 0; i < geomFn.size(); i++)
      LOG4CXX_INFO(GET_LOGGER(RobParser),"     '"<< geomFn[i] << "'");
    LOG4CXX_INFO(GET_LOGGER(RobParser),"");
    sizeErr = true;
  }
  if (!geomscale.empty() && n != geomscale.size() && 1 != geomscale.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of geometry scale variables specified ("<<geomscale.size()<<")");
    sizeErr = true;
  }
  if (!geommargin.empty() && n != geommargin.size() && 1 != geommargin.size()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of geometry margin variables specified ("<<geommargin.size()<<")");
    sizeErr = true;
  }

  if (sizeErr) {
    LOG4CXX_ERROR(GET_LOGGER(RobParser),"Size votes:");
    for(auto i=nvote.counts.begin();i!=nvote.counts.end();i++)
      LOG4CXX_ERROR(GET_LOGGER(RobParser),i->first<<": "<<i->second);
    return false;
  }

  LOG4CXX_INFO(GET_LOGGER(RobParser),"   Parsing robot file, "<< n <<" links read...");
  if (parents.empty()) {
    parents.resize(n);
    for (int i = 0; i < (int) n; i++)
      parents[i] = i - 1;
  }
  Initialize(n);

  //Init standard stuff
  if (linkNames.empty()) {
    linkNames.resize(links.size());
    for (size_t i = 0; i < links.size(); i++) {
      char buf[64];
      snprintf(buf,64, "Link_%d", (int)i);
      linkNames[i] = buf;
    }
  }

  accMax.resize(n, Inf);
  if (qVec.empty())
    q.set(Zero);
  else
    q.copy(&qVec[0]);
  if (qMinVec.empty())
    qMin.set(-Inf);
  else
    qMin.copy(&qMinVec[0]);
  if (qMaxVec.empty())
    qMax.set(Inf);
  else
    qMax.copy(&qMaxVec[0]);
  if (vMaxVec.empty())
    velMax.set(Inf);
  else
    velMax.copy(&vMaxVec[0]);
  if (vMinVec.empty())
    velMin.setNegative(velMax);
  else
    velMin.copy(&vMinVec[0]);
  if (tMaxVec.empty())
    torqueMax.set(Inf);
  else
    torqueMax.copy(&tMaxVec[0]);
  if (pMaxVec.empty())
    powerMax.set(Inf);
  else
    powerMax.copy(&pMaxVec[0]);
  if (aMaxVec.empty()) {
    if (tMaxVec.empty())
      accMax.set(Inf);
    else
      GetAccMax(*this, accMax);
  } else
    accMax.copy(&aMaxVec[0]);
  if (axes.empty())
    axes.resize(TParent.size(), Vector3(0, 0, 1));
  //dq.resize(n,0);
  if (TParent.empty()
      && (a.empty() || d.empty() || alpha.empty() || theta.empty())) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   No D-H parameters or link transforms specified");
    return false;
  } else if (TParent.empty()) {
    DenavitHartenbergRobotSetup(alpha, a, d, theta, *this);
  } else {
    for (size_t i = 0; i < links.size(); i++) {
      links[i].type = RobotLink3D::Revolute;
      links[i].T0_Parent = TParent[i];
      links[i].w = axes[i];
    }
  }

  for (size_t i = 0; i < links.size(); i++) {
    if (parents[i] < 0) {
      links[i].T0_Parent = baseTransform * links[i].T0_Parent;
    }
  }
  if (!jointType.empty()) {
    for (size_t i = 0; i < links.size(); i++) {
      links[i].type = (RobotLink3D::Type) jointType[i];
    }
  }
  UpdateFrames();
  if (comVec.empty()) {
    for (size_t i = 0; i < links.size(); i++)
      links[i].com.setZero();
  } else {
    for (size_t i = 0; i < links.size(); i++)
      links[i].com = comVec[i];
  }
  if (massVec.empty()) {
    for (size_t i = 0; i < links.size(); i++)
      links[i].mass = 1.0;
  } else {
    for (size_t i = 0; i < links.size(); i++)
      links[i].mass = massVec[i];
  }
  if (inertiaVec.empty()) {
    for (size_t i = 0; i < links.size(); i++)
      links[i].inertia.setIdentity();
  } else {
    for (size_t i = 0; i < links.size(); i++)
      links[i].inertia = inertiaVec[i];
  }
  //automatically compute torque limits from COMs
  if (autoTorque != 0.0) {
    Real grav = Abs(autoTorque) * 9.8;
    //TODO: more sophisticated chain structures
    Real sumMass = 0;
    Real sumCom = 0;
    for (int i = (int) links.size() - 1; i >= 0; i--) {
      Real oldMass = sumMass;
      Real oldCom = sumCom;
      sumMass += links[i].mass;
      oldCom += links[i].T0_Parent.t.length();
      sumCom = (links[i].com.length() * links[i].mass + oldCom * oldMass)
          / (links[i].mass + oldMass);
      torqueMax[i] = sumCom * sumMass * grav;
      //printf("Torque Max %d = %g, moment arm = %g\n",i,torqueMax[i],sumCom);
    }
  }

  //TODO: whole-body scaling isn't done yet
  if (scale != 1.0)
    FatalError("Scale not done yet");
  if (geomscale.size() == 1)
    geomscale.resize(n, geomscale[0]);
  
  geomManagers.resize(n);
  geomFiles.resize(n);
  Timer timer;
  for (size_t i = 0; i < geomFn.size(); i++) {
    if (geomFn[i].empty()) {
      continue;
    }
    geomFiles[i] = geomFn[i];
    geomFn[i] = ResolveFileReference(path,geomFn[i]);
    if(Robot::disableGeometryLoading) continue;
    if (!LoadGeometry(i, geomFn[i].c_str())) {
      LOG4CXX_ERROR(GET_LOGGER(RobParser),"   Unable to load link "<<i<<" geometry file "<<geomFn[i]);
      return false;
    }
    if (!geomscale.empty() && geomscale[i] != 1) {
      Matrix4 mscale;
      mscale.setIdentity();
      mscale(0, 0) = mscale(1, 1) = mscale(2, 2) = geomscale[i];
      geomManagers[i].TransformGeometry(mscale);
      geometry[i] = geomManagers[i];
    }
    if (geommargin.size() == 1)
      geometry[i]->margin = geommargin[0];
    else if(i < geommargin.size())
      geometry[i]->margin = geommargin[i];
  }
  int numGeomElements = 0;
  for(size_t i=0;i<geometry.size();i++)
    numGeomElements += (geometry[i] ? geometry[i]->NumElements() : 0);
  LOG4CXX_INFO(GET_LOGGER(RobParser),"Loaded geometries in time "<<timer.ElapsedTime()<<"s, "<<numGeomElements<<" total primitive elements");
  timer.Reset();

  //process transformation of geometry shapes
  if(geomTransformIndex.size() != geomTransform.size()){
    LOG4CXX_ERROR(GET_LOGGER(RobParser),"   Size of geomTransformIndex and geomTransform size not match!");
    return false;
  }
  for(size_t i = 0; i < geomTransformIndex.size(); i++){
    int geomIndex = geomTransformIndex[i];
    if (!geometry[geomIndex]) continue;
    if (geomFn[geomIndex].empty()) {
      continue;
    }
    geomManagers[geomIndex].TransformGeometry(geomTransform[i]);
    geometry[geomIndex] = geomManagers[geomIndex];
  }

  if (collision.empty()) {
    //TESTING: don't need to do this with dynamic initialization
    //InitCollisions();
  }
  else {
    FatalError("So far, no mechanism to select environment collisions");
  }
  for (size_t i = 0; i < noCollision.size(); i++) {
    int link = LinkIndex(noCollision[i].c_str());
    if (link < 0 || link >= (int) links.size()) {
      LOG4CXX_ERROR(GET_LOGGER(RobParser),"   Invalid no-collision index "<<noCollision[i]);
      return false;
    }
    FatalError("So far, no mechanism to turn off collisions");
  }

  if (joints.empty()) {
    joints.resize(links.size());
    for (size_t i = 0; i < links.size(); i++) {
      if (parents[i] == -1) {
        if (velMin[i] == 0 && velMax[i] == 0)
          joints[i].type = RobotJoint::Weld;
        else
          joints[i].type = RobotJoint::Normal;
      } else
        joints[i].type = RobotJoint::Normal;
      joints[i].linkIndex = (int) i;
    }
  }
  if (drivers.empty()) {
//    Assert(joints.size() == links.size());
    drivers.resize(0);
    for (size_t i = 0; i < joints.size(); i++) {
      if (joints[i].type != RobotJoint::Normal
          && joints[i].type != RobotJoint::Spin)
        continue;
      RobotJointDriver d;
      d.type = RobotJointDriver::Normal;
      d.linkIndices.push_back(i);
      d.qmin = qMin(i);
      d.qmax = qMax(i);
      d.vmin = velMin(i);
      d.vmax = velMax(i);
      d.tmin = -torqueMax(i);
      d.tmax = torqueMax(i);
      d.amin = -accMax(i);
      d.amax = accMax(i);
      d.servoP = 0;
      d.servoI = 0;
      d.servoD = 0;
      d.dryFriction = 0;
      d.viscousFriction = 0;
      drivers.push_back(d);
    }
    nd = (int) drivers.size();
  } else { //load normal driver settings
    for (size_t i = 0; i < drivers.size(); i++) {
      if (drivers[i].type == RobotJointDriver::Normal
          || drivers[i].type == RobotJointDriver::Translation
          || drivers[i].type == RobotJointDriver::Rotation) {
        int itemp = drivers[i].linkIndices[0];
        Assert(itemp >= 0 && itemp < (int) n);
        drivers[i].qmin = qMin(itemp);
        drivers[i].qmax = qMax(itemp);
        drivers[i].vmin = velMin(itemp);
        drivers[i].vmax = velMax(itemp);
        drivers[i].tmin = -torqueMax(itemp);
        drivers[i].tmax = torqueMax(itemp);
        drivers[i].amin = -accMax(itemp);
        drivers[i].amax = accMax(itemp);
      }
    }
  }
  if (driverNames.empty()) {
    driverNames.resize(0);
    for (size_t i = 0; i < drivers.size(); i++) {
      driverNames.push_back(linkNames[drivers[i].linkIndices[0]]);
    }
  }
  //setup driver servo parameters, if they exist
  if (!servoP.empty() && servoP.size() != nd) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser),
        "   Wrong number of servo P parameters specified: "<<servoP.size()<<" vs "<<nd);
    return false;
  }
  if (!servoI.empty() && servoI.size() != nd) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of servo I parameters specified ("<<servoI.size()<<")");
    return false;
  }
  if (!servoD.empty() && servoD.size() != nd) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of servo D parameters specified ("<<servoD.size()<<")");
    return false;
  }
  if (!dryFriction.empty() && dryFriction.size() != nd) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of dry friction parameters specified ("<<dryFriction.size()<<")");
    return false;
  }
  if (!viscousFriction.empty() &&  viscousFriction.size() != nd) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser), "   Wrong number of viscous friction parameters specified ("<<viscousFriction.size()<<")");
    return false;
  }
  for (size_t i = 0; i < servoP.size(); i++) {
    drivers[i].servoP = servoP[i];
    Assert(servoP[i] >= 0);
  }
  for (size_t i = 0; i < servoI.size(); i++) {
    drivers[i].servoI = servoI[i];
    Assert(servoI[i] >= 0);
  }
  for (size_t i = 0; i < servoD.size(); i++) {
    drivers[i].servoD = servoD[i];
    Assert(servoD[i] >= 0);
  }
  for (size_t i = 0; i < dryFriction.size(); i++) {
    drivers[i].dryFriction = dryFriction[i];
    Assert(dryFriction[i] >= 0);
  }
  for (size_t i = 0; i < viscousFriction.size(); i++) {
    drivers[i].viscousFriction = viscousFriction[i];
    Assert(viscousFriction[i] >= 0);
  }

  if (!CheckValid())
    return false;


  //first mount the geometries, they affect whether a link is included in self collision testing
  vector<int> mountLinkIndices(mountLinks.size());
  for (size_t i = 0; i < mountLinks.size(); i++) {
    int linkIndex = LinkIndex(mountLinks[i].c_str());
    if(linkIndex < 0 || linkIndex >= (int)links.size()) {
      if(mountLinks[i]=="-1") {
        //pass
      }
      else {
        LOG4CXX_ERROR(GET_LOGGER(RobParser),"   Invalid mount link "<<mountLinks[i]<<", out of range");
        return false;
      }
    }
    mountLinkIndices[i] = linkIndex;
  }
  for (size_t i = 0; i < mountLinks.size(); i++) {
    const char* ext = FileExtension(mountFiles[i].c_str());
    if(ext && (0==strcmp(ext,"rob") || 0==strcmp(ext,"urdf"))) {
      //its a robot, delay til later
    }
    else {
      string fn = ResolveFileReference(path,mountFiles[i]);
      LOG4CXX_INFO(GET_LOGGER(RobParser),"   Mounting geometry file " << mountFiles[i]);
      //mount a triangle mesh on top of another triangle mesh
      ManagedGeometry loader;
      if(!loader.Load(fn.c_str())) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser),"   Error loading mount geometry file " << fn);
        return false;
      }
      Mount(mountLinkIndices[i], *loader, mountT[i]);
    }
  }

  //automatically compute mass parameters from geometry
  if (autoMass) {
    for (size_t i = 0; i < links.size(); i++) {
      if (comVec.empty()) {
        if (geometry[i] && !geometry[i]->Empty())
          links[i].com = CenterOfMass(*geometry[i],surfaceFraction);
        else
          links[i].com.setZero();
      }
      if (inertiaVec.empty()) {
        if (!IsGeometryEmpty(i) && links[i].mass != 0.0) {
          links[i].inertia = Inertia(*geometry[i], links[i].com,
              links[i].mass,surfaceFraction);
          //check for infinity
          if(!links[i].inertia.isZero(1e300)) {
            LOG4CXX_INFO(GET_LOGGER(RobParser),"Huge automass inertia for "<<linkNames[i]<<": "<<links[i].inertia);
            LOG4CXX_INFO(GET_LOGGER(RobParser),"Press enter to continue...");
            //KrisLibrary::loggerWait();
          }
        } else {
          links[i].inertia.setZero();
          //LOG4CXX_INFO(GET_LOGGER(RobParser),"Automass setting zero inertia for "<<linkNames[i]);
        }
      }
    }
  }

  //Initialize self collisions -- pre subchain mounting
  CleanupSelfCollisions();
  vector<pair<string,string> > residualSelfCollisions,residualNoSelfCollisions;
  if (selfCollision.empty()) {
    InitAllSelfCollisions();
  } else {
    for (size_t i = 0; i < selfCollision.size(); i++) {
      int link1,link2;
      link1 = LinkIndex(selfCollision[i].first.c_str());
      link2 = LinkIndex(selfCollision[i].second.c_str());
      if (link1 < 0 || link1 >= (int) links.size() ||
          link2 < 0 || link2 >= (int) links.size()) {
        residualSelfCollisions.push_back(selfCollision[i]);
        continue;
      }
      if(link1 > link2) Swap(link1,link2);
      if(!(link1 < link2)) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser),"Robot::Load(): Invalid self collision pair "<<selfCollision[i].first<<", "<<selfCollision[i].second);
          return false;
      }
      InitSelfCollisionPair(link1,link2);
    }
  }

  for (size_t i = 0; i < noSelfCollision.size(); i++) {
      int link1,link2;
      link1 = LinkIndex(noSelfCollision[i].first.c_str());
      link2 = LinkIndex(noSelfCollision[i].second.c_str());
      if (link1 < 0 || link1 >= (int) links.size() ||
          link2 < 0 || link2 >= (int) links.size()) {
        residualNoSelfCollisions.push_back(noSelfCollision[i]);
        continue;
    }
      if(link1 > link2) Swap(link1,link2);
      if(!(link1 < link2)) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser),"Robot::Load(): Invalid no-self collision pair "<<noSelfCollision[i].first<<", "<<noSelfCollision[i].second);
        return false;
      }
    SafeDelete(selfCollisions(link1,link2));
  }


  timer.Reset();
  //do the mounting of subchains
  for (size_t i = 0; i < mountLinks.size(); i++) {
    const char* ext = FileExtension(mountFiles[i].c_str());
    if(ext && (0==strcmp(ext,"rob") || 0==strcmp(ext,"urdf"))) {
      string fn = ResolveFileReference(path,mountFiles[i]);
      LOG4CXX_INFO(GET_LOGGER(RobParser),"   Mounting subchain file " << mountFiles[i]);
      Robot subchain;
      if (!subchain.Load(fn.c_str())) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser),"   Error reading subchain file " << fn);
        return false;
      }
      const char* prefix = (mountNames[i].empty() ? NULL : mountNames[i].c_str());
      Mount(mountLinkIndices[i], subchain, mountT[i], prefix);
    }
  }
  if (!CheckValid())
    return false;

  //after mounting may need to add extra self collisions / no self collisions
  swap(selfCollision,residualSelfCollisions);
  swap(noSelfCollision,residualNoSelfCollisions);
  for (size_t i = 0; i < selfCollision.size(); i++) {
    int link1,link2;
    link1 = LinkIndex(selfCollision[i].first.c_str());
    link2 = LinkIndex(selfCollision[i].second.c_str());
    if (link1 < 0 || link1 >= (int) links.size() ||
        link2 < 0 || link2 >= (int) links.size()) {
      LOG4CXX_ERROR(GET_LOGGER(RobParser),"   Error, invalid self-collision index "<<selfCollision[i].first.c_str()<<"-"<<
        selfCollision[i].second.c_str()<<" (range is 0,...,"<<(int)links.size()-1 <<")");
      return false;
    }
    if(link1 > link2) Swap(link1,link2);
    if(!(link1 < link2)) {
      LOG4CXX_ERROR(GET_LOGGER(RobParser),"Robot::Load(): Invalid self collision pair "<<selfCollision[i].first<<", "<<selfCollision[i].second);
      return false;
    }
    InitSelfCollisionPair(link1,link2);
  }

  for (size_t i = 0; i < noSelfCollision.size(); i++) {
    int link1,link2;
    link1 = LinkIndex(noSelfCollision[i].first.c_str());
    link2 = LinkIndex(noSelfCollision[i].second.c_str());
    if (link1 < 0 || link1 >= (int) links.size() ||
        link2 < 0 || link2 >= (int) links.size()) {
      LOG4CXX_ERROR(GET_LOGGER(RobParser),"  Error, invalid no-collision index "<< noSelfCollision[i].first.c_str()<<"-"<<
        noSelfCollision[i].second.c_str()<<" (range is 0,...,"<< (int)links.size()-1<<")");
      return false;
    }
    if(link1 > link2) Swap(link1,link2);
    if(!(link1 < link2)) {
      LOG4CXX_ERROR(GET_LOGGER(RobParser),"Robot::Load(): Invalid no-self collision pair "<<noSelfCollision[i].first<<", "<<noSelfCollision[i].second);
      return false;
    }
    SafeDelete(selfCollisions(link1,link2));
  }
  LOG4CXX_INFO(GET_LOGGER(RobParser),"Done loading robot file "<<fn);
  return true;
}

void Robot::InitStandardJoints() {
  if (linkNames.empty()) {
    linkNames.resize(links.size());
    for (size_t i = 0; i < links.size(); i++) {
      char buf[64];
      snprintf(buf,64, "Link %d", (int)i);
      linkNames[i] = buf;
    }
  }
  joints.resize(links.size());
  int numNormal = 0;
  for (size_t i = 0; i < links.size(); i++) {
    if (parents[i] == -1)
      joints[i].type = RobotJoint::Weld;
    else {
      joints[i].type = RobotJoint::Normal;
      numNormal++;
    }
    joints[i].linkIndex = (int) i;
  }
  drivers.resize(0);
  driverNames.resize(0);
  for (size_t i = 0; i < links.size(); i++) {
    if (joints[i].type != RobotJoint::Normal
        && joints[i].type != RobotJoint::Spin)
      continue;
    RobotJointDriver d;
    d.type = RobotJointDriver::Normal;
    d.linkIndices.push_back(i);
    d.qmin = qMin(i);
    d.qmax = qMax(i);
    d.vmin = velMin(i);
    d.vmax = velMax(i);
    d.tmin = -torqueMax(i);
    d.tmax = torqueMax(i);
    drivers.push_back(d);
    driverNames.push_back(linkNames[i]);
  }
}

void Robot::SetGeomFiles(const char* geomPath,const char* geomExt) {
  geomFiles.resize(links.size());
  for (size_t i = 0; i < links.size(); i++) {
    stringstream ss;
    ss << geomPath << linkNames[i] << "." << geomExt;
    geomFiles[i] = ss.str();
  }
}

void Robot::SetGeomFiles(const vector<string>& files)
{
  geomFiles = files;
}

bool Robot::LoadGeometry(int i,const char* file)
{
  if(i >= (int)geomManagers.size())
    geomManagers.resize(geometry.size());
  //make the default appearance be grey, so that loader may override it
  if(geomManagers[i].Load(file)) {
    geometry[i] = geomManagers[i];
    SetDefaultAppearance(geomManagers[i].Appearance());
    return true;
  }
  return false;
}

bool Robot::SaveGeometry(const char* prefix) {
  for (size_t i = 0; i < links.size(); i++) {
    if (!IsGeometryEmpty(i)) {
      if(geomFiles[i].empty()) {
        LOG4CXX_ERROR(GET_LOGGER(RobParser),"Robot::SaveGeometry: warning, link "<<i<<" has empty file name");
        continue;
      }
      if(!geometry[i]->Save((string(prefix)+geomFiles[i]).c_str())) {
           LOG4CXX_ERROR(GET_LOGGER(RobParser), "Robot::SaveGeometry: Unable to save to geometry file " << string(prefix)+geomFiles[i] << "");
           return false;
         }
    }
  }
  return true;
}

bool Robot::Save(const char* fn) {
  ofstream file;
  file.open(fn, ios::out);
  if (!file.is_open()) {
    LOG4CXX_ERROR(GET_LOGGER(RobParser), fn << " cannot be opened!" << "");
    file.close();
    return false;
  }
  int nLinks = links.size();
  file << "links\t";
  for (int i = 0; i < nLinks; i++) {
    file << "\"" << linkNames[i] << "\" ";
  }
  file << endl << endl;

  file << "parents\t";
  for (int i = 0; i < (int) parents.size(); i++) {
    file << parents[i] << " ";
  }
  file << endl << endl;

  file << "axis\t";
  for (int i = 0; i < nLinks; i++)
    file << links[i].w << "\t";
  file << endl << endl;

  file << "jointtype\t";
  for (int i = 0; i < nLinks; i++) {
    if (links[i].type == RobotLink3D::Prismatic)
      file << "p ";
    else
      file << "r ";
  }
  file << endl << endl;

  file << "Tparent\t";
  for (int i = 0; i < nLinks; i++) {
    RigidTransform T(links[i].T0_Parent);
    file << T.R(0, 0) << " " << T.R(0, 1) << " " << T.R(0, 2) << "\t";
    file << T.R(1, 0) << " " << T.R(1, 1) << " " << T.R(1, 2) << "\t";
    file << T.R(2, 0) << " " << T.R(2, 1) << " " << T.R(2, 2) << "\t";
    file << T.t[0] << " " << T.t[1] << " " << T.t[2];
    if (i < nLinks - 1)
      file << " \\" << endl;
  }
  file << endl << endl;

  file <<"q\t";
  for (int i = 0; i < nLinks; i++)
    file << q[i] << " ";
  file << endl << endl;

  file << "qmin\t";
  for (int i = 0; i < nLinks; i++)
    file << qMin[i] << " ";
  file << endl << endl;

  file << "qmax\t";
  for (int i = 0; i < nLinks; i++)
    file << qMax[i] << " ";
  file << endl << endl;

  file << "geometry\t";
  for (int i = 0; i < nLinks; i++) {
    if (!geometry[i] || geometry[i]->Empty())
      file << "\"\" ";
    else
      file << "\"" << geomFiles[i] << "\" ";
  }
  file << endl << endl;

  file << "mass\t";
  for (int i = 0; i < nLinks; i++) {
    file << links[i].mass << " ";
  }
  file << endl << endl;

  file << "com\t";
  for (int i = 0; i < nLinks; i++)
    file << links[i].com[0] << " " << links[i].com[1] << " "
        << links[i].com[2] << "\t";
  file << endl << endl;

  file << "inertia\t";
  for (int i = 0; i < nLinks; i++) {
    Matrix3 inertia(links[i].inertia);
    file << inertia(0, 0) << " " << inertia(0, 1) << " " << inertia(0, 2)
        << " ";
    file << inertia(1, 0) << " " << inertia(1, 1) << " " << inertia(1, 2)
        << " ";
    file << inertia(2, 0) << " " << inertia(2, 1) << " " << inertia(2, 2)
        << "\t";
  }
  file << endl << endl;

  file << "torquemax\t";
  for (int i = 0; i < nLinks; i++) {
    file << torqueMax[i] << " ";
  }
  file << endl << endl;

  file << "velmax\t";
  for (int i = 0; i < nLinks; i++) {
    file << velMax[i] << " ";
  }
  file << endl << endl;

  file << "accmax\t";
  for (int i = 0; i < nLinks; i++) {
    file << accMax[i] << " ";
  }
  file << endl << endl;

  for(int i=0;i<nLinks;i++) {
    if(!geometry[i] || geometry[i]->Empty()) continue;
    vector<int> nocollision;
    for(int j=i+1;j<nLinks;j++) {
      if(!geometry[j] || geometry[j]->Empty()) continue;
      if(parents[i] != j && parents[j] != i)
        if(selfCollisions(i,j) == NULL)
    nocollision.push_back(j);
    }
    if(!nocollision.empty()) {
      file<<"noselfcollision\t";
      for(size_t j=0;j<nocollision.size();j++)
        file<<i<<" "<<nocollision[j]<<"\t";
      file<<endl;
    }
  }
  file << endl;

  int nJoints = joints.size();
  for (int i = 0; i < nJoints; i++) {
    file << "joint ";
    switch (joints[i].type) {
    case RobotJoint::Floating:
      file << "floating " << joints[i].linkIndex << " "
          << joints[i].baseIndex << endl;
      break;
    case RobotJoint::Normal:
      file << "normal " << joints[i].linkIndex << endl;
      break;
    case RobotJoint::Weld:
      file << "weld " << joints[i].linkIndex << endl;
      break;
    case RobotJoint::Spin:
      file << "spin " << joints[i].linkIndex << endl;
      break;
    case RobotJoint::FloatingPlanar:
      file << "floating2d " << joints[i].linkIndex << " "
          << joints[i].baseIndex << endl;
      break;
    case RobotJoint::BallAndSocket:
      file << "ballandsocket " << joints[i].linkIndex << " "
          << joints[i].baseIndex << endl;
      break;
    default:
      LOG4CXX_ERROR(GET_LOGGER(RobParser), "Unable to save joint type " << (int)joints[i].type
          << "");
      return false;
    }
  }
  file << endl << endl;

  int nDrivers = drivers.size();
  for (int i = 0; i < nDrivers; i++) {
    switch (drivers[i].type) {
    case RobotJointDriver::Normal:
      if(drivers[i].linkIndices.size() > 0){
        file << "driver normal " << drivers[i].linkIndices[0] << endl;
      }
      break;
    case RobotJointDriver::Affine:
      if(drivers[i].linkIndices.size() > 0){
        file << "driver affine "<<drivers[i].linkIndices.size() <<"\t";
        for (auto l:drivers[i].linkIndices)
          file << l<<" ";
        file << "\t";
        for (auto l:drivers[i].affScaling)
          file << l<<" ";
        file << "\t";
        for (auto l:drivers[i].affOffset)
          file << l<<" ";
        file << "\t";
        file << drivers[i].qmin<<" "<<drivers[i].qmax<<"\t"<<drivers[i].vmin<<" "<<drivers[i].vmax<<"\t"<<drivers[i].tmin<<" "<<drivers[i].tmax<<endl;
      }
      break;
    default:
      LOG4CXX_ERROR(GET_LOGGER(RobParser), "Unable to save driver type " << (int) drivers[i].type
          << "");
      return false;
    }
  }
  file << endl << endl;

  file << "servoP\t";
  for (int i = 0; i < nDrivers; i++) {
    file << drivers[i].servoP << " ";
  }
  file << endl;
  file << "servoI\t";
  for (int i = 0; i < nDrivers; i++) {
    file << drivers[i].servoI << " ";
  }
  file << endl;
  file << "servoD\t";
  for (int i = 0; i < nDrivers; i++) {
    file << drivers[i].servoD << " ";
  }
  file << endl << endl;

  file << "dryFriction\t";
  for (int i = 0; i < nDrivers; i++) {
    file << drivers[i].dryFriction << " ";
  }
  file << endl;

  file << "viscousFriction\t";
  for (int i = 0; i < nDrivers; i++) {
    file << drivers[i].viscousFriction << " ";
  }
  file << endl;
  file << endl;

  for(map<string,string>::const_iterator i=properties.begin();i!=properties.end();i++) {
    file << "property "<<i->first<<" ";
    SafeOutputString(file,i->second);
    file << endl;
  }
  file.close();
  return true;
}


bool Robot::CheckValid() const {
  for (size_t i = 0; i < parents.size(); i++)
    if (parents[i] < -1 || parents[i] >= (int) parents.size()) {
      LOG4CXX_ERROR(GET_LOGGER(Robot),"Invalid parent["<<i<<"]=" << parents[i]);
      return false;
    }
  /*
  for (size_t i = 0; i < links.size(); i++) {
    if (!FuzzyEquals(links[i].w.norm(), 1.0)) {
      printf("Invalid axis %d\n", i);
      return false;
    }
  }
  */
  vector<int> matchedLink(links.size(), -1);
  vector<int> drivenLink(links.size(), -1);

  //check joint definitions
  for (size_t i = 0; i < joints.size(); i++) {
    switch (joints[i].type) {
    case RobotJoint::Weld:
    case RobotJoint::Normal:
    case RobotJoint::Spin:
      if (matchedLink[joints[i].linkIndex]>=0) {
        LOG4CXX_ERROR(GET_LOGGER(Robot),"Joint "<<i<<" controls an already controlled link, "<<joints[i].linkIndex<<" controlled by "<<matchedLink[joints[i].linkIndex] ); 
        return false;
      }
      matchedLink[joints[i].linkIndex] = (int)i;
      if (joints[i].linkIndex < 0
          || joints[i].linkIndex >= (int) links.size()) {
        LOG4CXX_ERROR(GET_LOGGER(Robot),"Invalid joint "<<i<<" index " << joints[i].linkIndex);
        return false;
      }
      break;
    case RobotJoint::Floating: {
      int numLinks = 0;
      int link = joints[i].linkIndex;
      while (link != joints[i].baseIndex) {
        if (link < 0) {
          LOG4CXX_ERROR(GET_LOGGER(Robot),"Invalid floating chain");
          return false;
        }
        if (matchedLink[link] >= 0) {
          LOG4CXX_ERROR(GET_LOGGER(Robot),"Joint "<<i<<" controls an already controlled link, "<<link<<" controlled by "<<matchedLink[link]);
          return false;
        }
        matchedLink[link] = (int)i;
        link = parents[link];
        numLinks++;
      }
      if (numLinks != 6) {
        LOG4CXX_ERROR(GET_LOGGER(Robot),"Floating joints must have exactly 6 DOF");
        return false;
      }
    }
      break;
    case RobotJoint::FloatingPlanar: {
      int numLinks = 0;
      int link = joints[i].linkIndex;
      while (link != joints[i].baseIndex) {
        if (link < 0) {
          LOG4CXX_ERROR(GET_LOGGER(Robot),"Invalid floatingplanar chain");
          return false;
        }
        if (matchedLink[link] >= 0) {
          LOG4CXX_ERROR(GET_LOGGER(Robot),"Joint "<<i<<" controls an already controlled link, "<<link<<" controlled by "<<matchedLink[link]);
          return false;
        }
        matchedLink[link] = (int)i;
        link = parents[link];
        numLinks++;
      }
      if (numLinks != 3) {
        LOG4CXX_ERROR(GET_LOGGER(Robot),"Planar floating joints must have exactly 3 DOF");
        return false;
      }
    }
      break;
    case RobotJoint::BallAndSocket: {
      int numLinks = 0;
      int link = joints[i].linkIndex;
      while (link != joints[i].baseIndex) {
        if (link < 0) {
          LOG4CXX_ERROR(GET_LOGGER(Robot),"Invalid ballandsocket chain");
          return false;
        }
        if (matchedLink[link] >= 0) {
          LOG4CXX_ERROR(GET_LOGGER(Robot),"Joint "<<i<<" controls an already controlled link, "<<link<<" controlled by "<<matchedLink[link]);
          return false;
        }
        matchedLink[link] = (int)i;
        link = parents[link];
        numLinks++;
      }
      if (numLinks != 3) {
        LOG4CXX_ERROR(GET_LOGGER(Robot),"Ball-and-socket joints must have exactly 3 DOF");
        return false;
      }
    }
      break;
    default:
      LOG4CXX_ERROR(GET_LOGGER(Robot),"Can't yet handle joints of type " << joints[i].type);
      return false;
    }
  }
  for (size_t i = 0; i < matchedLink.size(); i++) {
    if (matchedLink[i] < 0) {
      LOG4CXX_ERROR(GET_LOGGER(Robot),"Link "<<i<<" not matched by a joint");
    }
  }

  //check drivers
  for (size_t i = 0; i < drivers.size(); i++) {
    if (drivers[i].linkIndices.empty()) {
      LOG4CXX_ERROR(GET_LOGGER(Robot),"Driver "<<i<<" doesn't affect any joints");
      return false;
    }
    if (drivers[i].type == RobotJointDriver::Normal
        || drivers[i].type == RobotJointDriver::Affine) {
      for (size_t j = 0; j < drivers[i].linkIndices.size(); j++) {
        if (drivenLink[drivers[i].linkIndices[j]] >= 0) {
          LOG4CXX_ERROR(GET_LOGGER(Robot), "Driver " << i << "affects an already driven link, "<< drivers[i].linkIndices[j] <<" driven by" << drivenLink[drivers[i].linkIndices[j]]);
          return false;
        }
        drivenLink[drivers[i].linkIndices[j]] = (int)i;
      }
    } else if (drivers[i].type == RobotJointDriver::Translation
        || drivers[i].type == RobotJointDriver::Rotation) {
      //only the first linkindex is actually driven
      if (drivenLink[drivers[i].linkIndices[0]] >= 0) {
        LOG4CXX_ERROR(GET_LOGGER(Robot), "Driver " << i << "affects an already driven link, " << drivers[i].linkIndices[0] << " driven by" << drivenLink[drivers[i].linkIndices[0]]);
        return false;
      }
      drivenLink[drivers[i].linkIndices[0]] = (int)i;
    }
  }
  return true;
}

void concat(Vector& x, const Vector& y) {
  Vector temp(x.n + y.n);
  temp.copySubVector(0, x);
  temp.copySubVector(x.n, y);
  x = temp;
}

template<class T>
void concat(Array2D<T>& x, const Array2D<T>& y, T emptyVal = 0) {
  Array2D<T> temp(x.m + y.m, x.n + y.n, emptyVal);
  for (int i = 0; i < x.m; i++)
    for (int j = 0; j < x.n; j++)
      temp(i, j) = x(i, j);
  for (int i = 0; i < y.m; i++)
    for (int j = 0; j < y.n; j++)
      temp(i + x.m, j + x.n) = y(i, j);
  x = temp;
}

void Robot::Mount(int link, const Geometry::AnyGeometry3D& mesh,
    const RigidTransform& T) {
  if(!geometry[link]) {
    if(link >= (int)geomManagers.size()) {
      LOG4CXX_INFO(GET_LOGGER(Robot),"Robot::Mount (geometry): Need to add geometry managers?");
      geomManagers.resize(geometry.size());
    }
    geomManagers[link].CreateEmpty();
    *geomManagers[link] = mesh;
    geomManagers[link]->Transform(Matrix4(T));
    geometry[link] = geomManagers[link];
    geomManagers[link].Appearance()->Set(*geometry[link]);
    SetDefaultAppearance(geomManagers[link].Appearance());
  }
  else {
    vector<Geometry::AnyGeometry3D> mergeMeshes(2);
    mergeMeshes[0] = *geometry[link];
    mergeMeshes[1] = mesh;
    mergeMeshes[1].Transform(Matrix4(T));
    if(link < (int)geomManagers.size()) {
      geomManagers[link].RemoveFromCache();
      geomManagers[link].SetUniqueAppearance();
    }
    else {
      LOG4CXX_INFO(GET_LOGGER(Robot),"Robot::Mount (geometry): Need to add geometry managers?");
      geomManagers.resize(geometry.size());
    }
    geomManagers[link].CreateEmpty();
    geomManagers[link]->Merge(mergeMeshes);
    geometry[link] = geomManagers[link];
    geomManagers[link].Appearance()->Set(*geometry[link]);
  }
  //TODO: reinitialize all self collisions with this mesh
}

void Robot::Mount(int link, const Robot& subchain, const RigidTransform& T,const char* prefix)
{
  Assert(&subchain != this);
  size_t norig = links.size();
  size_t lstart = links.size();
  size_t dstart = drivers.size();

  ArrayUtils::concat(links, subchain.links);
  //update mounting transform
  RigidTransform T0;
  if (link < 0)
    T0.setIdentity();
  else
    T0 = links[link].T_World;
  for (size_t i = 0; i < subchain.links.size(); i++) {
    links[i + norig].T_World = T0 * T * links[i + norig].T_World;
    if (subchain.parents[i] == -1) {
      links[i + norig].T0_Parent = T * links[i + norig].T0_Parent;
    }
  }
  ArrayUtils::concat(linkNames, subchain.linkNames);
  ArrayUtils::concat(parents, subchain.parents);
  //update parent numbering
  for (size_t i = 0; i < subchain.links.size(); i++) {
    if (parents[i + norig] < 0)
      parents[i + norig] = link;
    else
      parents[i + norig] += norig;
  }
  concat(q, subchain.q);
  concat(dq, subchain.dq);
  concat(qMax, subchain.qMax);
  concat(qMin, subchain.qMin);
  concat(velMax, subchain.velMax);
  concat(velMin, subchain.velMin);
  concat(torqueMax, subchain.torqueMax);
  concat(powerMax, subchain.powerMax);
  concat(accMax, subchain.accMax);
  ArrayUtils::concat(geometry, subchain.geometry);
  ArrayUtils::concat(geomManagers, subchain.geomManagers);
  ArrayUtils::concat(geomFiles, subchain.geomFiles);
  for(size_t i=0;i<geometry.size();i++) {
    //do we need to re-init collisions?
    /*
    if(geometry[i].collisionData.empty()) {
      //TESTING: don't need to do this with dynamic initialization
      geometry[i].InitCollisions();
    }
    */
  }
  ArrayUtils::concat(envCollisions, subchain.envCollisions);
  concat(selfCollisions, subchain.selfCollisions);
  //need to re-initialize the self collision pointers
  for (int i = 0; i < selfCollisions.m; i++) {
    for (int j = 0; j < selfCollisions.n; j++) {
      if (selfCollisions(i, j) != NULL) {
        selfCollisions(i, j) = NULL;
        InitSelfCollisionPair(i, j);
      }
    }
  }
  //init collisions between subchain and existing links
  for (size_t j = 0; j < subchain.links.size(); j++) {
    if (subchain.parents[j] < 0 && link >= 0 && geometry[link] && !geometry[link]->Empty()) {
      //rigidly attached to 'link' -- dont check self collision with link
      for (size_t i = 0; i < norig; i++) {
        if ((int) i != link)
          InitSelfCollisionPair(i, j + norig);
      }
    } else {
      for (size_t i = 0; i < norig; i++) {
        InitSelfCollisionPair(i, j + norig);
      }
    }
  }
  size_t dorig = drivers.size(), jorig = joints.size();
  ArrayUtils::concat(drivers, subchain.drivers);
  ArrayUtils::concat(driverNames, subchain.driverNames);
  ArrayUtils::concat(joints, subchain.joints);
  //update driver and joint numbering
  for (size_t i = 0; i < subchain.joints.size(); i++) {
    if (joints[i + jorig].linkIndex < 0)
      joints[i + jorig].linkIndex = link;
    else
      joints[i + jorig].linkIndex += norig;
    if (joints[i + jorig].baseIndex < 0)
      joints[i + jorig].baseIndex = link;
    else
      joints[i + jorig].baseIndex += norig;
  }
  for (size_t i = 0; i < subchain.drivers.size(); i++) {
    for (size_t j = 0; j < drivers[i + dorig].linkIndices.size(); j++) {
      if (drivers[i + dorig].linkIndices[j] < 0)
        drivers[i + dorig].linkIndices[j] = link;
      else
        drivers[i + dorig].linkIndices[j] += norig;
    }
  }

  //modify names of links and drivers
  if(prefix == NULL) {
    for (size_t k = lstart; k < links.size(); k++) 
      linkNames[k] = linkNames[k];
    for (size_t k = dstart; k < drivers.size(); k++)
      driverNames[k] = driverNames[k];
  }
  else {
    string sprefix(prefix);
    for (size_t k = lstart; k < links.size(); k++) 
      linkNames[k] = sprefix + ":" + linkNames[k];
    for (size_t k = dstart; k < drivers.size(); k++)
      driverNames[k] = sprefix + ":" + driverNames[k];
  }

  //modify sensors 
  for(PropertyMap::const_iterator i=subchain.properties.begin();i!=subchain.properties.end();i++) {
    if(i->first == "sensors") {
      stringstream ss(i->second);
      TiXmlElement e("sensors");
      ss >> e;
      if(!ss) {
        LOG4CXX_WARN(GET_LOGGER(Robot),"Robot::Mount: Warning, mounted robot sensors couldn't be loaded "<<i->second.c_str());
        continue;
      }
      //go through and modify all links
      TiXmlElement* c = e.FirstChildElement();
      while(c != NULL) {
        if(c->Attribute("link")) {
          //TODO: if the link is on the root element 0 or -1, transform Tsensor attribute by T
          int link;
          if(c->QueryIntAttribute("link",&link) == TIXML_SUCCESS) 
            c->SetAttribute("link",lstart+link);
          else {
            //named link
            if(prefix)
              c->SetAttribute("link",(string(prefix)+":"+string(c->Attribute("link"))).c_str());
          }
        }
        else if(c->Attribute("indices")) {
          stringstream ss(c->Attribute("indices"));
          stringstream ssout;
          int index;
          while(ss >> index) 
            ssout << lstart + index <<" ";
          c->SetAttribute("indices",ssout.str().c_str());
        }
        c = c->NextSiblingElement();
      }
      if(properties.count("sensors") > 0) {
        LOG4CXX_INFO(GET_LOGGER(Robot),"Robot::Mount: Adding sensors as children of previous sensors");
        //add sensors onto my sensors
        TiXmlElement emaster("sensors");
        stringstream ss1(properties["sensors"]);
        ss1 >> emaster;
        if(!ss1) {
          LOG4CXX_WARN(GET_LOGGER(Robot),"Robot::Mount: Warning, base robot's sensors couldn't be loaded "<<properties["sensors"]);
          continue;
        }
        TiXmlElement* c = e.FirstChildElement();
        while(c != NULL) {
          emaster.InsertEndChild(*c);
          c = c->NextSiblingElement();
        }
        stringstream ss2;
        ss2 << emaster;
        properties["sensors"] = ss2.str();
      }
      else {
        stringstream ss;
        ss << e;
        properties["sensors"] = ss.str();
      }

    }
    else if(i->first == "controller") {
      LOG4CXX_WARN(GET_LOGGER(Robot),"Robot::Mount: Warning, mounted robot will not preserve controller "<<i->second);
    }
  }
}

bool Robot::DoesJointAffect(int joint, int dof) const {
  switch (joints[joint].type) {
  case RobotJoint::Weld:
  case RobotJoint::Normal:
  case RobotJoint::Spin:
    return joints[joint].linkIndex == dof;
  case RobotJoint::Floating:
  case RobotJoint::FloatingPlanar:
  case RobotJoint::BallAndSocket: {
    int link = joints[joint].linkIndex;
    while (link != joints[joint].baseIndex) {
      if (link == dof)
        return true;
      link = parents[link];
    }
    return false;
  }
  default:
    FatalError("TODO joint type %d",joints[joint].type);
    return false;
  }
}

void Robot::GetJointIndices(int joint, vector<int>& indices) const {
  switch (joints[joint].type) {
  case RobotJoint::Weld:
  case RobotJoint::Normal:
  case RobotJoint::Spin:
    indices.resize(1);
    indices[0] = joints[joint].linkIndex;
    break;
  case RobotJoint::Floating:
  case RobotJoint::FloatingPlanar:
  case RobotJoint::BallAndSocket: {
    indices.resize(0);
    int link = joints[joint].linkIndex;
    while (link != joints[joint].baseIndex) {
      indices.push_back(link);
      link = parents[link];
    }
    reverse(indices.begin(), indices.end());
    break;
  }
  default:
    FatalError("TODO joint type %d",joints[joint].type);
    break;
  }
}

bool Robot::IsPassiveDOF(int dof) const {
  for (size_t i = 0; i < drivers.size(); i++)
    for (size_t j = 0; j < drivers.size(); j++)
      if (drivers[i].linkIndices[j] == dof)
        return false;
  return true;
}

bool Robot::DoesDriverAffect(int driver, int dof) const {
  return drivers[driver].Affects(dof);
}

void Robot::GetDriverIndices(int driver, vector<int>& indices) const {
  indices = drivers[driver].linkIndices;
}

Vector2 Robot::GetDriverLimits(int d) const {
  return Vector2(drivers[d].qmin, drivers[d].qmax);
}

Real Robot::GetDriverValue(int d) const {
  switch (drivers[d].type) {
  case RobotJointDriver::Normal:
  case RobotJointDriver::Translation:
  case RobotJointDriver::Rotation:
    return q(drivers[d].linkIndices[0]);
  case RobotJointDriver::Affine: {
    Real vavg = 0;
    for (size_t i = 0; i < drivers[d].linkIndices.size(); i++) {
      int k = drivers[d].linkIndices[i];
      Assert(k >= 0 && k < q.n);
      Real v = (q(k) - drivers[d].affOffset[i])
          / drivers[d].affScaling[i];
      vavg += v;
    }
    return vavg / drivers[d].linkIndices.size();
  }
  default:
    FatalError("TODO driver type %d",drivers[d].type);
    return 0;
  }
}

void Robot::SetDriverValue(int d, Real value) {
  switch (drivers[d].type) {
  case RobotJointDriver::Normal:
  case RobotJointDriver::Translation:
  case RobotJointDriver::Rotation:
    q(drivers[d].linkIndices[0]) = value;
    break;
  case RobotJointDriver::Affine: {
    for (size_t i = 0; i < drivers[d].linkIndices.size(); i++) {
      int k = drivers[d].linkIndices[i];
      Assert(k >= 0 && k < q.n);
      q(k) = drivers[d].affOffset[i] + value * drivers[d].affScaling[i];
    }
    break;
  }
  default:
    FatalError("TODO driver type %d",drivers[d].type);
  }
}

void Robot::SetJointByTransform(int j, int link, const RigidTransform& Tl) {
  vector<int> indices;
  GetJointIndices(j, indices);
  RigidTransform T0, T;
  if (parents[indices[0]] != -1)
    T0 = links[parents[indices[0]]].T_World * links[indices[0]].T0_Parent;
  else
    T0 = links[indices[0]].T0_Parent;
  T.mulInverseA(T0, Tl);

  switch (joints[j].type) {
  case RobotJoint::Weld:
    FatalError("Can't set a weld joint");
    break;
  case RobotJoint::Normal:
  case RobotJoint::Spin:
    Assert(joints[j].linkIndex == link);
    FatalError("TODO: infer Normal/Spin link parameter from transform");
    break;
  case RobotJoint::BallAndSocket:
    SetJointByOrientation(j, link, Tl.R);
    break;
  case RobotJoint::Floating:
    Assert(joints[j].linkIndex == link);
    {
      //TODO: only know how to do translation then RPY, make this more sophisticated
      Assert(indices.size() == 6);
      int tx = -1, ty = -1, tz = -1;
      int rx = -1, ry = -1, rz = -1;
      for (size_t i = 0; i < indices.size(); i++) {
        int k = indices[i];
        if (links[k].type == RobotLink3D::Prismatic) {
          if (links[k].w == Vector3(1, 0, 0))
            tx = (int) k;
          else if (links[k].w == Vector3(0, 1, 0))
            ty = (int) k;
          else if (links[k].w == Vector3(0, 0, 1))
            tz = (int) k;
        } else {
          if (links[k].w == Vector3(1, 0, 0))
            rx = (int) k;
          else if (links[k].w == Vector3(0, 1, 0))
            ry = (int) k;
          else if (links[k].w == Vector3(0, 0, 1))
            rz = (int) k;
        }
      }
      Assert(tx == indices[0]);
      Assert(ty == indices[1]);
      Assert(tz == indices[2]);
      Assert(rz == indices[3]);
      Assert(ry == indices[4]);
      Assert(rx == indices[5]);
      T.t.get(q(tx), q(ty), q(tz));
      EulerAngleRotation e;
      e.setMatrixZYX(T.R);
      e.get(q(rz), q(ry), q(rx));
    }
    break;
  case RobotJoint::FloatingPlanar:
    Assert(joints[j].linkIndex == link);
    {
      Assert(indices.size() == 3);
      Assert(links[indices[0]].type == RobotLink3D::Prismatic);
      Assert(links[indices[1]].type == RobotLink3D::Prismatic);
      Assert(links[indices[2]].type == RobotLink3D::Revolute);
      q(indices[0]) = links[indices[0]].w.dot(T.t);
      q(indices[1]) = links[indices[1]].w.dot(T.t);
      //rotation
      Vector3 x, y;
      GetCanonicalBasis(links[indices[2]].w, x, y);
      Vector3 desx = -(T.R * y);
      q(indices[2]) = Atan2(desx.y, desx.x);
    }
    break;
  default:
    FatalError("TODO joint type %d",joints[j].type);
    break;
  }
}

void Robot::SetJointByOrientation(int j, int link, const Matrix3& Rl) {
  vector<int> indices;
  GetJointIndices(j, indices);
  Matrix3 R0, R;
  if (parents[indices[0]] != -1)
    R0 = links[parents[indices[0]]].T_World.R
        * links[indices[0]].T0_Parent.R;
  else
    R0 = links[indices[0]].T0_Parent.R;
  R.mulTransposeA(R0, Rl);

  switch (joints[j].type) {
  case RobotJoint::Weld:
    FatalError("Can't set a weld joint");
    break;
  case RobotJoint::Normal:
  case RobotJoint::Spin:
    Assert(joints[j].linkIndex == link);
    FatalError("TODO: infer link parameter from transform");
    break;
  case RobotJoint::Floating:
    Assert(joints[j].linkIndex == link);
    {
      //TODO: only know how to do RPY, make this more sophisticated
      Assert(indices.size() == 6);
      int rx = -1, ry = -1, rz = -1;
      for (size_t i = 0; i < indices.size(); i++) {
        int k = indices[i];
        if (links[k].type == RobotLink3D::Revolute) {
          if (links[k].w == Vector3(1, 0, 0))
            rx = (int) k;
          else if (links[k].w == Vector3(0, 1, 0))
            ry = (int) k;
          else if (links[k].w == Vector3(0, 0, 1))
            rz = (int) k;
        }
      }
      Assert(rz == indices[3]);
      Assert(ry == indices[4]);
      Assert(rx == indices[5]);
      EulerAngleRotation e;
      e.setMatrixZYX(R);
      e.get(q(rz), q(ry), q(rx));
    }
    break;
  case RobotJoint::FloatingPlanar:
    Assert(joints[j].linkIndex == link);
    {
      Assert(indices.size() == 3);
      Assert(links[indices[0]].type == RobotLink3D::Prismatic);
      Assert(links[indices[1]].type == RobotLink3D::Prismatic);
      Assert(links[indices[2]].type == RobotLink3D::Revolute);
      //rotation
      //rotation
      Vector3 x, y;
      GetCanonicalBasis(links[indices[2]].w, x, y);
      Vector3 desx = -(R * y);
      q(indices[2]) = Atan2(desx.y, desx.x);
    }
    break;
  case RobotJoint::BallAndSocket:
    Assert(joints[j].linkIndex == link);
    {
      //TODO: only know how to do RPY, make this more sophisticated
      Assert(indices.size() == 3);
      int rx = -1, ry = -1, rz = -1;
      for (size_t i = 0; i < indices.size(); i++) {
        int k = indices[i];
        assert(links[k].type == RobotLink3D::Revolute);
        if (links[k].w == Vector3(1, 0, 0))
          rx = (int) k;
        else if (links[k].w == Vector3(0, 1, 0))
          ry = (int) k;
        else if (links[k].w == Vector3(0, 0, 1))
          rz = (int) k;
      }
      Assert(rz == indices[0]);
      Assert(ry == indices[1]);
      Assert(rx == indices[2]);
      EulerAngleRotation e;
      e.setMatrixZYX(R);
      e.get(q(rz), q(ry), q(rx));
    }
    break;
  default:
    FatalError("TODO joint type %d",joints[j].type);
    break;
  }
}

Real Robot::GetDriverVelocity(int d) const {
  switch (drivers[d].type) {
  case RobotJointDriver::Normal:
  case RobotJointDriver::Translation:
  case RobotJointDriver::Rotation:
    return dq(drivers[d].linkIndices[0]);
  case RobotJointDriver::Affine: {
    Real vavg = 0;
    for (size_t i = 0; i < drivers[d].linkIndices.size(); i++) {
      int k = drivers[d].linkIndices[i];
      Real v = dq[k] / drivers[d].affScaling[i];
      vavg += v;
    }
    return vavg / drivers[d].linkIndices.size();
  }
  default:
    FatalError("TODO driver type %d",drivers[d].type);
    return 0;
  }
}

void Robot::SetDriverVelocity(int d, Real value) {
  switch (drivers[d].type) {
  case RobotJointDriver::Normal:
    Assert(drivers[d].linkIndices.size() == 1);
    dq[drivers[d].linkIndices[0]] = value;
    break;
  case RobotJointDriver::Translation:
  case RobotJointDriver::Rotation:
    Assert(drivers[d].linkIndices.size() == 2);
    dq[drivers[d].linkIndices[0]] = value;
    break;
  case RobotJointDriver::Affine: {
    for (size_t j = 0; j < drivers[d].linkIndices.size(); j++)
      dq[drivers[d].linkIndices[j]] = value * drivers[d].affScaling[j];
  }
    break;
  default:
    FatalError("TODO driver type %d",drivers[d].type);
    break;
  }
}

//at the current euler angles theta (in zyx form), converts the angular
//velocity w to euler angle velocity dtheta 
void AngVelToEulerAngles(const Vector3& theta, const Vector3& w,
    Vector3& dtheta) {
  bool res = EulerAngleDerivative(theta, w, 2, 1, 0, dtheta);
  if (!res) {
        LOG4CXX_ERROR(GET_LOGGER(Robot),
        "AngVelToEulerAngles: Warning, at singularity of euler angle parameterization, derivative set to zero");
    dtheta.setZero();
    return;
  }
  if (!IsFinite(theta)) {
        LOG4CXX_ERROR(GET_LOGGER(Robot),
        "AngVelToEulerAngles: Warning, euler angles not finite");
    dtheta.setZero();
    return;
  }
  if (!IsFinite(w)) {
        LOG4CXX_ERROR(GET_LOGGER(Robot),
        "AngVelToEulerAngles: Warning, angular velocity is not finite");
    dtheta.setZero();
    return;
  }
  Assert(IsFinite(dtheta));
  Assert(res == true);
}

void Robot::SetJointVelocityByMoment(int j, int link, const Vector3& w,
    const Vector3& v) {
  switch (joints[j].type) {
  case RobotJoint::Weld:
    FatalError("Can't set a weld joint");
    break;
  case RobotJoint::Normal:
  case RobotJoint::Spin:
    Assert(joints[j].linkIndex == link);
    FatalError("TODO: infer Normal/Spin link velocity from twist");
    break;
  case RobotJoint::Floating: {
    //TODO: only know how to do translation then RPY, make this more sophisticated
    //TODO: only know how to do identity root transforms
    vector<int> indices;
    GetJointIndices(j, indices);
    Assert(joints[j].baseIndex == -1);
    Assert(joints[j].linkIndex == link);
    Assert(indices.size() == 6);
    int tx = -1, ty = -1, tz = -1;
    int rx = -1, ry = -1, rz = -1;
    for (size_t i = 0; i < indices.size(); i++) {
      int k = indices[i];
      if (links[k].type == RobotLink3D::Prismatic) {
        if (links[k].w == Vector3(1, 0, 0))
          tx = (int) k;
        else if (links[k].w == Vector3(0, 1, 0))
          ty = (int) k;
        else if (links[k].w == Vector3(0, 0, 1))
          tz = (int) k;
      } else {
        if (links[k].w == Vector3(1, 0, 0))
          rx = (int) k;
        else if (links[k].w == Vector3(0, 1, 0))
          ry = (int) k;
        else if (links[k].w == Vector3(0, 0, 1))
          rz = (int) k;
      }
    }
    Assert(tx == indices[0]);
    Assert(ty == indices[1]);
    Assert(tz == indices[2]);
    Assert(rz == indices[3]);
    Assert(ry == indices[4]);
    Assert(rx == indices[5]);
    v.get(dq(tx), dq(ty), dq(tz));
    Vector3 theta(q(rz), q(ry), q(rx)), dtheta;
    AngVelToEulerAngles(theta, w, dtheta);
    dtheta.get(dq(rz), dq(ry), dq(rx));
  }
    break;
  case RobotJoint::FloatingPlanar: {
    vector<int> indices;
    GetJointIndices(j, indices);
    Assert(joints[j].baseIndex == -1);
    Assert(joints[j].linkIndex == link);
    Assert(indices.size() == 3);
    Assert(links[indices[0]].type == RobotLink3D::Prismatic);
    Assert(links[indices[1]].type == RobotLink3D::Prismatic);
    Assert(links[indices[2]].type == RobotLink3D::Revolute);
    dq(indices[0]) = v.dot(links[indices[0]].w);
    dq(indices[1]) = v.dot(links[indices[1]].w);
    dq(indices[2]) = w.dot(links[indices[2]].w);
  }
    break;
  case RobotJoint::BallAndSocket: {
    //TODO: only know how to do RPY, make this more sophisticated
    vector<int> indices;
    GetJointIndices(j, indices);
    Assert(joints[j].baseIndex == -1);
    Assert(joints[j].linkIndex == link);
    Assert(indices.size() == 3);
    int rx = -1, ry = -1, rz = -1;
    for (size_t i = 0; i < indices.size(); i++) {
      int k = indices[i];
      assert(links[k].type == RobotLink3D::Revolute);
      if (links[k].w == Vector3(1, 0, 0))
        rx = (int) k;
      else if (links[k].w == Vector3(0, 1, 0))
        ry = (int) k;
      else if (links[k].w == Vector3(0, 0, 1))
        rz = (int) k;
    }
    Assert(rz == indices[0]);
    Assert(ry == indices[1]);
    Assert(rx == indices[2]);
    Vector3 theta(q(rz), q(ry), q(rx)), dtheta;
    AngVelToEulerAngles(theta, w, dtheta);
    dtheta.get(dq(rz), dq(ry), dq(rx));
  }
    break;
  default:
    FatalError("TODO joint type %d",joints[j].type);
    break;
  }
}

void Robot::GetDriverJacobian(int d, Vector& J) {
  J.resize(links.size(), Zero);
  switch (drivers[d].type) {
  case RobotJointDriver::Normal:
  case RobotJointDriver::Rotation:
  case RobotJointDriver::Translation:
    J(drivers[d].linkIndices[0]) = 1.0;
    break;
  case RobotJointDriver::Affine: {
    for (size_t j = 0; j < drivers[d].linkIndices.size(); j++)
      J(drivers[d].linkIndices[j]) = drivers[d].affScaling[j];
  }
    break;
  default:
    FatalError("TODO driver type %d",drivers[d].type);
    break;
  }
}

bool Robot::LoadURDF(const char* fn)
{
  string localfile = MakeURLLocal(fn);
  if(localfile.empty()) return false;
  string path = GetFilePath(fn);

  //Get content from the Willow Garage parser
  std::shared_ptr<urdf::ModelInterface> parser = urdf::parseURDF(localfile);
  if(!parser) {
    LOG4CXX_ERROR(GET_LOGGER(URDFParser),"Robot::LoadURDF: error parsing XML");
    return false;
  }
  std::shared_ptr<urdf::Link> root_link = parser->root_link_;
  if (!root_link) {
    LOG4CXX_ERROR(GET_LOGGER(URDFParser),"Robot::LoadURDF: Root link is NULL");
    return false;
  }

  //parse Klamp't extras
  //format:
  //- <robot>
  //  - <klampt [default_mass:float] [default_inertia:H] [default_acc_max:float] [use_vis_geom:bool] [flip_yz:bool] [world_frame:string] [package_root:string]>
  //    default_mass: sets the mass of links for which URDF has no mass.
  //    default_inertia can be either a float (diagonal of matrix),
  //      3-vector (diagonal of matrix), or 9-vector (entries of 3x3
  //      inertia matrix)
  //    default_acc_max: default acceleration limit (default: 100)
  //    use_vis_geom: true if you want to use visualization geometry
  //      instead of collision geometry
  //    flip_yz: true if you want to swap the y-z components when
  //      importing meshes.
  //    world_frame: set this to change the name of the world frame
  //      for fixed-base robots (default: "world").
  //    freeze_root_link: set this if you want the root link to be
  //      frozen in space (default: false).
  //    package_root: set this to determine relative location of package:// header
  //    - <link name:string [physical:bool] [servoP:float] [servoI:float] [servoD:float] [dryFriction:float] [viscousFriction:float]>
  //      name: identifies a link
  //      physical: sets whether it is considered a physical
  //        or virtual link.  Virtual links have mass set to 0.
  //      accMax: sets the acceleration limit
  //      servoP,servoI,servoD,dryFriction,viscousFriction: 
  //        optionally set the driver parameters for the given link
  //    - <noselfcollision pairs:"a1 b1 ... ak bk" >
  //      sets collision pairs for which self collision should not be
  //      tested
  //    - <selfcollision pairs:"a1 b1 ... ak bk" >
  //      sets collision pairs for which self collision should be
  //      tested
  //    - <mount link:{int or str} file:string transform:{12 floats giving rotation matrix} [as:string] >
  //      mounts a geometry or another URDF / .rob file to a link.
  double default_mass = 0.0001;
  Matrix3 default_inertia; default_inertia.setIdentity(); default_inertia *= 1e-8;  
  double default_acc_max = 10;
  int freezeRootLink = 0;
  string worldFrame = "world";
  vector<pair<string, string> > selfCollision;
  vector<pair<string, string> > noSelfCollision;
  map<string,bool> virtualLinks;
  map<string,Real> kP,kI,kD,dryFriction,viscousFriction,customAccMax;
  vector<string> mountLinks;
  vector<string> mountFiles;
  vector<RigidTransform> mountT;
  vector<string> mountNames;
  TiXmlDocument xml_doc;
  bool loaded=xml_doc.LoadFile(fn);
  if(!loaded) {
    LOG4CXX_ERROR(GET_LOGGER(URDFParser),"Strange, unable to re-open URDF robot file "<<fn);
  }
  TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
  TiXmlElement *klampt_xml = NULL;
  if(robot_xml)
    klampt_xml = robot_xml->FirstChildElement("klampt");
  if(klampt_xml) {
    int use_vis_geom,flip_yz;
    if(klampt_xml->QueryValueAttribute("use_vis_geom",&use_vis_geom)==TIXML_SUCCESS) {
      URDFConverter::useVisGeom = (use_vis_geom != 0);
    }
    if(klampt_xml->QueryValueAttribute("flip_yz",&flip_yz)==TIXML_SUCCESS) {
      URDFConverter::flipYZ = (flip_yz != 0);
    }
    if(klampt_xml->QueryValueAttribute("freeze_root_link",&freezeRootLink)==TIXML_SUCCESS) {
      //value has been read in already...
    }
    else freezeRootLink = false;

    if(klampt_xml->Attribute("world_frame") != 0) {
      worldFrame = klampt_xml->Attribute("world_frame");
    }
    if(klampt_xml->Attribute("package_root") != 0) {
      string root_path = klampt_xml->Attribute("package_root");
      //TODO: windows root references?
      if(root_path[0]=='/' || path.empty())
        URDFConverter::packageRootPath = root_path;
      else 
        URDFConverter::packageRootPath = path+"/"+root_path;
    }
    if(klampt_xml->QueryValueAttribute("default_acc_max",&default_acc_max)!=TIXML_SUCCESS) {
      //pass
    }
    if(klampt_xml->QueryValueAttribute("default_mass",&default_mass)!=TIXML_SUCCESS) {
      //pass
    }
    Real H;
    Vector3 H3;
    if(klampt_xml->QueryValueAttribute("default_inertia",&default_inertia)==TIXML_SUCCESS) {
      //pass
    }
    else if(klampt_xml->QueryValueAttribute("default_inertia",&H3)==TIXML_SUCCESS) {
      default_inertia.setIdentity();
      default_inertia(0,0) = H3.x;
      default_inertia(1,1) = H3.y;
      default_inertia(2,1) = H3.z;
    }
    else if(klampt_xml->QueryValueAttribute("default_inertia",&H)==TIXML_SUCCESS) {
      default_inertia.setIdentity();
      default_inertia *= H;
    }
    else {
      default_inertia.setIdentity(); 
      default_inertia *= 1e-8; 
    }

    //check for sensors and controller attribute / children
    const char* props[2] = {"sensors","controller"};
    for (int i=0;i<2;i++) {
      const char* prop = props[i];
      if(klampt_xml->Attribute(prop) != NULL) {
        const char* value = klampt_xml->Attribute(prop);
        //load from file
        stringstream ss(value);
        string file;
        SafeInputString(ss,file);
        const char* ext = FileExtension(file.c_str());
        if(ext && 0==strcmp(ext,"xml")) {
          //prepend the robot path
          string fn = ResolveFileReference(path,file);
          fn = MakeURLLocal(fn);
          if(fn.empty()) {
            LOG4CXX_ERROR(GET_LOGGER(URDFParser),"     Error loading file "<<prop<<" from file "<<fn);
            return false;
          }
          if(!GetFileContents(fn.c_str(),properties[prop])) {
            LOG4CXX_ERROR(GET_LOGGER(URDFParser),"     Unable to read "<<prop<<" property from file "<<fn);
            return false;
          }
        }
        else {
          LOG4CXX_ERROR(GET_LOGGER(URDFParser),"<klampt> XML tag \""<<prop<<"\" needs to be an external XML file");
          properties[prop] = value;
        }
      }
      else {
        //or <sensors> / <controller> tags can be placed under the <klampt> tag
        TiXmlElement* c = klampt_xml->FirstChildElement(prop);
        if(c != NULL) {
          stringstream ss;
          ss<<*c;
          properties[prop] = ss.str();
        } 
      }
    }
    TiXmlElement* e = klampt_xml->FirstChildElement("link");
    while(e != NULL) {
      string name;
      int physicalLink;
      Real temp;
      if(e->QueryValueAttribute("name",&name) != TIXML_SUCCESS) {
        LOG4CXX_ERROR(GET_LOGGER(URDFParser),"Warning: Xml element robot/klampt/link doesnt have name attribute");
        e = e->NextSiblingElement("link");
        continue;
      }
      if(e->QueryValueAttribute("physical",&physicalLink) == TIXML_SUCCESS && physicalLink==0) {
        virtualLinks[name] = true;
      }
      if(e->QueryValueAttribute("servoP",&temp) == TIXML_SUCCESS) {
        kP[name] = temp;
      }
      if(e->QueryValueAttribute("servoD",&temp) == TIXML_SUCCESS) {
        kD[name] = temp;
      }
      if(e->QueryValueAttribute("servoI",&temp) == TIXML_SUCCESS) {
        kI[name] = temp;
      }
      if(e->QueryValueAttribute("dryFriction",&temp) == TIXML_SUCCESS) {
        dryFriction[name] = temp;
      }
      if(e->QueryValueAttribute("viscousFriction",&temp) == TIXML_SUCCESS) {
        viscousFriction[name] = temp;
      }
      if(e->QueryValueAttribute("accMax",&temp) == TIXML_SUCCESS) {
        customAccMax[name] = temp;
      }
      e = e->NextSiblingElement("link");
    }

    e = klampt_xml->FirstChildElement("selfcollision");
    while(e != NULL) {
      if(e->Attribute("pairs")!=NULL) {
        stringstream ss(e->Attribute("pairs"));
        pair<string, string> ptemp;
        while (SafeInputString(ss,ptemp.first) && SafeInputString(ss,ptemp.second))
          selfCollision.push_back(ptemp);
      }
      else if(e->Attribute("group1")!=NULL && e->Attribute("group2")!=NULL) {
        vector<string> group1,group2;
        string stemp;
        stringstream ss1(e->Attribute("group1"));
        stringstream ss2(e->Attribute("group2"));
        while (SafeInputString(ss1,stemp))
          group1.push_back(stemp);
        while (SafeInputString(ss2,stemp))
          group2.push_back(stemp);
        for(size_t i=0;i<group1.size();i++) 
          for(size_t j=0;j<group2.size();j++) 
            selfCollision.push_back(pair<string,string>(group1[i],group2[j]));
      }
      else {
        LOG4CXX_ERROR(GET_LOGGER(URDFParser),"Error, robot/klampt/selfcollision does not contain pairs, or group1 and group2 attributes");
      }       


      e = e->NextSiblingElement("selfcollision");
    }
    e = klampt_xml->FirstChildElement("noselfcollision");
    while(e != NULL) {
      if(e->Attribute("pairs")!=NULL) {
        stringstream ss(e->Attribute("pairs"));
        pair<string, string> ptemp;
        while (SafeInputString(ss,ptemp.first) && SafeInputString(ss,ptemp.second))
          noSelfCollision.push_back(ptemp);
      }
      else if(e->Attribute("group1")!=NULL && e->Attribute("group2")!=NULL) {
        vector<string> group1,group2;
        string stemp;
        stringstream ss1(e->Attribute("group1"));
        stringstream ss2(e->Attribute("group2"));
        while (SafeInputString(ss1,stemp))
          group1.push_back(stemp);
        while (SafeInputString(ss2,stemp))
          group2.push_back(stemp);
        for(size_t i=0;i<group1.size();i++) 
          for(size_t j=0;j<group2.size();j++) 
            noSelfCollision.push_back(pair<string,string>(group1[i],group2[j]));
      }
      else {
        LOG4CXX_ERROR(GET_LOGGER(URDFParser),"Error, robot/klampt/noselfcollision does not contain pairs, or group1 and group2 attributes");
      }       

      e = e->NextSiblingElement("noselfcollision");
    }

    e = klampt_xml->FirstChildElement("mount");
    while(e != NULL) {
      if(e->Attribute("link")!=NULL) {
        string link = e->Attribute("link");
        string prefix = "";
        if(e->Attribute("file")==NULL) {
          LOG4CXX_ERROR(GET_LOGGER(URDFParser),"Error, robot/klampt/mount does not contain a 'file' attribute");
          return false;
        }
        string file=e->Attribute("file");
        RigidTransform T;
        T.setIdentity();
        if(e->Attribute("transform")) {
          stringstream ss(e->Attribute("transform"));
          ss>>T;
          if(!ss) {
            LOG4CXX_ERROR(GET_LOGGER(URDFParser),"Error, robot/klampt/mount has an invalid 'transform' attribute");
            return false;
          }
        }
        if(e->Attribute("prefix")) {
          prefix = e->Attribute("prefix");
        }
        if(e->Attribute("as")) {
          prefix = e->Attribute("as");
        }
        mountLinks.push_back(link);
        mountFiles.push_back(file);
        mountT.push_back(T);
        mountNames.push_back(prefix);
      }
      else {
        LOG4CXX_ERROR(GET_LOGGER(URDFParser),"Error, robot/klampt/mount does not contain a 'link' attribute'");
        return false;
      }
      e = e->NextSiblingElement("mount");
    }
  }

  //fixed-base robots have the root link "world", floating-base robots
  //do not.
  bool floating = (root_link->name != worldFrame);
  int links_size, joints_size;
  if(floating) {
    //links_size: URDF 36, ROB 41 with 5 extra DOFS for base
    links_size = (int)parser->links_.size() + 5;
    //joints_size: URDF 35, ROB 36 with 1 extra for base
    joints_size = (int)parser->joints_.size() + 1;
    //drivers_size: should be joints_size - 1 in ROB file

    if (joints_size != links_size - 5) {
      LOG4CXX_ERROR(GET_LOGGER(URDFParser), "joint size:" << joints_size << " and link size:" << links_size
     << " do not match for floating-base robot!" );
      for(auto i=parser->links_.begin();i!=parser->links_.end();i++)
        if(!i->second->parent_joint) {
          LOG4CXX_INFO(GET_LOGGER(URDFParser), "URDF link "<<i->second->name <<" is a root");
        }
      for(auto i=parser->joints_.begin();i!=parser->joints_.end();i++) {
        if(parser->links_.count(i->second->child_link_name) == 0) {
          LOG4CXX_INFO(GET_LOGGER(URDFParser), "URDF joint "<<i->first<<" child link "<<i->second->child_link_name <<" does not exist");
        }
        else {
          auto c = parser->links_[i->second->child_link_name];
          if(c->parent_joint != i->second)
            LOG4CXX_INFO(GET_LOGGER(URDFParser), "URDF link "<<i->second->child_link_name <<" has multiple parents?");
        }
      }
      return false;
    }

    if(freezeRootLink)
      joints_size += 5;
  }
  else {
    links_size = (int)parser->links_.size() - 1;
    joints_size = (int)parser->joints_.size();
    if(joints_size != links_size) {
      LOG4CXX_ERROR(GET_LOGGER(URDFParser), "joint size:" << joints_size << " and link size:" << links_size
     << " do not match for fixed-base robot!" );
      for(auto i=parser->links_.begin();i!=parser->links_.end();i++)
        if(!i->second->parent_joint) {
          LOG4CXX_INFO(GET_LOGGER(URDFParser), "URDF link "<<i->second->name <<" is a root");
        }
      for(auto i=parser->joints_.begin();i!=parser->joints_.end();i++) {
        if(parser->links_.count(i->second->child_link_name) == 0) {
          LOG4CXX_INFO(GET_LOGGER(URDFParser), "URDF joint "<<i->first<<" child link "<<i->second->child_link_name <<" does not exist");
        }
        else {
          auto c = parser->links_[i->second->child_link_name];
          if(c->parent_joint != i->second)
            LOG4CXX_INFO(GET_LOGGER(URDFParser), "URDF link "<<i->second->child_link_name <<" has multiple parents?");
        }
      }
      return false;
    }
  }

  LOG4CXX_INFO(GET_LOGGER(URDFParser), "Link size: " << links_size << "");
  LOG4CXX_INFO(GET_LOGGER(URDFParser), "Joint size: " << joints_size << "");

  //Feed the information from parser to required vectors in ROB format
  //The following vectors have the same dimension of the links vector
  Initialize(links_size);

  this->links.resize(links_size);
  this->parents.resize(links_size);
  this->linkNames.resize(links_size);
  this->geometry.resize(links_size);
  this->geomManagers.resize(links_size);
  this->geomFiles.resize(links_size);
  this->q.resize(links_size);
  this->q.setZero();
  this->qMin.resize(links_size);
  this->qMax.resize(links_size);
  this->accMax.resize(links_size);
  this->torqueMax.resize(links_size);
  this->powerMax.resize(links_size); //TODO
  this->velMax.resize(links_size);
  this->velMin.resize(links_size);

  //The following vectors have different dimensions
  this->joints.resize(joints_size);


  //floating base, the first 6 degrees
  if(floating) {
    for (int i = 0; i < 6; i++) {
      this->parents[i] = i - 1;
      stringstream ss;
      ss << i;
      string tmp;
      ss >> tmp;
      this->linkNames[i] = "base" + tmp;
      
      this->accMax[i] = Inf;
      this->qMax[i] = Inf;
      this->qMin[i] = -Inf;
      this->velMax[i] = Inf;
      this->velMin[i] = -Inf;
      this->torqueMax[i] = 0;
      this->powerMax[i] = Inf;
    }
    for (int i = 0; i < 3; i++)
      this->links[i].type = RobotLink3D::Prismatic;
    for (int i = 3; i < 6; i++)
      this->links[i].type = RobotLink3D::Revolute;

    //w: axis for the link movement in link's local frame
    this->links[0].w.set(1, 0, 0);
    this->links[1].w.set(0, 1, 0);
    this->links[2].w.set(0, 0, 1);
    this->links[3].w.set(0, 0, 1);
    this->links[4].w.set(0, 1, 0);
    this->links[5].w.set(1, 0, 0);
    for (int i = 0; i < 5; i++) {
      this->links[i].com.setZero();
      this->links[i].mass = 0;
      this->links[i].inertia.setZero();
      this->links[i].T0_Parent.setIdentity();
    }

    //floating base joint
    if(!freezeRootLink) {
      this->joints[0].type = RobotJoint::Floating;
      this->joints[0].linkIndex = 5;
      this->joints[0].baseIndex = -1;
    }
    else {
      //freezeRootLink = true, so freeze all those links
      for(int i=0;i<6;i++) {
        this->joints[i].type =  RobotJoint::Weld;
        this->joints[i].linkIndex = i;
        this->joints[i].baseIndex = -1;
      }
    }
  }
  else {
    //fixed base
    this->joints[0].type =  RobotJoint::Weld;
    this->joints[0].linkIndex = 0;
    this->joints[0].baseIndex = -1;
  }

  URDFLinkNode rootLinkNode(root_link, 0, -1);
  vector<URDFLinkNode> linkNodes;
  URDFConverter::DFSLinkTree(rootLinkNode, linkNodes);

  vector<std::shared_ptr<urdf::Joint> > urdfJoints;
  for (std::map<std::string, std::shared_ptr<urdf::Joint> >::iterator it =
      parser->joints_.begin(); it != parser->joints_.end(); ++it) {
    std::shared_ptr<urdf::Joint> joint = it->second;
    urdfJoints.push_back(joint);
  }
  URDFConverter::setJointforNodes(urdfJoints, linkNodes);
  URDFConverter::processTParentTransformations(linkNodes);

  vector<int> affineJointParents(links_size,-1);
  vector<double> affineJointOffsets(links_size,0);
  vector<double> affineJointScales(links_size,1);  
  size_t start = (floating ? 0 : 1);
  for (size_t i = start; i < linkNodes.size(); i++) {
    URDFLinkNode* linkNode = &linkNodes[i];
    int link_index = linkNode->index;
    int parent_index = linkNode->index_parent;
    if(floating) {
      link_index += 5;
      parent_index += 5;
    }
    else {
      link_index -= 1;
      parent_index -= 1;
    }

    this->linkNames[link_index] = linkNode->link->name; 
    this->parents[link_index] = parent_index;

    this->links[link_index].type = RobotLink3D::Revolute; //Check correctness
    this->links[link_index].T0_Parent.setIdentity();
    if (i > 0)
      this->links[link_index].w.set(linkNode->axis);
    if(this->links[link_index].w.norm() < 0.1){
      LOG4CXX_ERROR(GET_LOGGER(URDFParser),"Axis error: "<<linkNames[link_index]<<";"<<this->links[link_index].w);
    }

    //If have inertia specified, then, use the specified inertia
    if(virtualLinks.count(linkNode->link->name)!=0) {
        //assume its a virtual link
        this->links[link_index].com = Vector3(0, 0, 0);
        this->links[link_index].mass = 0;
        this->links[link_index].inertia.setZero();
    }
    else {
      if (linkNode->link->inertial) {
        this->links[link_index].com = linkNode->T_link_to_inertia.t;
        this->links[link_index].mass = linkNode->link->inertial->mass; //done
        Matrix3 ori_inertia = URDFConverter::convertInertial(
                   *linkNode->link->inertial);
        this->links[link_index].inertia.mul(
              linkNode->T_link_to_inertia_inverse.R, ori_inertia * transpose(linkNode->T_link_to_inertia_inverse.R));
      }
    //Otherwise, set it to default value
      else {
      this->links[link_index].com = Vector3(0, 0, 0);
      this->links[link_index].mass = default_mass;
      this->links[link_index].inertia = default_inertia;
      }
    }

    //At first, set them to be default Inf value; it will be modified in later steps.
    this->accMax[link_index] = Inf;
    this->qMax[link_index] = Inf;
    this->qMin[link_index] = -Inf;
    this->velMax[link_index] = Inf;
    this->velMin[link_index] = -Inf;
    this->torqueMax[link_index] = Inf;
    this->powerMax[link_index] = Inf;
    if(link_index == 5 && floating) {
      this->torqueMax[link_index] = 0;
      this->powerMax[link_index] = 0;
    }
    //set joint
    urdf::Joint* joint = linkNode->joint;
    if (joint) {
      int joint_index = link_index;
      if(floating) {
        if(!freezeRootLink) joint_index -= 5;
      }
      this->joints[joint_index].type = URDFConverter::jointType_URDF2ROB(
          joint->type); //done. finish function
      if (joint->type == urdf::Joint::PRISMATIC){
        this->links[link_index].type = RobotLink3D::Prismatic;
      }
      this->joints[joint_index].linkIndex = link_index; //done

      this->links[link_index].T0_Parent.set(linkNode->T_parent);

      if (joint->limits) {
        this->qMax[link_index] = joint->limits->upper;
        this->qMin[link_index] = joint->limits->lower;
        this->velMax[link_index] = joint->limits->velocity;
        this->velMin[link_index] = -joint->limits->velocity;
        this->torqueMax[link_index] = joint->limits->effort; //TODO: in URDF, no explicit value specified, effort has unit n*m,
      }
      if(customAccMax.count(linkNode->link->name) != 0)
        this->accMax[link_index] = customAccMax[linkNode->link->name];
      else
        this->accMax[link_index] = default_acc_max;
      if(this->joints[joint_index].type == RobotJoint::Spin) {
          this->qMax[link_index] = Inf;
          this->qMin[link_index] = -Inf;
      }
      if(this->joints[joint_index].type == RobotJoint::Weld){
        qMin[link_index] = 0;
        qMax[link_index] = 0;
        velMin[link_index] = 0;
        velMax[link_index] = 0;
        accMax[link_index] = 0;
        //torque max will be infinite...
        //torqueMax[link_index] = 0;
      }
      //parse mimic joints
      if(joint->mimic) {
        if(parser->joints_.count(joint->mimic->joint_name)) {
          auto joint_parent = parser->joints_[joint->mimic->joint_name];
          affineJointParents[link_index] = LinkIndex(joint_parent->child_link_name.c_str());
          if(affineJointParents[link_index] < 0)
            LOG4CXX_WARN(GET_LOGGER(URDFParser), "Couldn't load mimic joint "<<joint->name);
          affineJointScales[link_index] = joint->mimic->multiplier;
          affineJointOffsets[link_index] = joint->mimic->offset;
        }
        else {
          LOG4CXX_WARN(GET_LOGGER(URDFParser), "Couldn't load mimic joint "<<joint->name);
        }
      }
      if (this->joints[joint_index].type == RobotJoint::Normal
          || this->joints[joint_index].type == RobotJoint::Spin) {
        int linkI = this->joints[joint_index].linkIndex;
        RobotJointDriver driver;
        driverNames.push_back(linkNames[linkI]);
        driver.type = RobotJointDriver::Normal;
        driver.linkIndices.push_back(linkI);
        driver.qmin = qMin(linkI);
        driver.qmax = qMax(linkI);
        driver.vmin = velMin(linkI);
        driver.vmax = velMax(linkI);
        driver.tmin = -torqueMax(linkI);
        driver.tmax = torqueMax(linkI);
        driver.amin = -accMax(linkI);
        driver.amax = accMax(linkI);
        driver.servoP = 100;
        driver.servoI = 0;
        driver.servoD = 10;
        driver.dryFriction = 0;
        driver.viscousFriction = 0;
        if (joint->dynamics) {
          driver.dryFriction = joint->dynamics->friction;
          driver.viscousFriction = joint->dynamics->damping;
        }
        if(kP.count(linkNode->link->name) != 0)
          driver.servoP = kP[linkNode->link->name];
        if(kD.count(linkNode->link->name) != 0)
          driver.servoD = kD[linkNode->link->name];
        if(kI.count(linkNode->link->name) != 0)
          driver.servoI = kI[linkNode->link->name];
        if(dryFriction.count(linkNode->link->name) != 0)
          driver.dryFriction = dryFriction[linkNode->link->name];
        if(viscousFriction.count(linkNode->link->name) != 0)
          driver.viscousFriction = viscousFriction[linkNode->link->name];
        drivers.push_back(driver);
      }
    }
  }
  //compress the affine joints into single drivers
  for(size_t i=0;i<affineJointParents.size();i++) {
    if(affineJointParents[i] >= 0) {
      int p=affineJointParents[i];
      if(affineJointParents[p] >= 0) {
        affineJointParents[i] = affineJointParents[p];
        affineJointOffsets[i] += affineJointScales[i]*affineJointOffsets[p];
        affineJointScales[i] *= affineJointScales[p];
      }
    }
  }
  //delete normal drivers
  map<int,int> linkToDriver;
  size_t i=0;
  while(i<drivers.size()) {
    linkToDriver[drivers[i].linkIndices[0]] = i;
    if(affineJointParents[drivers[i].linkIndices[0]] >= 0) { 
      drivers.erase(drivers.begin()+i);
      driverNames.erase(driverNames.begin()+i);
    }
    else
      i++;
  }
  //now convert parents to affine drivers
  for(size_t i=0;i<affineJointParents.size();i++) {
    if(affineJointParents[i] >= 0) {
      Assert(linkToDriver.count(affineJointParents[i]) > 0);
      int d=linkToDriver[affineJointParents[i]];
      drivers[d].type = RobotJointDriver::Affine;
      if(drivers[d].affScaling.empty()) {
        drivers[d].affScaling.push_back(1);
        drivers[d].affOffset.push_back(0);
      }
      drivers[d].linkIndices.push_back(i);
      drivers[d].affScaling.push_back(affineJointScales[i]);
      drivers[d].affOffset.push_back(affineJointOffsets[i]);
    }
  }

  //double check links are specified properly
  for(auto kpi: kP) {
    if(LinkIndex(kpi.first.c_str()) < 0)
      LOG4CXX_WARN(GET_LOGGER(URDFParser),"Invalid <klampt><link> element named "<<kpi.first);
  }
  
  UpdateFrames();
  for (size_t i = start; i < linkNodes.size(); i++) {
    URDFLinkNode* linkNode = &linkNodes[i];
    int link_index = linkNode->index;
    if(floating) link_index += 5;
    else link_index -= 1;

    //geometry
    if (!linkNode->geomName.empty() && !Robot::disableGeometryLoading) {
      string fn;
      geomFiles[link_index] = linkNode->geomName;
      fn = ResolveFileReference(path,linkNode->geomName);
      if(FileUtils::Exists(fn.c_str())) {
        if (!LoadGeometry(link_index, fn.c_str())) {
          LOG4CXX_ERROR(GET_LOGGER(URDFParser), "Failed loading geometry " << linkNode->geomName << " for link " << link_index << "");
          //LOG4CXX_INFO
          LOG4CXX_ERROR(GET_LOGGER(URDFParser), "Temporarily ignoring error...");
          //return false;
        }
      }
      else if(FileUtils::Exists(geomFiles[link_index].c_str())) {
        if (!LoadGeometry(link_index, geomFiles[link_index].c_str())) {
          LOG4CXX_ERROR(GET_LOGGER(URDFParser), "Failed loading geometry " << linkNode->geomName  << " for link " << link_index << "");
          //TEMP
          LOG4CXX_INFO(GET_LOGGER(URDFParser), "Temporarily ignoring error...");
          //return false;
        }
      }
      else {
        localfile = MakeURLLocal(fn);
        if(localfile == fn) {
          LOG4CXX_ERROR(GET_LOGGER(URDFParser), "Could not load geometry " << linkNode->geomName <<", in relative or absolute paths");
          //TEMP
          LOG4CXX_INFO(GET_LOGGER(URDFParser), "Temporarily ignoring error...");
          //return false;
        }
        else {
          //try loiading from url
          if(!LoadGeometry(link_index,fn.c_str())) {
            LOG4CXX_ERROR(GET_LOGGER(URDFParser), "Failed loading geometry " << linkNode->geomName << " for link " << link_index << "");
            //TEMP
            LOG4CXX_INFO(GET_LOGGER(URDFParser), "Temporarily ignoring error...");
          }
        }
      }
    }
    if(!linkNode->geomData.Empty()) {
      if(link_index >= (int)geomManagers.size())
        geomManagers.resize(geometry.size());
      //*geomManagers[link_index] = geom;
      //TEMP: convert primitives to mesh?
      Geometry::AnyGeometry3D meshGeom;
      linkNode->geomData.Convert(Geometry::AnyGeometry3D::TriangleMesh,meshGeom,0.01);
      geomManagers[link_index].CreateEmpty();
      *geomManagers[link_index] = meshGeom;
      //make the default appearance be grey
      SetDefaultAppearance(geomManagers[link_index].Appearance());
      geometry[link_index] = geomManagers[link_index];
    }
    if(this->geometry[link_index]) {
      //LOG4CXX_INFO(GET_LOGGER(URDFParser),"Geometry "<<geomFiles[link_index]<<" has "<<this->geometry[link_index]->NumElements()<<" triangles");

      //set up color
      if(linkNode->link->visual && linkNode->link->visual->material) {
        urdf::Color c=linkNode->link->visual->material->color;
        this->geomManagers[link_index].SetUniqueAppearance();
        this->geomManagers[link_index].Appearance()->faceColor.set(c.r,c.g,c.b,c.a);
      }
      Matrix4 ident; ident.setIdentity();
      if(!linkNode->geomScale.isEqual(ident)) {
        this->geomManagers[link_index].TransformGeometry(linkNode->geomScale);
        this->geometry[link_index] = this->geomManagers[link_index];
      }
    }
  }


  //first mount the geometries, they affect whether a link is included in self collision testing
  vector<int> mountLinkIndices(mountLinks.size());
  for (size_t i = 0; i < mountLinks.size(); i++) {
    int linkIndex = LinkIndex(mountLinks[i].c_str());
    if(linkIndex < 0) {
      LOG4CXX_ERROR(GET_LOGGER(URDFParser),"   Invalid mount link "<<mountLinks[i]);
      return false;
    }
    mountLinkIndices[i] = linkIndex;
  }
  for (size_t i = 0; i < mountLinks.size(); i++) {
    const char* ext = FileExtension(mountFiles[i].c_str());
    if(ext && (0==strcmp(ext,"rob") || 0==strcmp(ext,"urdf"))) {
      //its a robot, delay til later
    }
    else {
      string fn = ResolveFileReference(path,mountFiles[i]);
      LOG4CXX_INFO(GET_LOGGER(URDFParser),"   Mounting geometry file " << mountFiles[i]);
      //mount a triangle mesh on top of another triangle mesh
      ManagedGeometry loader;
      if(!loader.Load(fn.c_str())) {
        LOG4CXX_ERROR(GET_LOGGER(URDFParser),"   Error loading mount geometry file " << fn);
        return false;
      }
      Mount(mountLinkIndices[i], *loader, mountT[i]);
    }
  }

  this->UpdateConfig(q);

  selfCollisions.resize(links_size, links_size, NULL);
  envCollisions.resize(links_size, NULL);

  //Initialize self collisions -- pre subchain mounting
  CleanupSelfCollisions();
  vector<pair<string,string> > residualSelfCollisions,residualNoSelfCollisions;
  if (selfCollision.empty()) {
    InitAllSelfCollisions();
  }
  else {
    for (size_t i = 0; i < selfCollision.size(); i++) {
      int link1,link2;
      link1 = LinkIndex(selfCollision[i].first.c_str());
      link2 = LinkIndex(selfCollision[i].second.c_str());
      if (link1 < 0 || link1 >= (int) links.size() ||
          link2 < 0 || link2 >= (int) links.size()) {
        residualSelfCollisions.push_back(selfCollision[i]);
        continue;
      }
      if(link1 > link2) Swap(link1,link2);
      if(!(link1 < link2)) {
        LOG4CXX_ERROR(GET_LOGGER(URDFParser),"Robot::LoadURDF(): Invalid self collision pair "<<selfCollision[i].first<<", "<<selfCollision[i].second);
          return false;
      }
      InitSelfCollisionPair(link1,link2);
    }
  }

  for (size_t i = 0; i < noSelfCollision.size(); i++) {
    int link1,link2;
    link1 = LinkIndex(noSelfCollision[i].first.c_str());
    link2 = LinkIndex(noSelfCollision[i].second.c_str());
    if (link1 < 0 || link1 >= (int) links.size() ||
        link2 < 0 || link2 >= (int) links.size()) {
      residualNoSelfCollisions.push_back(noSelfCollision[i]);
      continue;
    }
    if(link1 > link2) Swap(link1,link2);
    if(link1 == link2) continue;
    SafeDelete(selfCollisions(link1,link2));
  }

  //do the mounting of subchains
  for (size_t i = 0; i < mountLinks.size(); i++) {
    const char* ext = FileExtension(mountFiles[i].c_str());
    if(ext && (0==strcmp(ext,"rob") || 0==strcmp(ext,"urdf"))) {
      string fn = ResolveFileReference(path,mountFiles[i]);
      LOG4CXX_INFO(GET_LOGGER(URDFParser),"   Mounting subchain file " << mountFiles[i]);
      Robot subchain;
      if (!subchain.Load(fn.c_str())) {
        LOG4CXX_ERROR(GET_LOGGER(URDFParser),"   Error reading subchain file " << fn);
        return false;
      }
      const char* prefix = (mountNames[i].empty() ? NULL : mountNames[i].c_str());
      Mount(mountLinkIndices[i], subchain, mountT[i], prefix);
    }
  }
  if (!CheckValid())
    return false;

  //after mounting may need to add extra self collisions / no self collisions
  swap(selfCollision,residualSelfCollisions);
  swap(noSelfCollision,residualNoSelfCollisions);
  for (size_t i = 0; i < selfCollision.size(); i++) {
    int link1,link2;
    link1 = LinkIndex(selfCollision[i].first.c_str());
    link2 = LinkIndex(selfCollision[i].second.c_str());
    if (link1 < 0 || link1 >= (int) links.size() ||
        link2 < 0 || link2 >= (int) links.size()) {
      LOG4CXX_ERROR(GET_LOGGER(URDFParser),"   Error, invalid self-collision index "<<selfCollision[i].first.c_str()<<"-"<<
        selfCollision[i].second.c_str()<<" (range is 0,...,"<<(int)links.size()-1 <<")");
      return false;
    }
    if(link1 > link2) Swap(link1,link2);
    if(link1 == link2) continue;
    InitSelfCollisionPair(link1,link2);
  }

  for (size_t i = 0; i < noSelfCollision.size(); i++) {
    int link1,link2;
    link1 = LinkIndex(noSelfCollision[i].first.c_str());
    link2 = LinkIndex(noSelfCollision[i].second.c_str());
    if (link1 < 0 || link1 >= (int) links.size() ||
        link2 < 0 || link2 >= (int) links.size()) {
      LOG4CXX_ERROR(GET_LOGGER(URDFParser),"  Error, invalid no-collision index "<< noSelfCollision[i].first.c_str()<<"-"<<
        noSelfCollision[i].second.c_str()<<" (range is 0,...,"<< (int)links.size()-1<<")");
      return false;
    }
    if(link1 > link2) Swap(link1,link2);
    if(link1 == link2) continue;
    SafeDelete(selfCollisions(link1,link2));
  }
  /*

  //TESTING: don't need to do this with dynamic collision initialization
  //InitCollisions();
  CleanupSelfCollisions();
  if (selfCollision.empty()) {
    InitAllSelfCollisions();
  } else {
    for (size_t i = 0; i < selfCollision.size(); i++) {
      int link1,link2;
      link1 = LinkIndex(selfCollision[i].first.c_str());
      link2 = LinkIndex(selfCollision[i].second.c_str());
      if (link1 < 0 || link1 >= (int) links.size() ||
          link2 < 0 || link2 >= (int) links.size()) {
        LOG4CXX_ERROR(GET_LOGGER(URDFParser),"   Error, invalid self-collision index "<<selfCollision[i].first.c_str()<<"-"<<
          selfCollision[i].second.c_str()<<" (range is 0,...,"<<(int)links.size()-1<<")");
        return false;
      }
      if(link1 == link2) continue;
      if(link1 > link2) Swap(link1,link2);
      InitSelfCollisionPair(link1,link2);
    }
  }

  //Initialize self collisions -- these needs to be done after mounting
  for (size_t i = 0; i < noSelfCollision.size(); i++) {
      int link1,link2;
      link1 = LinkIndex(noSelfCollision[i].first.c_str());
      link2 = LinkIndex(noSelfCollision[i].second.c_str());
      if (link1 < 0 || link1 >= (int) links.size() ||
          link2 < 0 || link2 >= (int) links.size()) {
        LOG4CXX_ERROR(GET_LOGGER(URDFParser),"  Error, invalid no-collision index "<<noSelfCollision[i].first.c_str()<<"-"<< 
          noSelfCollision[i].second.c_str()<<" (range is 0,...,"<<(int)links.size()-1<<")");
        return false;
      }
      if(link1 == link2) continue;
      if(link1 > link2) Swap(link1,link2);
    SafeDelete(selfCollisions(link1,link2));
  }

  */

  LOG4CXX_INFO(GET_LOGGER(URDFParser),"Done loading robot file "<<fn);
  return true;
}

void Robot::ComputeLipschitzMatrix()
{
  Timer timer;
  lipschitzMatrix.resize(links.size(), links.size(), 0.0);
  for (size_t i = 0; i < links.size(); i++) {
    if (!geometry[i] || geometry[i]->Empty())
      continue;

    //translate workspace distance of link i into c-space distance
    Sphere3D s; //bound on workspace
    Box3D b;
    RigidTransform temp,ident;
    ident.setIdentity();
    temp = geometry[i]->GetTransform();
    geometry[i]->SetTransform(ident);
    b = geometry[i]->GetBB();
    geometry[i]->SetTransform(temp);
    s.center = b.origin + 0.5 * b.dims.x * b.xbasis
        + 0.5 * b.dims.y * b.ybasis + 0.5 * b.dims.z * b.zbasis;
    s.radius = Radius(*geometry[i]);

    //s.radius = b.dims.norm()*0.5;
    Real lipschitz = 0;
    int j = i;
    while (j >= 0) {
      //compute lipschitz constants for all parents of i
      Assert(j >= 0 && j < (int) links.size());
      if (links[j].type == RobotLink3D::Revolute) {
        //LOG4CXX_INFO(GET_LOGGER(Robot),"   Link "<<j<<" contributes "<<cross(links[j].w,s.center).norm()+s.radius <<" to lipschitz constant");  
        lipschitz += cross(links[j].w, s.center).norm() + s.radius;
        //re-bound geometry
        s.radius = cross(links[j].w, s.center).norm() + s.radius;
        s.center = links[j].w * links[j].w.dot(s.center);
        //shift to parent
        s.center = links[j].T0_Parent * s.center;
      } else {
        if (qMax[j] != qMin[j]) { //normal translation  joint
          //LOG4CXX_INFO(GET_LOGGER(Robot),"   Link "<<j <<" contributes 1 to lipschitz constant");
          lipschitz += 1.0;
          s.radius += qMax[j] - qMin[j];
          s.center = links[j].T0_Parent * s.center;
        }
      }
      lipschitzMatrix(j, i) = lipschitz;
      LOG4CXX_INFO(GET_LOGGER(Robot),"Lipschitz "<<j<<" "<<i<<" = "<<lipschitz);
      j = parents[j];
    }
  }
  LOG4CXX_INFO(GET_LOGGER(Robot),"Done computing lipschitz constants, took "<<timer.ElapsedTime()<<"s");
}

//defined in ODERobot.cpp
Matrix3 TranslateInertia(const Matrix3& Hc,const Vector3& c,Real mass);

void Robot::Reduce(Robot& reduced,vector<int>& dofMap)
{
  vector<bool> fixedLink(q.n,false);
  vector<int> fixedParent(q.n,-1);
  for(int i=0;i<q.n;i++) {
    if(qMin[i] == qMax[i] || (velMin[i] == 0 && velMax[i] == 0))
      fixedLink[i] = true;
  }
  for(size_t i=0;i<joints.size();i++)
    if(joints[i].type == RobotJoint::Weld) {
      fixedLink[joints[i].linkIndex] = true;
    }
  for(size_t i=0;i<drivers.size();i++)
    if(drivers[i].qmin == drivers[i].qmax || (drivers[i].vmax == 0 && drivers[i].vmin == 0)) {
      for(auto j:drivers[i].linkIndices) {
        fixedLink[j] = true;
      }
    }
  //determine fixed link parents
  for(int i=0;i<q.n;i++) {
    if(fixedLink[i]) {
      if(parents[i] >= 0)
        fixedParent[i] = fixedParent[parents[i]];
    }
    else {
      fixedParent[i] = i;
    }
    //int p=parents[i];
    //printf("Attaching link %d, %s (%s, normal parent %d) to %d\n",i,linkNames[i].c_str(),(fixedLink[i]?"fixed":"free"),p,fixedParent[i]);
  }
  vector<vector<int> > children(q.n);
  for(size_t i=0;i<fixedParent.size();i++) {
    if(fixedParent[i] >= 0)
      children[fixedParent[i]].push_back(int(i));
  }
  vector<int> freeToRobot;
  vector<int> robotToFree(q.n,-1);
  for(size_t i=0;i<fixedLink.size();i++) {
    if(!fixedLink[i]) {
      robotToFree[i] = (int)freeToRobot.size();
      freeToRobot.push_back(i);
    }
  }
  Config oldq = q;
  q.setZero();
  UpdateFrames();
  q = oldq;
  reduced.Initialize((int)freeToRobot.size());
  reduced.linkNames.resize(freeToRobot.size());
  reduced.accMax.resize(freeToRobot.size());
  reduced.geomManagers.resize(freeToRobot.size());
  reduced.geomFiles.resize(freeToRobot.size());
  reduced.name = name;
  for(size_t i=0;i<freeToRobot.size();i++) {
    int origLink = freeToRobot[i];
    //printf("Mapping link %d to %d\n",origLink,i);
    reduced.linkNames[i] = linkNames[origLink];
    reduced.qMin[i] = qMin[origLink];
    reduced.qMax[i] = qMax[origLink];
    reduced.velMin[i] = velMin[origLink];
    reduced.velMax[i] = velMax[origLink];
    reduced.accMax[i] = accMax[origLink];
    reduced.torqueMax[i] = torqueMax[origLink];
    reduced.powerMax[i] = powerMax[origLink];
    if(parents[origLink] >= 0) {
      int fp = fixedParent[parents[origLink]];
      if(fp >= 0)
        reduced.parents[i] = robotToFree[fp];
      else
        reduced.parents[i] = -1;
    }
    else
      reduced.parents[i] = -1;
    //printf("Parent of %d set to %d\n",i,reduced.parents[i]);
    reduced.links[i] = links[origLink];
    reduced.q[i] = q[origLink];
    reduced.dq[i] = dq[origLink];
    reduced.geometry[i] = geometry[origLink];
    reduced.geomFiles[i] = geomFiles[origLink];
    reduced.geomManagers[i] = geomManagers[origLink];
    RobotLink3D& newLink = reduced.links[i];
    //transform to parent may have been changed
    if(reduced.parents[i] >= 0)
      newLink.T0_Parent.mulInverseA(reduced.links[reduced.parents[i]].T_World,reduced.links[i].T_World);
    else
      newLink.T0_Parent = reduced.links[i].T_World;
    //update geometry / inertial properties for aggregate link
    if(children[origLink].size() > 1) {
      newLink.mass = 0;
      newLink.com.setZero();
      newLink.inertia.setZero();
      vector<Geometry::AnyGeometry3D> cgeoms(children[origLink].size());
      //printf("Merging link %d children ",origLink);
      //for(auto c:children[origLink])
      //  printf("%d ",c);
      //printf("\n");
      for(size_t j=0;j<children[origLink].size();j++) {
        int c = children[origLink][j];
        const RobotLink3D& oldLink = links[c];
        RigidTransform Trel;
        Trel.mulInverseA(links[origLink].T_World,oldLink.T_World);
        if(geometry[c] && !geometry[c]->Empty()) {
          cgeoms[j] = *geometry[c];
          cgeoms[j].Transform(Trel);
        }
        if(oldLink.mass <= 0) continue;
        newLink.mass += oldLink.mass;
        newLink.com += oldLink.mass*(Trel*oldLink.com);
        //transform inertia matrix
        Matrix3 temp,inertiaLink;
        temp.mulTransposeB(oldLink.inertia,Trel.R);
        inertiaLink.mul(Trel.R,temp);
        inertiaLink = TranslateInertia(inertiaLink,Trel.t,oldLink.mass);
        newLink.inertia += (inertiaLink*oldLink.mass);
      }
      reduced.geomFiles[i] = "";
      ManagedGeometry::GeometryPtr g = reduced.geomManagers[i].CreateEmpty();
      Assert(reduced.geomManagers[i].Empty());
      for(size_t j=0;j<cgeoms.size();j++)
        if(cgeoms[j].Empty()) {
          cgeoms[j] = cgeoms.back();
          cgeoms.resize(cgeoms.size()-1);
        }
      //printf("Creating group with %d geometries\n",cgeoms.size());
      if(cgeoms.size() == 1) {
        *g = CollisionGeometry(cgeoms[0]);
        Assert(!reduced.geomManagers[i].Empty());
      }
      else if(cgeoms.size() > 1) {
        *g = CollisionGeometry(cgeoms);
        Assert(!reduced.geomManagers[i].Empty());
      }
      reduced.geomManagers[i].OnGeometryChange();
      reduced.geometry[i] = reduced.geomManagers[i];
      reduced.geomFiles[i] = "";
      newLink.com /= newLink.mass;
      newLink.inertia /= newLink.mass;
    }
  }
  reduced.UpdateFrames();
  reduced.UpdateGeometry();
  //copy self-collision information
  for(size_t i=0;i<freeToRobot.size();i++) {
    int origLink = freeToRobot[i];
    for(size_t j=0;j<freeToRobot.size();j++) {
      int origLink2 = freeToRobot[j];
      if(selfCollisions(origLink,origLink2) != NULL) {
        reduced.InitSelfCollisionPair(i,j);
      }
    }
  }
  //copy driver information
  reduced.drivers.resize(0);
  reduced.driverNames.resize(0);
  for(size_t i=0;i<drivers.size();i++) {
    const auto& d=drivers[i];
    vector<int> mappedIndices (d.linkIndices.size());
    bool include = true;
    for(size_t j=0;j<d.linkIndices.size();j++) {
      if(robotToFree[d.linkIndices[j]] < 0) {
        include=false;
        break;
      }
      mappedIndices[j] = robotToFree[d.linkIndices[j]];
    }
    if(!include) {
      LOG4CXX_WARN(GET_LOGGER(Robot),"Robot::Reduce: driver dropped because it drives a fixed link??");
      continue;
    }
    reduced.drivers.push_back(d);
    reduced.drivers.back().linkIndices = mappedIndices;
    reduced.driverNames.push_back(driverNames[i]);
  }
  //copy joint information
  reduced.joints.resize(0);
  for(size_t i=0;i<joints.size();i++) {
    const auto& j=joints[i];
    if(fixedLink[j.linkIndex]) continue;
    Assert(j.type != RobotJoint::Weld);
    reduced.joints.push_back(j);
    Assert(j.linkIndex >= 0);
    reduced.joints.back().linkIndex = robotToFree[j.linkIndex];
    if(j.baseIndex >= 0)
      reduced.joints.back().baseIndex = robotToFree[j.baseIndex];
  }
  dofMap = robotToFree;
}

void Robot::Merge(const std::vector<Robot*>& robots)
{
  vector<RobotWithGeometry*> grobots(robots.size());
  copy(robots.begin(),robots.end(),grobots.begin());
  RobotWithGeometry::Merge(grobots);

  size_t nl = 0, nj = 0, nd = 0;
  vector<size_t> offset(robots.size());
  vector<size_t> joffset(robots.size());
  vector<size_t> doffset(robots.size());
  for(size_t i=0;i<robots.size();i++) {
    offset[i] = nl;
    joffset[i] = nj;
    doffset[i] = nd;
    nl += robots[i]->links.size();
    nj += robots[i]->joints.size();
    nd += robots[i]->drivers.size();
  }
  
  geomFiles.resize(nl);
  accMax.resize(nl);
  joints.resize(nj);
  drivers.resize(nd);
  linkNames.resize(nl);
  driverNames.resize(nd);
  for(size_t i=0;i<robots.size();i++) {
    accMax.copySubVector(offset[i],robots[i]->accMax);
    copy(robots[i]->geomFiles.begin(),robots[i]->geomFiles.end(),geomFiles.begin()+offset[i]);
    copy(robots[i]->linkNames.begin(),robots[i]->linkNames.end(),linkNames.begin()+offset[i]);
    for(size_t j=0;j<robots[i]->joints.size();j++) {
      joints[j+joffset[i]] = robots[i]->joints[j];
      joints[j+joffset[i]].linkIndex += offset[i];
      if(joints[j+joffset[i]].baseIndex >= 0)
  joints[j+joffset[i]].baseIndex += offset[i];
    }
    for(size_t j=0;j<robots[i]->drivers.size();j++) {
      drivers[j+doffset[i]] = robots[i]->drivers[j];
      for(size_t k=0;k<robots[i]->drivers[j].linkIndices.size();k++)
  drivers[j+doffset[i]].linkIndices[k] += offset[i];
    }
    copy(robots[i]->driverNames.begin(),robots[i]->driverNames.end(),driverNames.begin()+doffset[i]);
  }
}
