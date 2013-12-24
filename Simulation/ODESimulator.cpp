#include "ODESimulator.h"
#include "ODECommon.h"
#include "ODECustomGeometry.h"
#include "Settings.h"
#include <list>
#include <fstream>
//#include "Geometry/Clusterize.h"
#include <geometry/ConvexHull2D.h>
#include <statistics/KMeans.h>
#include <statistics/HierarchicalClustering.h>
#include <utils/EquivalenceMap.h>
#include <utils/permutation.h>
#include <ode/ode.h>
#include <Timer.h>
#ifndef WIN32
#include <unistd.h>
#endif //WIN32

#define TEST_READ_WRITE_STATE 0
#define DO_TIMING 0

const static size_t gMaxKMeansSize = 5000;
const static size_t gMaxHClusterSize = 2000;
static size_t gPreclusterContacts = 0;
static double gClusterTime = 0;
static double gContactDetectTime = 0;

template <class T>
bool TestReadWriteState(T& obj,const char* name="")
{
  File fwrite,fwritenew;
  fwrite.OpenData();
  if(!obj.WriteState(fwrite)) {
    fprintf(stderr,"WriteState %s failed\n",name);
    return false;
  }
  //HACK for File internal buffer length bug returning buffer capacity rather
  //than size
  //int n = fwrite.Length();
  int n1 = fwrite.Position();
  fwrite.Seek(0,FILESEEKSTART);
  if(!obj.ReadState(fwrite)) {
    fprintf(stderr,"ReadState %s failed\n",name);
    return false;
  }
  fwritenew.OpenData();
  if(!obj.WriteState(fwritenew)) {
    fprintf(stderr,"Second WriteState %s failed\n",name);
    return false;
  }
  //HACK for File internal buffer length bug returning buffer capacity rather
  //than size
  //int n2 = fwritenew.Length();
  int n2 = fwritenew.Position();
  char* d1 = (char*)fwrite.GetDataBuffer();
  char* d2 = (char*)fwritenew.GetDataBuffer();
  if(n1 != n2) {
    fprintf(stderr,"WriteState %s wrote different numbers of bytes: %d -> %d\n",name,n1,n2);
    return false;
  }
  for(int i=0;i<n1;i++) {
    if(d1[i] != d2[i]) {
      fprintf(stderr,"WriteState %s wrote different byte at position %d/%d: 0x%x vs 0x%x\n",name,i,n1,(int)d1[i],(int)d2[i]);
      return false;
    }
  }
  return true;
}


//contact merging -- these are now handled by the contact clustering procedure
const static bool kMergeContacts = false;
const static Real kContactPosMergeTolerance = 3e-3;
const static Real kContactOriMergeTolerance = 1e-1;


ODESimulatorSettings::ODESimulatorSettings()
{
  gravity[0] = gravity[1] = 0;
  gravity[2] = -9.8;
  defaultEnvPadding = gDefaultEnvPadding;
  defaultEnvSurface.kFriction = 0.3;
  defaultEnvSurface.kRestitution = 0.1;
  defaultEnvSurface.kStiffness = 80000;
  defaultEnvSurface.kDamping = 20000;
  //defaultEnvSurface.kStiffness = Inf;
  //defaultEnvSurface.kDamping = Inf;

  boundaryLayerCollisions = gBoundaryLayerCollisionsEnabled;
  rigidObjectCollisions = gRigidObjectCollisionsEnabled;
  robotSelfCollisions = gRobotSelfCollisionsEnabled;
  robotRobotCollisions = gRobotRobotCollisionsEnabled;

  maxContacts = 20;
  clusterNormalScale = 0.1;

  errorReductionParameter = 0.95;
  dampedLeastSquaresParameter = 1e-6;
}

inline Real ERPFromSpring(Real timestep,Real kP,Real kD)
{
  return timestep*kP/(timestep*kP+kD);
}

inline Real CFMFromSpring(Real timestep,Real kP,Real kD)
{
  return 1.0/(timestep*kP+kD);
}



struct ODEObject 
{
  bool gODEInitialized;
  ODEObject () : gODEInitialized(false) {}
  void Init() {
    if(!gODEInitialized) {
      #ifdef dDOUBLE
      if(dCheckConfiguration("ODE_double_precision")!=1) {
	FatalError("ODE is compiled with single precision but Klamp't is compiled with double, either reconfigure ODE with --enable-double-precision or recompile Klamp't with dDOUBLE");
      }
      #else
      if(dCheckConfiguration("ODE_single_precision")!=1) {
	FatalError("ODE is compiled with double precision but Klamp't is compiled with single, either reconfigure ODE without --enable-double-precision or recompile Klamp't with dSINGLE");
      }
      #endif

      printf("Initializing ODE...\n");
      dInitODE();
      InitODECustomGeometry();
      gODEInitialized = true;
    }
  }
  ~ODEObject() { 
    if(gODEInitialized) {
      printf("Closing ODE...\n");
      dCloseODE(); 
    }
  }
};

ODEObject g_ODE_object;



//stuff for contact detection callbacks
struct ODEContactResult
{
  dGeomID o1,o2;
  vector<dContactGeom> contacts;
  vector<dJointFeedback> feedback;
};

const static int max_contacts = 1000;
static dContactGeom gContactTemp[max_contacts];
static list<ODEContactResult> gContacts;



ODESimulator::ODESimulator()
{
  timestep = 0;

  g_ODE_object.Init();
  worldID = dWorldCreate();
  contactGroupID = dJointGroupCreate(0);
  //envSpaceID = dHashSpaceCreate(0);
  //dHashSpaceSetLevels(envSpaceID,-2,1);
  envSpaceID = dSimpleSpaceCreate(0);

  dWorldSetERP(worldID,settings.errorReductionParameter);
  dWorldSetCFM(worldID,settings.dampedLeastSquaresParameter);
  dWorldSetGravity(worldID,settings.gravity[0],settings.gravity[1],settings.gravity[2]);
}

void ODESimulator::SetGravity(const Vector3& g)
{
  g.get(settings.gravity);
  dWorldSetGravity(worldID,g.x,g.y,g.z);
}

void ODESimulator::SetERP(double erp)
{
  settings.errorReductionParameter = erp;
  dWorldSetERP(worldID,erp);
}

void ODESimulator::SetCFM(double cfm)
{
  settings.dampedLeastSquaresParameter = cfm;
  dWorldSetCFM(worldID,cfm);
}

ODESimulator::~ODESimulator()
{
  dJointGroupDestroy(contactGroupID);
  for(size_t i=0;i<envGeoms.size();i++)
    delete envGeoms[i];
  for(size_t i=0;i<robots.size();i++)
    delete robots[i];
  dSpaceDestroy(envSpaceID);
  dWorldDestroy(worldID);
}

void ODESimulator::AddEnvironment(Environment& env)
{
  envs.push_back(&env);
  envGeoms.resize(envGeoms.size()+1);
  envGeoms.back() = new ODEGeometry;
  envGeoms.back()->Create(&env.geometry,envSpaceID,Vector3(Zero),settings.boundaryLayerCollisions);
  envGeoms.back()->surf() = settings.defaultEnvSurface;
  envGeoms.back()->SetPadding(settings.defaultEnvPadding);
  if(!env.kFriction.empty())
    envGeoms.back()->surf().kFriction = env.kFriction[0];
  //the index of the environment is encoded as -1-index
  dGeomSetData(envGeoms.back()->geom(),(void*)(-(int)envs.size()));
  dGeomSetCategoryBits(envGeoms.back()->geom(),0x1);
  dGeomSetCollideBits(envGeoms.back()->geom(),0xffffffff ^ 0x1);
}

void ODESimulator::AddRobot(Robot& robot)
{
  robots.push_back(new ODERobot(robot));
  //For some reason, self collisions don't work with hash spaces
  robots.back()->Create(worldID,settings.boundaryLayerCollisions);
  //robotStances.resize(robots.size());
  for(size_t i=0;i<robot.links.size();i++)
    if(robots.back()->triMesh(i) && robots.back()->geom(i)) {
      if(robots.back()->robot.parents[i] == -1) { //treat as part of the terrain
	dGeomSetCategoryBits(robots.back()->geom(i),0x1);
	dGeomSetCollideBits(robots.back()->geom(i),0xffffffff ^ 0x1);
      }
      else {
	dGeomSetCategoryBits(robots.back()->geom(i),0x4);
	dGeomSetCollideBits(robots.back()->geom(i),0xffffffff);
      }
    }
}

void ODESimulator::AddObject(RigidObject& object)
{
  objects.push_back(new ODERigidObject(object));
  objects.back()->Create(worldID,envSpaceID,settings.boundaryLayerCollisions);
  dGeomSetData(objects.back()->geom(),(void*)(objects.size()-1));
  dGeomSetCategoryBits(objects.back()->geom(),0x2);
  dGeomSetCollideBits(objects.back()->geom(),0xffffffff);
}


void ODESimulator::Step(Real dt)
{
  Assert(timestep == 0);

#if DO_TIMING
  Timer timer;
  double collisionTime,stepTime,updateTime;
  gContactDetectTime = gClusterTime = 0;
  gPreclusterContacts = 0;
#endif // DO_TIMING

  gContacts.clear();

  timestep=dt;
  DetectCollisions();

  //printf("  %d contacts detected\n",gContacts.size());

#if DO_TIMING
  collisionTime = timer.ElapsedTime();
  timer.Reset();
#endif // DO_TIMING

  StepDynamics(dt);

#if DO_TIMING
  stepTime = timer.ElapsedTime();
  timer.Reset();
#endif // DO_TIMING

  for(map<pair<ODEObjectID,ODEObjectID>,ODEContactList>::iterator i=contactList.begin();i!=contactList.end();i++) {  
    ODEContactList& cl=i->second;
    cl.forces.clear();
    for(size_t j=0;j<cl.feedbackIndices.size();j++) {
      int k=cl.feedbackIndices[j];
      Assert(k >= 0 && k < (int)gContacts.size());
      list<ODEContactResult>::iterator cres=gContacts.begin();
      advance(cres,k);
      Vector3 temp;
      for(size_t i=0;i<cres->feedback.size();i++) {
	CopyVector(temp,cres->feedback[i].f1);
	cl.forces.push_back(temp);
	/*
	if(!cl.points.back().isValidForce(-temp)) {
	  printf("ODESimulator: Warning, solved contact force %d %d is invalid\n",k,i);
	  cout<<"  Force: "<<-temp<<", normal "<<cl.points.back().n<<" kFriction "<<cl.points.back().kFriction<<endl;
	}
	*/
      }
    }
    if(!cl.feedbackIndices.empty())
      Assert(cl.points.size() == cl.forces.size());
  }

  timestep = 0;

#if DO_TIMING
  updateTime = timer.ElapsedTime();
#endif //DO_TIMING

#if DO_TIMING
  ofstream out("odesimulator_timing.csv",ios::app);
  if(out.tellp()==0) {
    cout<<"Saving to odesimulator_timing.csv"<<endl;
    out<<"total,#preclusterContacts,#contacts,collision detection,contact detect,clustering,dynamics step,misc update"<<endl;
  }
  size_t nc = 0;
  for(list<ODEContactResult>::iterator i=gContacts.begin();i!=gContacts.end();i++)
    nc += i->contacts.size();
  out<<collisionTime+stepTime+updateTime<<","<<gPreclusterContacts<<","<<nc<<","<<collisionTime<<","<<gContactDetectTime<<","<<gClusterTime<<","<<stepTime<<","<<updateTime<<endl;
#endif

  //KH: commented this out so GetContacts() would work for ContactSensor simulation.  Be careful about loading state
  //gContacts.clear();
}


struct EqualPlane
{
  //position tolerance and angle tolerance, in radians
  EqualPlane(double xtol,double atol) : ptol(xtol),catol(Cos(atol)) {}

  //assumes normalized normals
  bool operator () (const dContactGeom& a,const dContactGeom& b)
  {
    Vector3 an,bn;
    CopyVector(an,a.normal);
    CopyVector(bn,b.normal);
    Real ndot = an.dot(bn);
    if(ndot > catol) {
      Vector3 ax,bx;
      CopyVector(ax,a.pos);
      CopyVector(bx,b.pos);
      return  FuzzyEquals(ax.dot(an),bx.dot(an),ptol) && 
	FuzzyEquals(ax.dot(bn),bx.dot(bn),ptol);
    }
    return false;
  }

  double ptol,catol;
};

struct EqualPoint
{
  //position tolerance and angle tolerance, in radians
  EqualPoint(double xtol) : ptol(xtol) {}

  //assumes normalized normals
  bool operator () (const dContactGeom& a,const dContactGeom& b)
  {
    Vector3 ax,bx;
    CopyVector(ax,a.pos);
    CopyVector(bx,b.pos);
    return  ax.distanceSquared(bx) < Sqr(ptol);
  }

  double ptol;
};


/** Two ways of merging frictionless contacts without resorting to 6D
 * wrench space
 * 1) interior points in 2D convex hull, with the projection defined 
 *    by the contact normal
 * 2) interior points in convex cone at a single contact point (TODO)
 * 
 * If contacts have friction, the convex hull method only works when the
 * points are on the same plane.
 */
void CHContactsPlane(vector<dContactGeom>& contacts)
{
  if(contacts.empty()) return;
  if(contacts.size() <= 2) return;

  Vector3 c(Zero),n(Zero);
  for(size_t i=0;i<contacts.size();i++) {
    Vector3 cx,nx;
    CopyVector(cx,contacts[i].pos);
    CopyVector(nx,contacts[i].normal);
    c += cx;
    n += nx;
  }
  c /= contacts.size();
  n.inplaceNormalize();

  //get the deepest contact
  size_t deepest = 0;
  for(size_t i=1;i<contacts.size();i++) 
    if(contacts[i].depth > contacts[deepest].depth) deepest=i;

  //make a plane
  Vector3 x,y;
  n.getOrthogonalBasis(x,y);
  //Real nofs = n.dot(c);
  Real xofs = x.dot(c);
  Real yofs = y.dot(c);
  vector<Vector2> pt2d(contacts.size());
  for(size_t i=0;i<contacts.size();i++) {
    Vector3 cx;
    CopyVector(cx,contacts[i].pos);
    pt2d[i].x = x.dot(cx)-xofs;
    pt2d[i].y = y.dot(cx)-yofs;
  }

  //get the convex hull
  vector<Vector2> ch(contacts.size());
  vector<int> mapping(contacts.size());
  int num=Geometry::ConvexHull2D_Chain_Unsorted(&pt2d[0],contacts.size(),&ch[0],&mapping[0]);
  vector<dContactGeom> temp(num);
  bool addDeepest = true;
  for(int i=0;i<num;i++) {
    Assert(mapping[i] < (int)contacts.size());
    temp[i] = contacts[mapping[i]];
    if(mapping[i] == (int)deepest)
      addDeepest = false;
  }
  if(addDeepest)
    temp.push_back(contacts[deepest]);
  swap(temp,contacts);
}

void interpolate(const dContactGeom& a,const dContactGeom& b,Real u,dContactGeom& x)
{
  for(int i=0;i<3;i++) {
    x.pos[i] = (1-u)*a.pos[i]+u*b.pos[i];
  }
  Vector3 an,bn;
  CopyVector(an,a.normal);
  CopyVector(bn,b.normal);
  Vector3 n = (1-u)*an+u*bn;
  n.inplaceNormalize();
  CopyVector(x.normal,n);

  x.depth = Max(a.depth,b.depth);
  x.g1 = a.g1;
  x.g2 = a.g2;
  Assert(x.g1 == b.g1);
  Assert(x.g2 == b.g2);
}


void ClusterContactsMerge(vector<dContactGeom>& contacts,int maxClusters,Real clusterNormalScale)
{
  if((int)contacts.size() <= maxClusters) return;
  vector<Vector> pts(contacts.size());
  for(size_t i=0;i<pts.size();i++) {
    pts[i].resize(7);
    pts[i][0] = contacts[i].pos[0];
    pts[i][1] = contacts[i].pos[1];
    pts[i][2] = contacts[i].pos[2];
    pts[i][3] = contacts[i].normal[0]*clusterNormalScale;
    pts[i][4] = contacts[i].normal[1]*clusterNormalScale;
    pts[i][5] = contacts[i].normal[2]*clusterNormalScale;
    pts[i][6] = contacts[i].depth;
  }

  Timer timer;
  Statistics::HierarchicalClustering clust;
  clust.Build(pts,maxClusters,Statistics::HierarchicalClustering::AverageLinkage);
  //cout<<"Clustering time: "<<timer.ElapsedTime()<<endl;
  /*
  cout<<"Points"<<endl;
  for(size_t i=0;i<pts.size();i++)
    cout<<pts[i]<<endl;
  */

  //read out the clusters
  contacts.resize(maxClusters);
  for(int i=0;i<maxClusters;i++) {
    const vector<int>& inds = clust.Cluster(i);
    /*
    cout<<"Cluster "<<i<<": ";
    for(size_t j=0;j<inds.size();j++)
      cout<<inds[j]<<", ";
    cout<<endl;
    */
    Vector mean(7,Zero);
    for(size_t j=0;j<inds.size();j++) 
      mean += pts[inds[j]];
    mean /= inds.size();

    contacts[i].pos[0] = mean[0];
    contacts[i].pos[1] = mean[1];
    contacts[i].pos[2] = mean[2];
    contacts[i].normal[0] = mean[3]/clusterNormalScale;
    contacts[i].normal[1] = mean[4]/clusterNormalScale;
    contacts[i].normal[2] = mean[5]/clusterNormalScale;

    Real len = Vector3(contacts[i].normal[0],contacts[i].normal[1],contacts[i].normal[2]).length();
    if(FuzzyZero(len) || !IsFinite(len)) {
      printf("ODESimulator: Warning, clustered normal became zero/infinite\n");
      int found = inds[0];
      contacts[i].pos[0] = pts[found][0];
      contacts[i].pos[1] = pts[found][1];
      contacts[i].pos[2] = pts[found][2];
      contacts[i].normal[0] = pts[found][3];
      contacts[i].normal[1] = pts[found][4];
      contacts[i].normal[2] = pts[found][5];
      Real len = Vector3(contacts[i].normal[0],contacts[i].normal[1],contacts[i].normal[2]).length();
      contacts[i].normal[0] /= len;
      contacts[i].normal[1] /= len;
      contacts[i].normal[2] /= len;
      contacts[i].depth = pts[found][6];
      continue;
    }
    contacts[i].normal[0] /= len;
    contacts[i].normal[1] /= len;
    contacts[i].normal[2] /= len;
    contacts[i].depth = mean[6];
  }
}

void ClusterContactsKMeans(vector<dContactGeom>& contacts,int maxClusters,Real clusterNormalScale)
{
  if((int)contacts.size() <= maxClusters) return;
  vector<Vector> pts(contacts.size());
  for(size_t i=0;i<pts.size();i++) {
    pts[i].resize(7);
    pts[i][0] = contacts[i].pos[0];
    pts[i][1] = contacts[i].pos[1];
    pts[i][2] = contacts[i].pos[2];
    pts[i][3] = contacts[i].normal[0]*clusterNormalScale;
    pts[i][4] = contacts[i].normal[1]*clusterNormalScale;
    pts[i][5] = contacts[i].normal[2]*clusterNormalScale;
    pts[i][6] = contacts[i].depth;
  }

  Statistics::KMeans kmeans(pts,maxClusters);
  //randomized
  //kmeans.RandomInitialCenters();
  //deterministic
  for(size_t i=0;i<kmeans.centers.size();i++)
    kmeans.centers[i] = kmeans.data[(i*pts.size())/kmeans.centers.size()];
  int iters=20;
  kmeans.Iterate(iters);
  contacts.resize(kmeans.centers.size());
  vector<int> degenerate;
  for(size_t i=0;i<contacts.size();i++) {
    contacts[i].pos[0] = kmeans.centers[i][0];
    contacts[i].pos[1] = kmeans.centers[i][1];
    contacts[i].pos[2] = kmeans.centers[i][2];
    contacts[i].normal[0] = kmeans.centers[i][3]/clusterNormalScale;
    contacts[i].normal[1] = kmeans.centers[i][4]/clusterNormalScale;
    contacts[i].normal[2] = kmeans.centers[i][5]/clusterNormalScale;
    Real len = Vector3(contacts[i].normal[0],contacts[i].normal[1],contacts[i].normal[2]).length();
    if(FuzzyZero(len) || !IsFinite(len)) {
      printf("ODESimulator: Warning, clustered normal became zero/infinite\n");
      //pick any in the cluster
      int found = -1;
      for(size_t k=0;k<kmeans.labels.size();k++) {
	if(kmeans.labels[k] == (int)i) {
	  found = (int)k;
	  break;
	}
      }
      if(found < 0) {
	//strange -- degenerate cluster?
	degenerate.push_back(i);
	continue;
      }
      contacts[i].pos[0] = pts[found][0];
      contacts[i].pos[1] = pts[found][1];
      contacts[i].pos[2] = pts[found][2];
      contacts[i].normal[0] = pts[found][3];
      contacts[i].normal[1] = pts[found][4];
      contacts[i].normal[2] = pts[found][5];
      Real len = Vector3(contacts[i].normal[0],contacts[i].normal[1],contacts[i].normal[2]).length();
      contacts[i].normal[0] /= len;
      contacts[i].normal[1] /= len;
      contacts[i].normal[2] /= len;
      contacts[i].depth = pts[found][6];
      continue;
    }
    contacts[i].normal[0] /= len;
    contacts[i].normal[1] /= len;
    contacts[i].normal[2] /= len;
    //cout<<"Clustered contact "<<contacts[i].pos[0]<<" "<<contacts[i].pos[1]<<" "<<contacts[i].pos[2]<<endl;
    //cout<<"Clustered normal "<<contacts[i].normal[0]<<" "<<contacts[i].normal[1]<<" "<<contacts[i].normal[2]<<endl;
    contacts[i].depth = kmeans.centers[i][6];
  }
  reverse(degenerate.begin(),degenerate.end());
  for(size_t i=0;i<degenerate.size();i++) {
    contacts.erase(contacts.begin()+degenerate[i]);
  }
}

void ClusterContacts(vector<dContactGeom>& contacts,int maxClusters,Real clusterNormalScale)
{
  gPreclusterContacts += contacts.size();

  //for really big contact sets, do a subsampling
  if(contacts.size()*maxClusters > gMaxKMeansSize && contacts.size()*contacts.size() > gMaxHClusterSize) {
    int minsize = Max((int)gMaxKMeansSize/maxClusters,(int)Sqrt(Real(gMaxHClusterSize)));
    printf("ClusterContacts: subsampling %d to %d contacts\n",(int)contacts.size(),minsize);
    //subsample
    vector<int> subsample(contacts.size());
    RandomPermutation(subsample);
    subsample.resize(minsize);
    vector<dContactGeom> subcontacts(subsample.size());
    for(size_t i=0;i<subsample.size();i++)
      subcontacts[i] = contacts[subsample[i]];
    swap(subcontacts,contacts);
  }
  size_t hclusterSize = contacts.size()*contacts.size();
  size_t kmeansSize = contacts.size()*maxClusters;
  if(hclusterSize < gMaxHClusterSize)
    ClusterContactsMerge(contacts,maxClusters,clusterNormalScale);
  else 
    ClusterContactsKMeans(contacts,maxClusters,clusterNormalScale);
}

void MergeContacts(vector<dContactGeom>& contacts,double posTolerance,double oriTolerance)
{
  EqualPlane eq(posTolerance,oriTolerance);
  vector<vector<int> > sets;
  EquivalenceMap(contacts,sets,eq);

  vector<dContactGeom> res;
  vector<dContactGeom> temp;
  for(size_t i=0;i<sets.size();i++) {
    if(sets[i].empty()) continue;
    temp.resize(sets[i].size());
    for(size_t j=0;j<sets[i].size();j++)
      temp[j] = contacts[sets[i][j]];
    CHContactsPlane(temp);
    //printf("CH Reduced from %d to %d\n",sets[i].size(),temp.size());

    //EqualPoint eq(posTolerance);
    //Clusterize(temp,eq);

    res.insert(res.end(),temp.begin(),temp.end());
  }
  swap(contacts,res);
}

void collisionCallback(void *data, dGeomID o1, dGeomID o2)
{
  Assert(!dGeomIsSpace(o1) && !dGeomIsSpace(o2));
  
  int num = dCollide (o1,o2,max_contacts,gContactTemp,sizeof(dContactGeom));
  vector<dContactGeom> vcontact(num);
  int numOk = 0;
  for(int i=0;i<num;i++) {
    if(gContactTemp[i].g1 == o2 && gContactTemp[i].g2 == o1) {
      printf("Swapping contact\n");
      std::swap(gContactTemp[i].g1,gContactTemp[i].g2);
      for(int k=0;k<3;k++) gContactTemp[i].normal[k]*=-1.0;
      std::swap(gContactTemp[i].side1,gContactTemp[i].side2);
    }
    Assert(gContactTemp[i].g1 == o1);
    Assert(gContactTemp[i].g2 == o2);
    vcontact[numOk] = gContactTemp[i];
    const dReal* n=vcontact[numOk].normal;
    if(Sqr(n[0])+Sqr(n[1])+Sqr(n[2]) < 0.9 || Sqr(n[0])+Sqr(n[1])+Sqr(n[2]) > 1.2) {
      //GIMPACT will report this
      //printf("Warning, degenerate contact with normal %f %f %f\n",vcontact[numOk].normal[0],vcontact[numOk].normal[1],vcontact[numOk].normal[2]);
      continue;
    }
    numOk++;
  }
  vcontact.resize(numOk);
  
  if(vcontact.size() > 0) {
    gContacts.push_back(ODEContactResult());
    gContacts.back().o1 = o1;
    gContacts.back().o2 = o2;
    swap(gContacts.back().contacts,vcontact);
  }
}

void selfCollisionCallback(void *data, dGeomID o1, dGeomID o2)
{
  ODERobot* robot = reinterpret_cast<ODERobot*>(data);
  Assert(!dGeomIsSpace(o1) && !dGeomIsSpace(o2));
  intptr_t link1 = (intptr_t)dGeomGetData(o1);
  intptr_t link2 = (intptr_t)dGeomGetData(o2);
  Assert(link1 >= 0 && (int)link1 < (int)robot->robot.links.size());
  Assert(link2 >= 0 && (int)link2 < (int)robot->robot.links.size());
  if(robot->robot.selfCollisions(link1,link2)==NULL) {
    return;
  }
  
  int num = dCollide (o1,o2,max_contacts,gContactTemp,sizeof(dContactGeom));
  vector<dContactGeom> vcontact(num);
  int numOk = 0;
  for(int i=0;i<num;i++) {
    if(gContactTemp[i].g1 == o2 && gContactTemp[i].g2 == o1) {
      printf("Swapping contact\n");
      std::swap(gContactTemp[i].g1,gContactTemp[i].g2);
      for(int k=0;k<3;k++) gContactTemp[i].normal[k]*=-1.0;
      std::swap(gContactTemp[i].side1,gContactTemp[i].side2);
    }
    Assert(gContactTemp[i].g1 == o1);
    Assert(gContactTemp[i].g2 == o2);
    vcontact[numOk] = gContactTemp[i];
    const dReal* n=vcontact[numOk].normal;
    if(Sqr(n[0])+Sqr(n[1])+Sqr(n[2]) < 0.9 || Sqr(n[0])+Sqr(n[1])+Sqr(n[2]) > 1.2) {
      printf("Warning, degenerate contact with normal %f %f %f\n",vcontact[numOk].normal[0],vcontact[numOk].normal[1],vcontact[numOk].normal[2]);
      //continue;
    }
    numOk++;
  }
  //TEMP: printing self collisions
  //if(numOk > 0) printf("%d self collision contacts between links %d and %d\n",numOk,(int)link1,(int)link2);
  vcontact.resize(numOk);
  
  if(kMergeContacts && numOk > 0) {
    MergeContacts(vcontact,kContactPosMergeTolerance,kContactOriMergeTolerance);
  }

  if(vcontact.size() > 0) {
    if(numOk != (int)vcontact.size())
    	//// The int type is not guaranteed to be big enough, use intptr_t
		//cout<<numOk<<" contacts between env "<<(int)dGeomGetData(o2)<<" and body "<<(int)dGeomGetData(o1)<<"  (clustered to "<<vcontact.size()<<")"<<endl;
		cout<<numOk<<" contacts between link "<<(intptr_t)dGeomGetData(o2)<<" and link "<<(intptr_t)dGeomGetData(o1)<<"  (clustered to "<<vcontact.size()<<")"<<endl;
    gContacts.push_back(ODEContactResult());
    gContacts.back().o1 = o1;
    gContacts.back().o2 = o2;
    swap(gContacts.back().contacts,vcontact);
  }
}

void ProcessContacts(list<ODEContactResult>::iterator start,list<ODEContactResult>::iterator end,const ODESimulatorSettings& settings,bool aggregateCount=true)
{
  if(kMergeContacts) {
    for(list<ODEContactResult>::iterator j=start;j!=end;j++) 
      MergeContacts(j->contacts,kContactPosMergeTolerance,kContactOriMergeTolerance);
  }

  static bool warnedContacts = false;
  if(aggregateCount) {
    int numContacts = 0;
    for(list<ODEContactResult>::iterator j=start;j!=end;j++) 
      numContacts += (int)j->contacts.size();
    if(numContacts > settings.maxContacts) {
      //printf("Warning: %d robot-env contacts > maximum %d, may crash\n",numContacts,settings.maxContacts);
      if(settings.maxContacts > 50) {
	if(!warnedContacts) {
	  printf("Max contacts > 50, may crash.  Press enter to continue...\n");
	  getchar();
	}
	warnedContacts = true;
      }
      Real scale = Real(settings.maxContacts)/numContacts;
      for(list<ODEContactResult>::iterator j=start;j!=end;j++) {
	int n=(int)Ceil(Real(j->contacts.size())*scale);
	//printf("Clustering %d->%d\n",j->contacts.size(),n);
	ClusterContacts(j->contacts,n,settings.clusterNormalScale);
      }
    }
  }
  else {
    for(list<ODEContactResult>::iterator j=start;j!=end;j++) {
      if(settings.maxContacts > 50) {
	if(!warnedContacts) {
	  printf("Max contacts > 50, may crash.  Press enter to continue...\n");
	  getchar();
	}
	warnedContacts = true;
      }
      for(list<ODEContactResult>::iterator j=start;j!=end;j++) {
	ClusterContacts(j->contacts,settings.maxContacts,settings.clusterNormalScale);
      }
    }
  }
}

void ODESimulator::ClearCollisions()
{
  dJointGroupEmpty(contactGroupID);
  contactList.clear();
}

void ODESimulator::GetSurfaceParameters(const ODEObjectID& a,const ODEObjectID& b,dSurfaceParameters& surface) const
{
  //TODO: base the friction on the properties of the contact point
  //completely rigid contact
  surface.mode = dContactApprox1;
  //surface.mode = 0;
  surface.bounce = 0;
  surface.bounce_vel = 0;
  
  //printf("GetSurfaceParameters a = %d,%d, b = %d,%d\n",a.type,a.index,b.type,b.index);
  ODEGeometry *ma,*mb;
  if(a.type == 0) {
    Assert(a.index < (int)envs.size());
    ma = envGeoms[a.index];
  }
  else if(a.type == 1) 
    ma = robots[a.index]->triMesh(a.bodyIndex);
  else if(a.type == 2)
    ma = objects[a.index]->triMesh();
  else Abort();
  if(b.type == 0) {
    Assert(b.index < (int)envs.size());
    mb=envGeoms[b.index];
  }
  else if(b.type == 1) 
    mb=robots[b.index]->triMesh(b.bodyIndex);
  else if(b.type == 2) 
    mb=objects[b.index]->triMesh();
  else Abort();

  const ODESurfaceProperties &propa=ma->surf(),&propb=mb->surf();
  if(!IsInf(propa.kStiffness) || !IsInf(propb.kStiffness)) {
    surface.mode |= (dContactSoftERP | dContactSoftCFM);
    Real kStiffness = 1.0/(1.0/propa.kStiffness+1.0/propb.kStiffness);
    Real kDamping = 1.0/(1.0/propa.kDamping+1.0/propb.kDamping);
    surface.soft_erp = ERPFromSpring(timestep,kStiffness,kDamping);
    surface.soft_cfm = CFMFromSpring(timestep,kStiffness,kDamping);
    //printf("Joint stiffness %g, damping %g\n",kStiffness,kDamping);
    //printf("ERP = %g, CFM = %g\n",surface.soft_erp,surface.soft_cfm);
  }
  surface.mu = 2.0/(1.0/propa.kFriction+1.0/propb.kFriction);
  //correction to account for pyramid shaped friction cone
  surface.mu *= 0.707;
  surface.bounce = 0.5*(propa.kRestitution+propb.kRestitution);
  if(surface.bounce != 0)
    surface.mode |= dContactBounce;
}

void ODESimulator::SetupContactResponse(const ODEObjectID& a,const ODEObjectID& b,int feedbackIndex,ODEContactResult& c)
{
  dContact contact;
  GetSurfaceParameters(a,b,contact.surface);
  dBodyID b1 = dGeomGetBody(c.o1);
  dBodyID b2 = dGeomGetBody(c.o2);
  c.feedback.resize(c.contacts.size());
  for(size_t k=0;k<c.contacts.size();k++) {
    //add contact joint to joint group
    contact.geom = c.contacts[k];
    Assert(contact.geom.g1 == c.o1 || contact.geom.g1 == c.o2);
    Assert(contact.geom.g2 == c.o1 || contact.geom.g2 == c.o2);
    Assert(contact.geom.depth >= 0);
    
    Assert(contact.geom.g1 == c.o1);
    dJointID joint = dJointCreateContact(worldID,contactGroupID,&contact);
    dJointSetFeedback(joint,&c.feedback[k]);
    //if(b2==0)
    //dJointAttach(joint,b2,b1);
      //else
      dJointAttach(joint,b1,b2);
  }
  //if contact feedback is enabled, do it!
  pair<ODEObjectID,ODEObjectID> cindex;
  bool reverse = false;
  if(b < a) {
    cindex.first = b;
    cindex.second = a;
    reverse = true;
  }
  else {
    cindex.first = a;
    cindex.second = b;
  }
  ODEContactList* cl=NULL;
  if(contactList.count(cindex) != 0) {
    cl=&contactList[cindex];
  }
  else {
    //check if there's a generic robot feedback list
    bool checkRobot=false;
    if(cindex.first.type == 1 && cindex.first.bodyIndex != -1) {
      cindex.first.bodyIndex=-1;
      checkRobot=true;
    }
    if(cindex.second.type == 1 && cindex.second.bodyIndex != -1) {
      cindex.second.bodyIndex=-1;
      checkRobot=true;
    }
    if(checkRobot) {
      if(contactList.count(cindex) != 0) 
	cl=&contactList[cindex];
    }
  }
  if(cl) {
    size_t start=cl->points.size();
    cl->points.resize(start+c.contacts.size());
    for(size_t k=0;k<c.contacts.size();k++) {
      Assert(k+start < cl->points.size());
      CopyVector(cl->points[k+start].x,c.contacts[k].pos);
      CopyVector(cl->points[k+start].n,c.contacts[k].normal);
      cl->points[k+start].kFriction = contact.surface.mu;
      if(reverse)
	cl->points[k+start].n.inplaceNegative();
    }
    Assert(feedbackIndex >= 0 && feedbackIndex < (int)gContacts.size());
    cl->feedbackIndices.push_back(feedbackIndex);
  }
}

void ODESimulator::DetectCollisions()
{
#if DO_TIMING
  Timer timer;
#endif //DO_TIMING

  dJointGroupEmpty(contactGroupID);
  //clear feedback structure
  for(map<pair<ODEObjectID,ODEObjectID>,ODEContactList>::iterator i=contactList.begin();i!=contactList.end();i++) {
    i->second.points.clear();
    i->second.forces.clear();
    i->second.feedbackIndices.clear();
  }
  gContacts.clear();

  pair<ODEObjectID,ODEObjectID> cindex;
  int jcount=0;
  if(settings.rigidObjectCollisions) {
    //call the collision routine between objects and the world
    dSpaceCollide(envSpaceID,(void*)this,collisionCallback);

#if DO_TIMING
    gContactDetectTime += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING
    
    ProcessContacts(gContacts.begin(),gContacts.end(),settings,false);

#if DO_TIMING
    gClusterTime += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING

    for(list<ODEContactResult>::iterator j=gContacts.begin();j!=gContacts.end();j++,jcount++) {
      //// int is not necessarily big enough, use intptr_t
      //int o1 = (int)dGeomGetData(j->o1);
      //int o2 = (int)dGeomGetData(j->o2);
      intptr_t o1 = (intptr_t)dGeomGetData(j->o1);
      intptr_t o2 = (intptr_t)dGeomGetData(j->o2);
      if(o1 < 0) {  //it's an environment
	cindex.first = ODEObjectID(0,(-o1-1));
      }
      else {
	cindex.first = ODEObjectID(2,o1);
      }
      if(o2 < 0) {  //it's an environment
	cindex.second = ODEObjectID(0,(-o2-1));
      }
      else {
	cindex.second = ODEObjectID(2,o2);
      }
      if(o1 < 0 && o2 < 0) {
	fprintf(stderr,"Warning, detecting terrain-terrain collisions?\n");
      }
      else {
	j->feedback.resize(j->contacts.size());
	SetupContactResponse(cindex.first,cindex.second,jcount,*j);
      }
    }
  }

  //do robot-environment collisions
  for(size_t i=0;i<robots.size();i++) {
    cindex.first = ODEObjectID(1,i);

#if DO_TIMING
    timer.Reset();
#endif //DO_TIMING

    //call the collision routine between the robot and the world
	bool gContactsEmpty = gContacts.empty();
	list<ODEContactResult>::iterator gContactStart;
	if(!gContactsEmpty) gContactStart = --gContacts.end();
	dSpaceCollide2((dxGeom *)robots[i]->space(),(dxGeom *)envSpaceID,(void*)this,collisionCallback);
	if(!gContactsEmpty) ++gContactStart;
	else gContactStart = gContacts.begin();

#if DO_TIMING
    gContactDetectTime += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING

    ProcessContacts(gContactStart,gContacts.end(),settings);

#if DO_TIMING
    gClusterTime += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING

    //setup the contact "joints" and contactLists
    for(list<ODEContactResult>::iterator j=gContactStart;j!=gContacts.end();j++,jcount++) {
      bool isBodyo1 = false, isBodyo2 = false;
      for(size_t k=0;k<robots[i]->robot.links.size();k++) {
	if(robots[i]->triMesh(k) && j->o1 == robots[i]->geom(k)) isBodyo1=true;
	if(robots[i]->triMesh(k) && j->o2 == robots[i]->geom(k)) isBodyo2=true;
      }
      intptr_t body,obj;
      if(isBodyo2) {
	printf("Warning, ODE collision result lists bodies in reverse order\n");
	Assert(!isBodyo1);
	body = (intptr_t)dGeomGetData(j->o2);
	obj = (intptr_t)dGeomGetData(j->o1);
      }
      else {
	Assert(!isBodyo2);
	Assert(isBodyo1);
	body = (intptr_t)dGeomGetData(j->o1);
	obj = (intptr_t)dGeomGetData(j->o2);
      }
      //printf("Collision between body %d and obj %d\n",body,obj);
      Assert(body >= 0 && body < (int)robots[i]->robot.links.size());
      Assert(obj >= -(int)envs.size() && obj < (int)objects.size());
      if(robots[i]->robot.parents[body] == -1 && obj < 0) { //fixed links
	fprintf(stderr,"Warning, colliding a fixed link and the terrain\n");
	continue;
      }
      cindex.first.bodyIndex = body;
      if(obj < 0) {  //it's an environment
	cindex.second = ODEObjectID(0,(-obj-1));
      }
      else {
	cindex.second = ODEObjectID(2,obj);
      }
      SetupContactResponse(cindex.first,cindex.second,jcount,*j);
    }

    if(settings.robotSelfCollisions) {
      robots[i]->EnableSelfCollisions(true);

#if DO_TIMING
      timer.Reset();
#endif

      cindex.second = ODEObjectID(1,i);
      gContactsEmpty = gContacts.empty();
	  if(!gContactsEmpty) gContactStart = --gContacts.end();
      //call the self collision routine for the robot
      dSpaceCollide(robots[i]->space(),(void*)robots[i],selfCollisionCallback);
      if(!gContactsEmpty) ++gContactStart;
	  else gContactStart = gContacts.begin();

#if DO_TIMING
    gContactDetectTime += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING
      
      ProcessContacts(gContactStart,gContacts.end(),settings);

#if DO_TIMING
    gClusterTime += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING

      //setup the contact "joints" and contactLists
      for(list<ODEContactResult>::iterator j=gContactStart;j!=gContacts.end();j++,jcount++) {
	intptr_t body1 = (intptr_t)dGeomGetData(j->o1);
	intptr_t body2 = (intptr_t)dGeomGetData(j->o2);
	//printf("Collision between body %d and body %d\n",body1,body2);
	Assert(body1 >= 0 && body1 < (int)robots[i]->robot.links.size());
	Assert(body2 >= 0 && body2 < (int)robots[i]->robot.links.size());
	cindex.first.bodyIndex = body1;
	cindex.second.bodyIndex = body2;
	SetupContactResponse(cindex.first,cindex.second,jcount,*j);
      }
    }

    if(settings.robotRobotCollisions) {    
      for(size_t k=i+1;k<robots.size();k++) {
	cindex.second = ODEObjectID(1,k);

#if DO_TIMING
    timer.Reset();
#endif //DO_TIMING

	gContactsEmpty = gContacts.empty();
	if(!gContactsEmpty) gContactStart = --gContacts.end();
	dSpaceCollide2((dxGeom *)robots[i]->space(),(dxGeom *)robots[k]->space(),(void*)this,collisionCallback);
	if(!gContactsEmpty) ++gContactStart;
	else gContactStart = gContacts.begin();

#if DO_TIMING
    gContactDetectTime += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING

	ProcessContacts(gContactStart,gContacts.end(),settings);

#if DO_TIMING
    gClusterTime += timer.ElapsedTime();
    timer.Reset();
#endif //DO_TIMING

	//setup the contact "joints" and contactLists
	for(list<ODEContactResult>::iterator j=gContactStart;j!=gContacts.end();j++,jcount++) {
	  intptr_t body1 = (intptr_t)dGeomGetData(j->o1);
	  intptr_t body2 = (intptr_t)dGeomGetData(j->o2);
	  //printf("Collision between robot %d and robot %d\n",i,k);
	  Assert(body1 >= 0 && body1 < (int)robots[i]->robot.links.size());
	  Assert(body2 >= 0 && body2 < (int)robots[k]->robot.links.size());
	  cindex.first.bodyIndex = body1;
	  cindex.second.bodyIndex = body2;
	  SetupContactResponse(cindex.first,cindex.second,jcount,*j);
	}
      }
    }
  }
}

void ODESimulator::EnableContactFeedback(const ODEObjectID& a,const ODEObjectID& b)
{
  pair<ODEObjectID,ODEObjectID> index;
  if(a < b) {
    index.first=a;
    index.second=b;
  }
  else {
    index.first=b;
    index.second=a;
  }
  contactList[index] = ODEContactList();
}

ODEContactList* ODESimulator::GetContactFeedback(const ODEObjectID& a,const ODEObjectID& b)
{
  pair<ODEObjectID,ODEObjectID> index;
  if(a < b) {
    index.first=a;
    index.second=b;
  }
  else {
    index.first=b;
    index.second=a;
  }
  if(contactList.count(index) != 0) {
    return &contactList[index];
  }
  return NULL;
}

bool HasContact(dBodyID a)
{
  if(a == 0) return false;
  int n = dBodyGetNumJoints (a);
  for(int i=0;i<n;i++) {
    dJointID j=dBodyGetJoint (a,i);
    if(dJointGetType(j)==dJointTypeContact) return true;
  }
  return false;
}

///Will produce bogus o1 and o2 vectors
void GetContacts(dBodyID a,vector<ODEContactList>& contacts)
{
  if(a == 0) return;

  contacts.resize(0);
  for(list<ODEContactResult>::iterator i=gContacts.begin();i!=gContacts.end();i++) {
    if(a == dGeomGetBody(i->o1) || a == dGeomGetBody(i->o2)) {
      dBodyID b = dGeomGetBody(i->o2);
      bool reverse = false;
      if(b == a) { b = dGeomGetBody(i->o1); reverse = true; }
      contacts.resize(contacts.size()+1);
      contacts.back().points.resize(i->contacts.size());
      contacts.back().forces.resize(i->feedback.size());
      for(size_t j=0;j<i->feedback.size();j++) {
	CopyVector(contacts.back().forces[j],i->feedback[j].f1);
	CopyVector(contacts.back().points[j].x,i->contacts[j].pos);
	CopyVector(contacts.back().points[j].n,i->contacts[j].normal);
	//contacts.back().points[j].kFriction = i->contacts[j].surface.mu;
	contacts.back().points[j].kFriction = 0;
	if(reverse) {
	  contacts.back().forces[j].inplaceNegative();
	  contacts.back().points[j].n.inplaceNegative();
	}
      }
    }
  }
}


bool HasContact(dBodyID a,dBodyID b)
{
  if(a == 0 && b == 0) return false; //two environments
  if(a == 0) Swap(a,b);
  int n = dBodyGetNumJoints (a);
  for(int i=0;i<n;i++) {
    dJointID j=dBodyGetJoint (a,i);
    if(dJointGetType(j)==dJointTypeContact) {
      dBodyID j0=dJointGetBody(j,0);
      dBodyID j1=dJointGetBody(j,1);
      if(j0 == b || j1 == b) return true;
    }
  }
  return false;
}

bool ODESimulator::InContact(const ODEObjectID& a) const
{
  if(a.type == 0) { //environment
    //must loop through all objects to see what is in contact with the
    //environment
    for(size_t i=0;i<objects.size();i++) {
      if(HasContact(objects[i]->body(),0)) return true;
    }
    for(size_t i=0;i<robots.size();i++) {
      for(size_t j=0;j<robots[i]->robot.links.size();j++) {
	if(HasContact(robots[i]->body(j),0)) return true;
      }
    }    
    return false;
  }
  else if(a.type == 2) {//object
    return HasContact(objects[a.index]->body());
  }
  else {
    if(a.bodyIndex >= 0)
      return HasContact(robots[a.index]->body(a.bodyIndex));
    else { //any robot link
      for(size_t i=0;i<robots[a.index]->robot.links.size();i++)
	if(HasContact(robots[a.index]->body(i))) return true;
      return false;
    }
  }
  return false;
}

bool ODESimulator::InContact(const ODEObjectID& a,const ODEObjectID& b) const
{
  vector<dBodyID> bodya, bodyb;
  if(a.type == 0) {
    bodya.push_back(0);
  }
  else if(a.type == 2) {
    bodya.push_back(objects[a.index]->body());
  }
  else {
    if(a.bodyIndex >= 0)
      bodya.push_back(robots[a.index]->body(a.bodyIndex));
    else {
      for(size_t i=0;i<robots[a.index]->robot.links.size();i++)
	if(robots[a.index]->body(i))
	  bodya.push_back(robots[a.index]->body(i));
    }
  }
  if(b.type == 0) {
    bodyb.push_back(0);
  }
  else if(b.type == 2) {
    bodyb.push_back(objects[b.index]->body());
  }
  else {
    if(b.bodyIndex >= 0)
      bodyb.push_back(robots[b.index]->body(b.bodyIndex));
    else {
      for(size_t i=0;i<robots[b.index]->robot.links.size();i++)
	if(robots[b.index]->body(i))
	  bodyb.push_back(robots[b.index]->body(i));
    }
  }
  for(size_t i=0;i<bodya.size();i++)
    for(size_t j=0;j<bodyb.size();j++)
      if(HasContact(bodya[i],bodyb[j])) return true;
  return false;
}

void ODESimulator::StepDynamics(Real dt)
{
  dWorldStep(worldID,dt);
  //dWorldQuickStep(worldID,dt);
}


bool ODESimulator::ReadState(File& f)
{
#if TEST_READ_WRITE_STATE
  for(size_t i=0;i<robots.size();i++) 
    TestReadWriteState(*robots[i],"ODERobot");
  for(size_t i=0;i<objects.size();i++) 
    TestReadWriteState(*objects[i],"ODEObject");
#endif

  for(size_t i=0;i<robots.size();i++) {
    if(!robots[i]->ReadState(f)) {
      fprintf(stderr,"ODESimulator::ReadState(): failed to read robot %d\n",i);
      return false;
    }
  }
  for(size_t i=0;i<objects.size();i++) {
    if(!objects[i]->ReadState(f)) {
      fprintf(stderr,"ODESimulator::ReadState(): failed to read object %d\n",i);
      return false;
    }
  }
  return true;
}

bool ODESimulator::WriteState(File& f) const
{
  for(size_t i=0;i<robots.size();i++) 
    if(!robots[i]->WriteState(f)) return false;
  for(size_t i=0;i<objects.size();i++) 
    if(!objects[i]->WriteState(f)) return false;
  return true;
}
