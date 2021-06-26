#include "RandomizedSelfCollisions.h"
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/utils/ProgressPrinter.h>
#include "Planning/DistanceQuery.h"


namespace Klampt {

void SampleRobot(RobotWithGeometry& robot)
{
  for(int i=0;i<robot.q.n;i++) {
    if(!IsInf(robot.qMin(i)) && !IsInf(robot.qMax(i)))
      robot.q(i) = Rand(robot.qMin(i),robot.qMax(i));
  }
  robot.UpdateFrames();
  robot.UpdateGeometry();
}

//adds potentially colliding pairs to canCollide
//NOTE: tests only those in robot's collision pairs
void TestCollisions(RobotWithGeometry& robot,Array2D<bool>& canCollide,int numSamples)
{
  cout<<"Randomly calculating new collisions..."<<endl;
  ProgressPrinter progress(numSamples);
  for(int k=0;k<numSamples;k++) {
    progress.Update();
    SampleRobot(robot);
    for(int i=0;i<robot.q.n;i++)
      for(int j=0;j<robot.q.n;j++) {
        if(!canCollide(i,j)) {
          if(robot.SelfCollision(i,j))
            canCollide(i,j) = true;
        }
      }
  }
  progress.Done();
}

//of the collision pairs in canCollide, finds the ones that have independent
//collisions
void TestIndependentCollisions(RobotWithGeometry& robot,Array2D<bool>& canCollide,Array2D<bool>& independent,int numSamples)
{
  Array2D<bool> knownIndependent = independent;

  cout<<"Randomly calculating new independent collisions..."<<endl;
  ProgressPrinter progress(numSamples);
  for(int k=0;k<numSamples;k++) {
    progress.Update();
    SampleRobot(robot);
    int numCollisions = 0;
    int iCollide=-1,jCollide=-1;
    for(int i=0;i<robot.q.n && numCollisions<=1;i++)
      for(int j=0;j<robot.q.n && numCollisions<=1;j++) {
        if(canCollide(i,j)) {
          if(robot.SelfCollision(i,j)) {
            numCollisions++;
            iCollide=i;
            jCollide=j;
            if(independent(i,j)) numCollisions = 5;  //can quit now
          }
        }
      }
    if(numCollisions == 1) {
      assert(iCollide >= 0&& jCollide >= 0);
      independent(iCollide,jCollide) = true;
    }
  }
  progress.Done();

  int numNewPairs=0;
  for(size_t i=0;i<robot.links.size();i++) {
    for(size_t j=0;j<robot.links.size();j++) {
      if(robot.selfCollisions(i,j) && independent(i,j)) {
        if(!knownIndependent(i,j)) 
          numNewPairs++;
      }
    }
  }
  cout<<numNewPairs<<" new independent pairs"<<endl;
}



void RandomizedSelfCollisionPairs(RobotWithGeometry& robot,Array2D<bool>& collision,int numSamples)
{
  Array2D<bool> oldCollisions(robot.q.n,robot.q.n);
  for(int i=0;i<robot.q.n;i++)
    for(int j=0;j<robot.q.n;j++)
      oldCollisions(i,j) = (robot.selfCollisions(i,j) != NULL);

  robot.CleanupSelfCollisions();
  robot.InitAllSelfCollisions();
  collision.resize(robot.q.n,robot.q.n,false);
  TestCollisions(robot,collision,numSamples);

  //restore self collisions
  robot.InitSelfCollisionPairs(oldCollisions);

  int numPairs=0;
  int numNewPairs=0;
  for(int i=0;i<robot.q.n;i++) {
    for(int j=0;j<robot.q.n;j++) {
      if(collision(i,j)) numPairs++;
      if(collision(i,j) && !oldCollisions(i,j)) 
        numNewPairs++;
    }
  }
  cout<<numNewPairs<<" new pairs, "<<numPairs<<" total"<<endl;
}

void RandomizedIndependentSelfCollisionPairs(RobotWithGeometry& robot,Array2D<bool>& collision,int numSamples)
{
  Array2D<bool> oldCollisions(robot.q.n,robot.q.n);
  for(int i=0;i<robot.q.n;i++)
    for(int j=0;j<robot.q.n;j++)
      oldCollisions(i,j) = (robot.selfCollisions(i,j) != NULL);

  robot.CleanupSelfCollisions();
  robot.InitAllSelfCollisions();
  collision.resize(robot.q.n,robot.q.n,false);
  TestCollisions(robot,collision,numSamples);
  Array2D<bool> independent;
  independent.resize(robot.q.n,robot.q.n,false);
  TestIndependentCollisions(robot,collision,independent,numSamples);
  robot.InitSelfCollisionPairs(oldCollisions);

  collision = independent;
  int numPairs=0;
  int numNewPairs=0;
  for(int i=0;i<robot.q.n;i++) {
    for(int j=0;j<robot.q.n;j++) {
      if(collision(i,j)) numPairs++;
      if(collision(i,j) && !oldCollisions(i,j)) 
        numNewPairs++;
    }
  }
  cout<<numNewPairs<<" new pairs, "<<numPairs<<" total"<<endl;
}



void RandomizedSelfCollisionDistances(RobotWithGeometry& robot,Array2D<Real>& minDistance,Array2D<Real>& maxDistance,int numSamples)
{
  minDistance.resize(robot.q.n,robot.q.n,Inf);
  maxDistance.resize(robot.q.n,robot.q.n,-Inf);
  Array2D<shared_ptr<RobotWithGeometry::CollisionQuery> > queries(robot.q.n,robot.q.n);
  for(int i=0;i<robot.q.n;i++) {
    if(robot.IsGeometryEmpty(i)) continue;
    for(int j=i+1;j<robot.q.n;j++) {
      if(!robot.IsGeometryEmpty(j))
        queries(i,j).reset(new RobotWithGeometry::CollisionQuery(*robot.geometry[i],*robot.geometry[j]));
    }
  }

  //TODO: configure these
  Real absErr = 0.001;
  Real relErr = 0.01;
  for(int iters=0;iters<numSamples;iters++) {
    if(iters*100 % numSamples == 0) {
      cout<<iters*100/numSamples<<"..."; cout.flush();
    }
    SampleRobot(robot);
    for(int i=0;i<robot.q.n;i++) {
      for(int j=i+1;j<robot.q.n;j++) {
        if(!queries(i,j)) continue;
        Real d=queries(i,j)->Distance(absErr,relErr);
        minDistance(i,j) = Min(minDistance(i,j),d);
        maxDistance(i,j) = Max(maxDistance(i,j),d);
      }
    }
  }  
  //fill out the lower triangle  
  for(int i=0;i<robot.q.n;i++) {
    for(int j=0;j<i;j++) {
      minDistance(i,j) = minDistance(j,i); 
      maxDistance(i,j) = maxDistance(j,i); 
    }
    minDistance(i,i) = maxDistance(i,i) = 0;
  }
}

} //namespace Klampt