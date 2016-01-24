#include "DistanceQuery.h"
#include <KrisLibrary/errors.h>
using namespace std;

const static Real defaultTolerance=0.2,defaultAbsErr=0.05,defaultRelErr=0.1;

DistanceQuery::DistanceQuery()
  :query(NULL),s(Invalid)
{
  distanceTolerance=defaultTolerance;
  distanceAbsErr=defaultAbsErr;
  distanceRelErr=defaultRelErr;
}

void DistanceQuery::NextCycle()
{
  switch(s) {
  case Far:    s=WasFar; break;
  case Close:  s=WasClose; break;
  case Contact:s=WasContact; break;
  default: break;
  }
}

Real DistanceQuery::UpdateQuery()
{
  Assert(query!=NULL);
  Real d;
  switch(s) {
  case Far:
    //printf("Warning: we seem to have already evaluated distance\n");
    return distanceTolerance;

  case Close:
    //printf("Warning: we seem to have already evaluated distance\n");
    //return query->Distance_Cached();
    return query->Distance(distanceAbsErr,distanceRelErr);

  case Contact:
    //return -query->PenetrationDepth_Cached();
    return -query->PenetrationDepth();

  case WasContact:  //check to see if we still have contact
    d=query->PenetrationDepth();
    if(d > Zero) { s = Contact;  return -d; }
    else {
      //d = query->Distance_Coherent(distanceAbsErr,distanceRelErr);
      d = query->Distance(distanceAbsErr,distanceRelErr);
      if(d < distanceTolerance) { s = Close; return d;  }
      else { s = Far; return distanceTolerance;  }
    }
    break;
    
  case WasClose: //check to see how distance has changed
    //d = query->Distance_Coherent(distanceAbsErr,distanceRelErr);
    d = query->Distance(distanceAbsErr,distanceRelErr);
    if(d > Zero) {
      if(d < distanceTolerance) { s = Close; return d;  }
      else { s = Far; return distanceTolerance; }
    }
    else { s = Contact;  return -query->PenetrationDepth(); }
    break;

  case WasFar:  //check to see if tolerance has been reached
  default:
    if(!query->WithinDistance(distanceTolerance)) {
      s = Far;  return distanceTolerance;  }
    else {
      //d=query->Distance_Coherent(distanceAbsErr,distanceRelErr);
      d=query->Distance(distanceAbsErr,distanceRelErr);
      if(d > Zero) { s = Close; return d;  }
      else { s = Contact; return -query->PenetrationDepth();  }
    }
  }
  AssertNotReached();
}

bool DistanceQuery::ClosestPoints(Vector3& cp1, Vector3& cp2, Vector3& dir)
{
  Assert(query!=NULL);
  switch(s) {
  case Far: case Close: case Contact: break;
  default:
    printf("Warning: querying closest points without an updated query\n");
    UpdateQuery();
    break;
  }

  switch(s) {
  case Far:
    cp1.setZero();
    cp2.setZero();
    dir.setZero();
    return false;
  case Close:
  case Contact:
    {
      vector<Vector3> cps1,cps2;
      query->InteractingPoints(cps1,cps2);
      Assert(cps1.size()==1);
      cp1 = cps1[0];
      cp2 = cps2[0];

      RigidTransform T1,T2;
      T1 = query->a->GetTransform();
      T2 = query->b->GetTransform();
      Vector3 cp1_world,cp2_world;
      T1.mulPoint(cp1,cp1_world);
      T2.mulPoint(cp2,cp2_world);
      dir.sub(cp1_world,cp2_world);
    }
    break;
  default:
    AssertNotReached();
  }
  return true;
}


