#include "ContactDistance.h"
#include <KrisLibrary/math/angle.h>
using namespace Klampt;

ContactDistanceMetric::ContactDistanceMetric()
  :normalWeight(0.3),angleWeight(0.7),wheelRollWeight(0.2),fixedWheelWeight(0.5)
{}

Real ContactDistanceMetric::operator () (const ContactFeatureMapping& a,const ContactFeatureMapping& b) const
{
  Real d=0;
  d += a.contact.x.distanceSquared(b.contact.x);
  d += normalWeight*a.contact.n.distanceSquared(b.contact.n);
  if(a.feature->GetType() != ContactFeatureBase::Point)
    d += angleWeight*Sqr(AngleDiff(a.angle,b.angle));
  if(a.feature->GetType() == ContactFeatureBase::Wheel) {
    d += wheelRollWeight*Sqr(AngleDiff(a.wheelRoll,b.wheelRoll));
    if(a.fixedWheel != b.fixedWheel)
      d += fixedWheelWeight;
  }
  return Sqrt(d);
}

Real ContactDistanceMetric::operator () (const ContactFeatureMapping& a,const ContactPoint& cp) const
{
  Real d=0;
  d += a.contact.x.distanceSquared(cp.x);
  d += normalWeight*a.contact.n.distanceSquared(cp.n);
  return d;
}
