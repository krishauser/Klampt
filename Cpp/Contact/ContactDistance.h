#ifndef CONTACT_DISTANCE_H
#define CONTACT_DISTANCE_H

#include "ContactFeatureMapping.h"

struct ContactDistanceMetric
{
  ContactDistanceMetric();
  Real operator () (const ContactFeatureMapping& a,const ContactFeatureMapping& b) const ;
  Real operator () (const ContactFeatureMapping& a,const ContactPoint& cp) const;

  Real normalWeight,angleWeight,wheelRollWeight,fixedWheelWeight;
};

#endif
