#ifndef MOTORCALIBRATE_H
#define MOTORCALIBRATE_H
#include "motorcalibrate.h"
#include "Modeling/Resources.h"
#include "Modeling/Robot.h"
#include "Modeling/Interpolate.h"
#include <robotics/ConstrainedDynamics.h>
#include <math/differentiation.h>
#include <math/LDL.h>
#include <optimization/Minimization.h>
#include <utils/AnyCollection.h>
#include <fstream>
#include <Timer.h>

//set this to something small if you want this to be faster
//const static size_t gMaxMilestones = 100;
static int gMaxMilestones = 100000;

static Vector3 gGravity(0,0,-9.8);
static Real gDefaultTimestep = 1e-3;
static Real gDefaultVelocityWeight = Sqr(0.005);


class MotorCalibrateSettings : public AnyCollection
{
public:
  MotorCalibrateSettings() {
  }
  bool read(const char* fn) {
    ifstream in(fn,ios::in);
    if(!in) return false;
    AnyCollection newEntries;
    if(!newEntries.read(in)) return false;
    merge(newEntries);
    return true;
  }
  bool write(const char* fn) {
    ofstream out(fn,ios::out);
    if(!out) return false;
    AnyCollection::write(out);
    out.close();
    return true;
  }
};

int main_shell(int argc,char **argv);
string motorcalibrate(AnyCollection settings);

#endif
