#ifndef PLANNING_SELF_TEST_H
#define PLANNING_SELF_TEST_H

#include "RobotCSpace.h"
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include "Modeling/DynamicPath.h"

namespace Klampt {

//tests shortcutting on randomly generated paths between a and b
void TestShortcutting(SingleRobotCSpace* cspace,MotionPlannerFactory& plannerFactory,const Config& a,const Config& b);

//tests the anytime shortcutting procedure
void TestDynamicShortcutting(SingleRobotCSpace& freeSpace,const ParabolicRamp::DynamicPath& porig);

} //namespace Klampt

#endif
