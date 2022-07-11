#include "Interface/WorldViewProgram.h"
#include "Modeling/GeneralizedRobot.h"
using namespace Klampt;

int main(int argc,const char** argv)
{
  WorldModel world;
  if(!LoadWorldCommandLine(world,argc,argv)) {
    printf("Error loading arguments\n");
    return 1;
  }
  GeneralizedRobotModel grobot(world);
  RobotModel out;
  grobot.GetMegaRobot(out);
  printf("Saving to merged.rob, geometry to merged/...\n");
  for(size_t i=0;i<out.geomFiles.size();i++) {
    if(!out.geomFiles[i].empty())
      out.geomFiles[i] = string("merged/")+out.geomFiles[i];
  }
  out.Save("merged.rob");
  out.SaveGeometry("");
  return 0;
}
