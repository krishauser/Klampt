#include "WorldViewProgram.h"
#include "Modeling/GeneralizedRobot.h"

int main(int argc,const char** argv)
{
  RobotWorld world;
  if(!LoadWorldCommandLine(world,argc,argv)) {
    printf("Error loading arguments\n");
    return 1;
  }
  GeneralizedRobot grobot(world);
  Robot out;
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
