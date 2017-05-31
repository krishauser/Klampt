#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "WorldViewProgram.h"
#include "Modeling/GeneralizedRobot.h"

int main(int argc,const char** argv)
{
  RobotWorld world;
  WorldViewProgram program(&world);
  if(!program.LoadCommandLine(argc,argv)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"Error loading arguments\n");
    return 1;
  }
  GeneralizedRobot grobot(*program.world);
  Robot out;
  grobot.GetMegaRobot(out);
  LOG4CXX_INFO(KrisLibrary::logger(),"Saving to merged.rob, geometry to merged/...\n");
  for(size_t i=0;i<out.geomFiles.size();i++) {
    if(!out.geomFiles[i].empty())
      out.geomFiles[i] = string("merged/")+out.geomFiles[i];
  }
  out.Save("merged.rob");
  out.SaveGeometry("");
  return 0;
}
