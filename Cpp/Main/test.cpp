#include "Interface/RobotTestGUI.h"

int main(int argc,const char** argv)
{
  RobotWorld world;
  RobotTestBackend backend(&world);
  if(!backend.LoadCommandLine(argc,argv)) {
    return 1;
  }
  GLUIRobotTestGUI gui(&backend,&world);
  gui.SetWindowTitle("RobotTest");
  gui.Run();
  return 0;
}
