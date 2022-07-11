#include "Interface/SimTestGUI.h"
#include <stdio.h>
using namespace Klampt;

int main(int argc,const char** argv)
{
  WorldModel world;
  SimTestBackend backend(&world);
  if(!backend.LoadAndInitSim(argc,argv)) {
    return 1;
  }
  GLUISimTestGUI gui(&backend,&world);
  gui.SetWindowTitle("SimTest");
  gui.Run();
  return 0;
}
