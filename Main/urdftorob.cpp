/*
 * urdftorob.cpp
 *
 *  Created on: Jun 07, 2013
 *      Author: jingru
 */
#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include "Modeling/Robot.h"
#include <string.h>
#include <KrisLibrary/utils/stringutils.h>
#include "IO/URDFConverter.h"
#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/utils/apputils.h>
#include <fstream>
#include <string.h>

using namespace std;


int URDFtoRob(AnyCollection& settings,string infile,string outfile){
  string path = GetFilePath(outfile.c_str());


  Robot robot;

  URDFConverter::useVisGeom = settings["useVisGeom"];
  URDFConverter::flipYZ = settings["flipYZ"];
  settings["packageRootPath"].as(URDFConverter::packageRootPath);
  string geomPrefix,geomExtension;
  settings["outputGeometryPrefix"].as(geomPrefix);
  settings["outputGeometryExtension"].as(geomExtension);
  robot.LoadURDF(infile.c_str());
  if(!geomExtension.empty()) {
    //change the geometry file extension
    robot.SetGeomFiles(geomPrefix.c_str(),geomExtension.c_str());
  }
  robot.Save(outfile.c_str());
  if(!geomExtension.empty()) {
    //save in absolute path
    robot.SaveGeometry(path.c_str());
  }
  return 1;
}


int main_shell(int argc, char** argv)
{
  AppUtils::ProgramSettings settings("Klampt");
  settings["useVisGeom"] = false;
  settings["flipYZ"] = false;
  settings["outputGeometryExtension"] = string("off");
  settings["outputGeometryPrefix"] = string("");
  settings["packageRootPath"] = string("");
  if(!settings.read("urdftorob.settings")) {
    LOG4CXX_INFO(KrisLibrary::logger(),"Didn't read settings from [APPDATA]/urdftorob.settings\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"Writing default settings to [APPDATA]/urdftorob.settings\n");
    settings.write("urdftorob.settings");
  }

  if (argc < 2 || argc > 3) {
    LOG4CXX_INFO(KrisLibrary::logger(),
	   "USAGE: URDFtoRob robot_file.urdf [optional filename for .rob]\n");
    return 0;
  }

  string infile=argv[1],outfile;
  
  if(argc == 2) {
    outfile.assign(argv[1]);
    StripExtension(outfile);
    outfile.append(".rob");
  }
  else if(argc == 3)
    outfile.assign(argv[2]);

  const char* ext = FileExtension(argv[1]);
  if (0 == strcmp(ext, "urdf")) {
    URDFtoRob(settings,infile,outfile);
    LOG4CXX_INFO(KrisLibrary::logger(),"Converted "<<infile<<" to "<< outfile<<"\n");
    LOG4CXX_INFO(KrisLibrary::logger(), "Done!" << "\n");
    return 1;
  }
  else {
    LOG4CXX_INFO(KrisLibrary::logger(),"Unknown file extension "<< ext<<" on file "<< argv[1]);
    return 0;
  }
}
#ifndef HAVE_QT
int main(int argc,char** argv){
    main_shell(argc,argv);
}
#endif

