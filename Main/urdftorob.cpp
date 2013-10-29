/*
 * urdftorob.cpp
 *
 *  Created on: Jun 07, 2013
 *      Author: jingru
 */
#include "Modeling/Robot.h"
#include <string.h>
#include <utils/stringutils.h>
#include "IO/URDFConverter.h"
#include <utils/AnyCollection.h>
#include <fstream>

class ProgramSettings : public AnyCollection
{
public:
  ProgramSettings() { }

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


int main(int argc, char** argv)
{
	ProgramSettings settings;
	settings["useVisGeom"] = false;
	settings["outputGeometryExtension"] = string("tri");
	settings["outputGeometryPrefix"] = string("");
	if(!settings.read("urdftorob.settings")) {
	  printf("Didn't read settings from urdftorob.settings\n");
	  printf("Writing default settings to urdftorob_default.settings\n");
	  settings.write("urdftorob_default.settings");
	}

	if (argc < 2 || argc > 3) {
		printf(
				"USAGE: URDFtoRob robot_file.urdf [optional filename for .rob]\n");
		return 0;
	}

	string filename;
	if(argc == 2) {
		filename.assign(argv[1]);
		StripExtension(filename);
		filename.append(".rob");
	}
	else if(argc == 3)
		filename.assign(argv[2]);
	const char* ext = FileExtension(argv[1]);
	string path = GetFilePath(argv[1]);


	if (0 == strcmp(ext, "urdf")) {
		Robot robot;

		URDFConverter::useVisGeom = settings["useVisGeom"];
		string geomPrefix,geomExtension;
		settings["outputGeometryPrefix"].as(geomPrefix);
		settings["outputGeometryExtension"].as(geomExtension);
		robot.LoadURDF(argv[1]);
		robot.Save(filename.c_str(), geomPrefix.c_str());
		robot.SaveGeometry((path+geomPrefix).c_str(),geomExtension.c_str());
	} else {
		printf("Unknown file extension %s on file %s!\nOnly converts URDF to Rob", ext, argv[1]);
		return 1;
	}
	cout<<"Converted "<<argv[1]<<" to "<< filename<<endl;
	cout << "Done!" << endl;
}
