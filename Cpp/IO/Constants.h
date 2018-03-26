/*
 * Constants.h
 *
 *  Created on: Sep 21, 2012
 *      Author: jingru
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_
#include <string>
#include <vector>
using namespace std;

extern bool TOURDF;
extern bool LOADSimpleGEOM;
extern string ROBOT_DIR;
extern string MODEL_DIR;
extern string KINBODY_DIR;
extern string URDF_DIR;

extern double defaultmass;
extern double defaultDensity;

extern vector< vector<int> > linkgroups;
extern vector< vector<int> > drivergroups;
extern vector< double > defaultTorqueMax;
extern vector< double > defaultVelMax;
extern vector< double > defaultAccMax;

extern vector<double> defaultSP;
extern vector<double> defaultSI;
extern vector<double> defaultSD;
extern vector<double> defaultDF;

void initJaemihuboGroups();
void initHuboplusGroups();

#endif /* CONSTANTS_H_ */
