/*
 * Constants.cpp
 *
 *  Created on: Sep 21, 2012
 *      Author: jingru
 */
#include "Constants.h"
#include <KrisLibrary/math/math.h>
bool TOURDF = false;
bool LOADSimpleGEOM = false;
string ROBOT_DIR = "";
string MODEL_DIR = "";
string KINBODY_DIR = "";

string URDF_DIR = "data/hubo/urdf/";

double defaultmass = 1;
double defaultDensity = 1;

vector< vector<int> > linkgroups;
vector< vector<int> > drivergroups;
vector< double > defaultTorqueMax;
vector< double > defaultVelMax;
vector< double > defaultAccMax;

vector<double> defaultSP;
vector<double> defaultSI;
vector<double> defaultSD;
vector<double> defaultDF;

double dAcc = 2;
double dVel = 2;

void initHuboplusGroups(){
	vector<int> groupsize;
	groupsize.push_back(6); // torso
	groupsize.push_back(2); // head
	groupsize.push_back(6); // leftArm
	groupsize.push_back(3); // leftThumb
	groupsize.push_back(3); // leftPinky
	groupsize.push_back(3); // leftRing
	groupsize.push_back(3); // leftMiddle
	groupsize.push_back(3); // leftIndex
	groupsize.push_back(6); // rightArm
	groupsize.push_back(3); // rightThumb
	groupsize.push_back(3); // rightPinky
	groupsize.push_back(3); // rightRing
	groupsize.push_back(3); // rightMiddle
	groupsize.push_back(3); // rightIndex
	groupsize.push_back(1); // hip
	groupsize.push_back(6); // leftLeg
	groupsize.push_back(6); // rightLeg

	linkgroups.resize(groupsize.size());
	int base = 0;
	for(int i = 0; i < linkgroups.size(); i++){
		for(int j = 0; j < groupsize[i]; j++){
			linkgroups[i].push_back(base++);
		}
	}

	groupsize.clear();
	groupsize.push_back(2); // head
	groupsize.push_back(6); // leftArm
	groupsize.push_back(3); // leftThumb
	groupsize.push_back(3); // leftPinky
	groupsize.push_back(3); // leftRing
	groupsize.push_back(3); // leftMiddle
	groupsize.push_back(3); // leftIndex
	groupsize.push_back(6); // rightArm
	groupsize.push_back(3); // rightThumb
	groupsize.push_back(3); // rightPinky
	groupsize.push_back(3); // rightRing
	groupsize.push_back(3); // rightMiddle
	groupsize.push_back(3); // rightIndex
	groupsize.push_back(1); // hip
	groupsize.push_back(6); // leftLeg
	groupsize.push_back(6); // rightLeg

	drivergroups.resize(groupsize.size());
	base = 0;
	for(int i = 0; i < drivergroups.size(); i++){
		for(int j = 0; j < groupsize[i]; j++){
			drivergroups[i].push_back(base++);
		}
	}

	defaultTorqueMax.resize(linkgroups.size());
	int index = 0;
	defaultTorqueMax[index] = 0; index++;
	defaultTorqueMax[index] = 10; index++; //head
	defaultTorqueMax[index] = 50; index++; // special: [3] = 20; [4,5] = 10
	defaultTorqueMax[index] = 0.3; index++;
	defaultTorqueMax[index] = 0.3; index++;
	defaultTorqueMax[index] = 0.3; index++;
	defaultTorqueMax[index] = 0.3; index++;
	defaultTorqueMax[index] = 0.3; index++;
	defaultTorqueMax[index] = 50; index++; // special: [3] = 20; [4,5] = 10
	defaultTorqueMax[index] = 0.3; index++;
	defaultTorqueMax[index] = 0.3; index++;
	defaultTorqueMax[index] = 0.3; index++;
	defaultTorqueMax[index] = 0.3; index++;
	defaultTorqueMax[index] = 0.3; index++;
	defaultTorqueMax[index] = 50; index++;
	defaultTorqueMax[index] = 100; index++;
	defaultTorqueMax[index] = 100; index++;

	defaultVelMax.resize(linkgroups.size());
	index = 0;
	defaultVelMax[index] = Math::Inf; index++;
	defaultVelMax[index] = dVel; index++; //head
	defaultVelMax[index] = dVel; index++; //leftarm
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++; //rightarm
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++; //hip
	defaultVelMax[index] = dVel; index++; //leftleg
	defaultVelMax[index] = dVel; index++; //rightleg

	defaultAccMax.resize(linkgroups.size());
	index = 0;
	defaultAccMax[index] = Math::Inf; index++;
	defaultAccMax[index] = dAcc; index++; //head
	defaultAccMax[index] = dAcc; index++; // leftarm
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++; //rightarm
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++; //hip
	defaultAccMax[index] = dAcc; index++; //leftleg
	defaultAccMax[index] = dAcc; index++; //rightleg

	defaultSP.resize(drivergroups.size());
	index = 0;
	defaultSP[index] = 300; index++;
	defaultSP[index] = 200; index++; //special [3,4,5] = 50
	defaultSP[index] = 1; index++;
	defaultSP[index] = 1; index++;
	defaultSP[index] = 1; index++;
	defaultSP[index] = 1; index++;
	defaultSP[index] = 1; index++;
	defaultSP[index] = 200; index++; //special [3,4,5] = 50
	defaultSP[index] = 1; index++;
	defaultSP[index] = 1; index++;
	defaultSP[index] = 1; index++;
	defaultSP[index] = 1; index++;
	defaultSP[index] = 1; index++;
	defaultSP[index] = 200; index++; //hip
	defaultSP[index] = 400; index++;
	defaultSP[index] = 400; index++;

	defaultSI.resize(drivergroups.size());
	index = 0;
	defaultSI[index] = 40; index++; //head
	defaultSI[index] = 50; index++; //special [3,4,5] = 20
	defaultSI[index] = 0.2; index++;
	defaultSI[index] = 0.2; index++;
	defaultSI[index] = 0.2; index++;
	defaultSI[index] = 0.2; index++;
	defaultSI[index] = 0.2; index++;
	defaultSI[index] = 50; index++; //special [3,4,5] = 20
	defaultSI[index] = 0.2; index++;
	defaultSI[index] = 0.2; index++;
	defaultSI[index] = 0.2; index++;
	defaultSI[index] = 0.2; index++;
	defaultSI[index] = 0.2; index++;
	defaultSI[index] = 20; index++; //hip
	defaultSI[index] = 60; index++; //special [4,5] = 20
	defaultSI[index] = 60; index++; //special [4,5] = 20

	defaultSD.resize(drivergroups.size());
	index = 0;
	defaultSD[index] = 10; index++;
	defaultSD[index] = 5; index++;
	defaultSD[index] = 0.1; index++;
	defaultSD[index] = 0.1; index++;
	defaultSD[index] = 0.1; index++;
	defaultSD[index] = 0.1; index++;
	defaultSD[index] = 0.1; index++;
	defaultSD[index] = 5; index++;
	defaultSD[index] = 0.1; index++;
	defaultSD[index] = 0.1; index++;
	defaultSD[index] = 0.1; index++;
	defaultSD[index] = 0.1; index++;
	defaultSD[index] = 0.1; index++;
	defaultSD[index] = 10; index++;
	defaultSD[index] = 15; index++; //special [4,5] = 10
	defaultSD[index] = 15; index++; //special [4,5] = 10

	defaultDF.resize(drivergroups.size());
	index = 0;
	defaultDF[index] = 1; index++;
	defaultDF[index] = 1; index++; //special [3, 4, 5]  0.5
	defaultDF[index] = 0.1; index++;
	defaultDF[index] = 0.1; index++;
	defaultDF[index] = 0.1; index++;
	defaultDF[index] = 0.1; index++;
	defaultDF[index] = 0.1; index++;
	defaultDF[index] = 1; index++; //special [3, 4, 5]  0.5
	defaultDF[index] = 0.1; index++;
	defaultDF[index] = 0.1; index++;
	defaultDF[index] = 0.1; index++;
	defaultDF[index] = 0.1; index++;
	defaultDF[index] = 0.1; index++;
	defaultDF[index] = 2; index++;
	defaultDF[index] = 3; index++;
	defaultDF[index] = 3; index++;
}

void initJaemihuboGroups(){
	vector<int> groupsize;
	groupsize.push_back(6); // torso
	groupsize.push_back(1); // rightTorsoDummy
	groupsize.push_back(5); // rightArm
	groupsize.push_back(3); // rightPalm
	groupsize.push_back(3); // rightThumb
	groupsize.push_back(3); // rightPinky
	groupsize.push_back(3); // rightRing
	groupsize.push_back(3); // rightMiddle
	groupsize.push_back(3); // rightIndex
	groupsize.push_back(1); // leftTorsoDummy
	groupsize.push_back(5); // leftArm
	groupsize.push_back(3); // leftPalm
	groupsize.push_back(3); // leftThumb
	groupsize.push_back(3); // leftPinky
	groupsize.push_back(3); // leftRing
	groupsize.push_back(3); // leftMiddle
	groupsize.push_back(3); // leftIndex
	groupsize.push_back(2); // head
	groupsize.push_back(2); // hip hipDummyRight
	groupsize.push_back(6); // rightLeg
	groupsize.push_back(1); // hipDummyLeft
	groupsize.push_back(6); // leftLeg

	linkgroups.resize(22);
	int base = 0;
	for(int i = 0; i < linkgroups.size(); i++){
		for(int j = 0; j < groupsize[i]; j++){
			linkgroups[i].push_back(base++);
		}
	}

	groupsize.clear();
	groupsize.push_back(5); // rightArm
	groupsize.push_back(2); // rightPalm
	groupsize.push_back(3); // rightThumb
	groupsize.push_back(3); // rightPinky
	groupsize.push_back(3); // rightRing
	groupsize.push_back(3); // rightMiddle
	groupsize.push_back(3); // rightIndex
	groupsize.push_back(5); // leftArm
	groupsize.push_back(2); // leftPalm
	groupsize.push_back(3); // leftThumb
	groupsize.push_back(3); // leftPinky
	groupsize.push_back(3); // leftRing
	groupsize.push_back(3); // leftMiddle
	groupsize.push_back(3); // leftIndex
	groupsize.push_back(2); // head
	groupsize.push_back(1); // hip
	groupsize.push_back(6); // rightLeg
	groupsize.push_back(6); // leftLeg
	drivergroups.resize(groupsize.size());
	base = 0;
	for(int i = 0; i < drivergroups.size(); i++){
		for(int j = 0; j < groupsize[i]; j++){
			drivergroups[i].push_back(base++);
		}
	}

	defaultTorqueMax.resize(linkgroups.size());
	int index = 0;
	defaultTorqueMax[index] = 0; index++;
	defaultTorqueMax[index] = Math::Inf; index++;
	defaultTorqueMax[index] = 50;  index++;
	defaultTorqueMax[index] = 10; index++; // special: [2] = Inf;
	defaultTorqueMax[index] = 0.5; index++;
	defaultTorqueMax[index] = 0.5; index++;
	defaultTorqueMax[index] = 0.5; index++;
	defaultTorqueMax[index] = 0.5; index++;
	defaultTorqueMax[index] = 0.5; index++;
	defaultTorqueMax[index] = Math::Inf; index++;
	defaultTorqueMax[index] = 50;  index++;
	defaultTorqueMax[index] = 10; index++; // special: [2] = Inf;
	defaultTorqueMax[index] = 0.5; index++;
	defaultTorqueMax[index] = 0.5; index++;
	defaultTorqueMax[index] = 0.5; index++;
	defaultTorqueMax[index] = 0.5; index++;
	defaultTorqueMax[index] = 0.5; index++;
	defaultTorqueMax[index] = 10; index++;
	defaultTorqueMax[index] = 50; index++; // special: [1] = Inf;
	defaultTorqueMax[index] = 100; index++;
	defaultTorqueMax[index] = Math::Inf; index++;
	defaultTorqueMax[index] = 100; index++;

	double dVel = 1;
	defaultVelMax.resize(linkgroups.size());
	index = 0;
	defaultVelMax[index] = Math::Inf; index++;
	defaultVelMax[index] = 0; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++; // special: [2] = 0;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = 0; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++; // special: [2] = 0;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = dVel; index++; // special: [1] = 0;
	defaultVelMax[index] = dVel; index++;
	defaultVelMax[index] = 0; index++;
	defaultVelMax[index] = dVel; index++;

	defaultAccMax.resize(linkgroups.size());
	index = 0;
	defaultAccMax[index] = Math::Inf; index++;
	defaultAccMax[index] = 0; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++; // special: [2] = 0;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = 0; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++; // special: [2] = 0;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = dAcc; index++; // special: [1] = 0;
	defaultAccMax[index] = dAcc; index++;
	defaultAccMax[index] = 0; index++;
	defaultAccMax[index] = dAcc; index++;

	defaultSP.resize(drivergroups.size());
	index = 0;
	defaultSP[index] = 100; index++;
	defaultSP[index] = 50; index++;
	defaultSP[index] = 10; index++;
	defaultSP[index] = 10; index++;
	defaultSP[index] = 10; index++;
	defaultSP[index] = 10; index++;
	defaultSP[index] = 10; index++;
	defaultSP[index] = 100; index++;
	defaultSP[index] = 50; index++;
	defaultSP[index] = 10; index++;
	defaultSP[index] = 10; index++;
	defaultSP[index] = 10; index++;
	defaultSP[index] = 10; index++;
	defaultSP[index] = 10; index++;
	defaultSP[index] = 50; index++;
	defaultSP[index] = 100; index++;
	defaultSP[index] = 100; index++; //special [4,5] = 50
	defaultSP[index] = 100; index++; //special [4,5] = 50

	defaultSI.resize(drivergroups.size());
	index = 0;
	defaultSI[index] = 40; index++;
	defaultSI[index] = 5; index++;
	defaultSI[index] = 1; index++;
	defaultSI[index] = 1; index++;
	defaultSI[index] = 1; index++;
	defaultSI[index] = 1; index++;
	defaultSI[index] = 1; index++;
	defaultSI[index] = 40; index++;
	defaultSI[index] = 5; index++;
	defaultSI[index] = 1; index++;
	defaultSI[index] = 1; index++;
	defaultSI[index] = 1; index++;
	defaultSI[index] = 1; index++;
	defaultSI[index] = 1; index++;
	defaultSI[index] = 5; index++;
	defaultSI[index] = 5; index++;
	defaultSI[index] = 60; index++; //special [4,5] = 20
	defaultSI[index] = 60; index++; //special [4,5] = 20

	defaultSD.resize(drivergroups.size());
	index = 0;
	defaultSD[index] = 7; index++;
	defaultSD[index] = 5; index++;
	defaultSD[index] = 1; index++;
	defaultSD[index] = 1; index++;
	defaultSD[index] = 1; index++;
	defaultSD[index] = 1; index++;
	defaultSD[index] = 1; index++;
	defaultSD[index] = 7; index++;
	defaultSD[index] = 5; index++;
	defaultSD[index] = 1; index++;
	defaultSD[index] = 1; index++;
	defaultSD[index] = 1; index++;
	defaultSD[index] = 1; index++;
	defaultSD[index] = 1; index++;
	defaultSD[index] = 5; index++;
	defaultSD[index] = 5; index++;
	defaultSD[index] = 7; index++; //special [4,5] = 4
	defaultSD[index] = 7; index++; //special [4,5] = 4

	defaultDF.resize(drivergroups.size());
	index = 0;
	defaultDF[index] = 2; index++;
	defaultDF[index] = 1; index++;
	defaultDF[index] = 0.01; index++;
	defaultDF[index] = 0.01; index++;
	defaultDF[index] = 0.01; index++;
	defaultDF[index] = 0.01; index++;
	defaultDF[index] = 0.01; index++;
	defaultDF[index] = 2; index++;
	defaultDF[index] = 1; index++;
	defaultDF[index] = 0.01; index++;
	defaultDF[index] = 0.01; index++;
	defaultDF[index] = 0.01; index++;
	defaultDF[index] = 0.01; index++;
	defaultDF[index] = 0.01; index++;
	defaultDF[index] = 1; index++;
	defaultDF[index] = 2; index++;
	defaultDF[index] = 3; index++;
	defaultDF[index] = 3; index++;

}

