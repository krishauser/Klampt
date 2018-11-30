/*
 * orXmlTags.h
 *
 *  Created on: Sep 4, 2012
 *      Author: jingru
 */

#ifndef ORXMLTAGS_H_
#define ORXMLTAGS_H_

#include <vector>
#include <string>
#include <tinyxml.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/utils/stringutils.h>
#include "PrimitiveShape.h"
#include "Constants.h"
using namespace Math3D;
using namespace std;
using namespace PrimitiveShape;
//#define DEBUGREAD


class OrXmlGeom;
class OrXmlMass;
class OrXmlJoint;
class OrXmlAdjacent;

//const char* FileExtension(const char* str);
string FileNameWithoutExt(const string& str);

//given inertia matrix Hc about c, returns inertia matrix around origin
Matrix3 TranslateInertia2Origin(const Matrix3& Hc,const Vector3& c,Real mass);

void axisAngle2RotationMat(const Vector4& aa, Matrix3& mat);
void quat2RotationMat(const Vector4& aa, Matrix3& mat);
void rotationMat2RPY(const Matrix3& mat, Vector3& vec);

TiXmlElement* getFirstChild(TiXmlElement* e, const char* value);
TiXmlElement* getNextSibling(TiXmlElement* e, const char* value);
bool hasTransformation(TiXmlElement* e);

class OrXmlTransformation {
public:
	OrXmlTransformation(TiXmlElement* element);
	OrXmlTransformation(){e = 0; T = 0;};
	~OrXmlTransformation();

	bool GetContent();
	bool ComputeTransform();
	int getTransformType(TiXmlElement* et);
	TiXmlElement* e;
	//final rigid transformation
	RigidTransform* T;

	vector<int> rotationorder; //rotation order
	vector<int> rotationindex; //index of ith rotation in its type

	vector<Vector4*> rotationaxis;
	vector<Matrix3*> rotationmat;
	vector<Vector3*> translation;
	vector<Vector4*> quat;
};

class OrXmlAdjacent{
public:
	OrXmlAdjacent(TiXmlElement* element);
	~OrXmlAdjacent();
	bool GetContent();
	TiXmlElement* e;
	string body1;
	string body2;
};

//not support hinge2,universal
//not support tag
//what is mode?
class OrXmlJoint {
public:
	OrXmlJoint(TiXmlElement* element);
	~OrXmlJoint();
	bool GetContent();
	bool GetCleanJointInfo();

	Vector3* anchor;
	Vector3* axis;
	string type; //hinge, slider, universal, hinge2, spherical
	string offsetfrom;
	string name;
	double* resolution;

	int linkI;

	bool enabled;

	vector<string> bodys;
	vector<double> limits;
	vector<double> limitsdeg;
	vector<double> limitsrad;
	vector<double> maxvel;
	vector<double> maxveldeg;
	vector<double> maxaccel;
	vector<double> maxacceldeg;
	vector<double> weight;
	vector<double> initial;
	vector<double> maxtorque;

	vector<double> cleanLimits;
	vector<double> cleanMaxVel;
	vector<double> cleanMaxAcc;
	vector<double> cleanMaxTorque;

	TiXmlElement* e;
};

class OrXmlMass {
public:
	OrXmlMass(TiXmlElement* element);
	OrXmlMass();
	OrXmlMass(OrXmlMass& newm);
	~OrXmlMass();
	bool GetContent();

	void GetBoxMass();
	void GetSphereMass();
	void GetUnitSphereMass();
	void GetCylinderMass();
	void AddMass(OrXmlMass* r);
	void ChangeCoordinateSystem( RigidTransform& trans);
	void ChangeCenterOfRotation(const Vector3& newcor);

	Vector3* extents;
	Vector3* com;
	Matrix3* inertia;
	double* density;
	double* radius;
	double* height;
	double* total;

	string type;
	TiXmlElement* e;
};

class OrXmlGeom {
public:
	OrXmlGeom(TiXmlElement* element);
	~OrXmlGeom();
	bool GetContent();
	bool Convert2Tri();

	bool applyTransformation();
	bool applyScale(bool visible);
	void loadGeom(bool visible);
	void loadWrl(bool visible);
	void loadTri(bool visible);
	void copyPrimitiveMesh(const vector<MyPoint3D>& pts, const vector<TriFaceIndex>& tis);

	Vector3* extents;
	double* height;
	double* radius;
	OrXmlTransformation* transformation;
	bool dorender;
	string render;
	string data;
	double* renderscale;
	double* datascale;

	string type;
	TiXmlElement* e;

	vector<MyPoint3D> vispoints;
	vector<TriFaceIndex> visindexes;

	vector<MyPoint3D> colpoints;
	vector<TriFaceIndex> colindexes;
};

std::ostream& operator << (std::ostream&, const OrXmlAdjacent&);
std::ostream& operator << (std::ostream&, const OrXmlJoint&);
std::ostream& operator << (std::ostream&, const OrXmlMass&);
std::ostream& operator << (std::ostream&, const OrXmlGeom&);
std::ostream& operator << (std::ostream&, const OrXmlTransformation&);

#endif /* ORXMLTAGS_H_ */
