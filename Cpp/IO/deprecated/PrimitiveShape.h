/*
 * LadderGenerator.h
 *
 *  Created on: Aug 29, 2012
 *      Author: jingru
 */

#ifndef PRIMITIVESHAPE_H_
#define PRIMITIVESHAPE_H_

#include <vector>
#include <math.h>
#include <string.h>
#include <sstream>

namespace PrimitiveShape{

using namespace std;

#define PI	3.14159265358979323846

class MyPoint3D;
class TriFaceIndex;

void write2Wrl(string filename, vector<MyPoint3D>& points, vector<TriFaceIndex>& indexes);
void loadTri(string filename, vector<MyPoint3D>& points, vector<TriFaceIndex>& indexes);
void write2Tri(string filename, vector<MyPoint3D>& points, vector<TriFaceIndex>& indexes);
void loadWrl(string filename, vector<MyPoint3D>& points, vector<TriFaceIndex>& indexes);


class MyPoint3D{
public:
	MyPoint3D();
	MyPoint3D(const MyPoint3D &p);
	MyPoint3D(double d0,double d1,double d2);
	~MyPoint3D(){};
	void Set(const MyPoint3D& p);
	void Set(double d0,double d1,double d2);

	double data[3];
};

class TriFaceIndex{
public:
	TriFaceIndex(){};
	TriFaceIndex(const TriFaceIndex& rfIndex);
	TriFaceIndex(const int* _indexes);
	TriFaceIndex(int i0,int i1, int i2);
	void Set(int i0,int i1, int i2);
	~TriFaceIndex();
	void AddBase(int base);

	int data[3];
};

class ShapeBase{
public:
	ShapeBase(){};
	~ShapeBase(){};
	vector<MyPoint3D> points;
	vector<TriFaceIndex> indexes;
};
//axis-aligned box
class Box : public ShapeBase{
public:
	Box(){};
	Box(double _x, double _y, double _z);
	~Box(){};

	double half_x;
	double half_z;
	double half_y;
};

class Cylinder : public ShapeBase{
public:
	Cylinder(){};
	Cylinder( double _radius, double _height, int _n=20);
	~Cylinder(){}
	void RotateX(const MyPoint3D &ori,MyPoint3D &p);

	int n; //number of points along the circle
	double radius;
	double half_height;
};

class Sphere : public ShapeBase{
public:
	Sphere(){};
	Sphere(double _radius);
	~Sphere(){};
	void readInUnitSphere();
	double radius;
};

}//end of namespace

#endif /* LADDERGENERATOR_H_ */
