/*
 * LadderGenerator.cpp
 *
 *  Created on: Aug 29, 2012
 *      Author: jingru
 */
#include <KrisLibrary/math3d/primitives.h>
#include "PrimitiveShape.h"
#include <fstream>
#include <iostream>

namespace PrimitiveShape{

MyPoint3D::MyPoint3D(){
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
}
MyPoint3D::MyPoint3D(const MyPoint3D& p){
	data[0] = p.data[0];
	data[1] = p.data[1];
	data[2] = p.data[2];
}
MyPoint3D::MyPoint3D(double d0,double d1,double d2){
	data[0] = d0;
	data[1] = d1;
	data[2] = d2;
}
void MyPoint3D::Set(double d0,double d1,double d2){
	data[0] = d0;
	data[1] = d1;
	data[2] = d2;
}
void MyPoint3D::Set(const MyPoint3D& p){
	data[0] = p.data[0];
	data[1] = p.data[1];
	data[2] = p.data[2];
}

TriFaceIndex::TriFaceIndex(const TriFaceIndex& rfIndex){
	for(int i = 0; i < 3; i++){
		data[i] = rfIndex.data[i];
	}
}

TriFaceIndex::TriFaceIndex(const int* _indexes){
	for(int i = 0; i < 3; i++){
		data[i] = _indexes[i];
	}
}

TriFaceIndex::TriFaceIndex(int i0,int i1, int i2){
	data[0] = i0;
	data[1] = i1;
	data[2] = i2;
}

void TriFaceIndex::Set(int i0,int i1, int i2){
	data[0] = i0;
	data[1] = i1;
	data[2] = i2;
}


void TriFaceIndex::AddBase(int base){
	for(int i = 0; i < 3; i++)
		data[i] += base;
}

TriFaceIndex::~TriFaceIndex(){
}

Box::Box(double _x, double _y, double _z){
	half_x = _x;
	half_y = _y;
	half_z = _z;

	MyPoint3D p1(0+half_x,0-half_y,0-half_z);
	points.push_back(p1);
	p1.Set(0+half_x,0+half_y,0-half_z);
	points.push_back(p1);
	p1.Set(0-half_x,0+half_y,0-half_z);
	points.push_back(p1);
	p1.Set(0-half_x,0-half_y,0-half_z);
	points.push_back(p1);

	p1.Set(0+half_x,0-half_y,0+half_z);
	points.push_back(p1);
	p1.Set(0+half_x,0+half_y,0+half_z);
	points.push_back(p1);
	p1.Set(0-half_x,0+half_y,0+half_z);
	points.push_back(p1);
	p1.Set(0-half_x,0-half_y,0+half_z);
	points.push_back(p1);

	TriFaceIndex fi(3,1,0);
	indexes.push_back(fi);
	fi.Set(3,2,1);
	indexes.push_back(fi);
	fi.Set(4,5,7);
	indexes.push_back(fi);
	fi.Set(5,6,7);
	indexes.push_back(fi);
	for(int i = 0; i < 4-1; i++){
		TriFaceIndex tmpfi(i,i+1,i+1+4);
		indexes.push_back(tmpfi);
		tmpfi.Set(i,i+5,i+4);
		indexes.push_back(tmpfi);
	}
	TriFaceIndex tmpfi(3,0,4);
	indexes.push_back(tmpfi);
	tmpfi.Set(3,4,7);
	indexes.push_back(tmpfi);
}

void Cylinder::RotateX(const MyPoint3D &ori,MyPoint3D &p){
	double data[4][4];
	double angle = PI * 3.0 / 2.0;
	data[0][0] = 1;
	data[0][1] = 0;
	data[0][2] = 0;
//	data[0][3] = 0;
	data[1][0] = 0;
	data[1][1] = cos(angle);
	data[1][2] = -sin(angle);
//	data[0][3] = 0;//-p.data[2];
	data[2][0] = 0;
	data[2][1] = sin(angle);
	data[2][2] = cos(angle);
//	data[2][3] = 0;//p.data[2];
	data[3][0] = 0;
	data[3][1] = 0;
	data[3][2] = 0;
//	data[3][3] = 1;

	MyPoint3D p1(p);
	for(int i = 0; i < 3; i++){
		p.data[i] = data[i][0]*p1.data[0]+data[i][1]*p1.data[1]+data[i][2]*p1.data[2];//+data[i][3]*1;
	}
	p.data[1] -= ori.data[2];
	p.data[2] += ori.data[2];
}

Cylinder::Cylinder(double _radius, double _height, int _n){
	radius = _radius;
	half_height = _height/2.0;
	n = _n;
	double theta = 2*PI / (double)n;
	MyPoint3D p;
	for(int i = 0; i < n; i++){
		double angle = theta * i;
		p.Set(radius*cos(angle), 0-half_height, radius*sin(angle));
		points.push_back(p);
	}
	for(int i = 0; i < n; i++){
		double angle = theta*i;
		MyPoint3D p(radius*cos(angle), 0+half_height, radius*sin(angle));
		points.push_back(p);
	}
	TriFaceIndex fi;
	for(int i = 0 ; i < n - 2 ; i++){
		fi.Set(0,i+1,i+2);
		indexes.push_back(fi);
	}
	for(int i = 0 ; i < n - 2 ; i++){
		fi.Set(n+i+2,n+i+1,n);
		indexes.push_back(fi);
	}
	for(int i = 0; i < n-1; i++){
		fi.Set(i+n,i+1,i);
		indexes.push_back(fi);
		fi.Set(n+i,n+i+1,i+1);
		indexes.push_back(fi);
	}
	fi.Set(n+n-1,0,n-1);
	indexes.push_back(fi);
	fi.Set(n+n-1,n,0);
	indexes.push_back(fi);
}

Sphere::Sphere(double _radius){
	radius = _radius;
	readInUnitSphere();
	if(radius > 1.0){
		Math3D::Matrix3 tran;
		tran.setIdentity();
		tran.data[0][0] = radius;
		tran.data[1][1] = radius;
		tran.data[2][2] = radius;
		for(int i = 0; i < points.size(); i++){
			Math3D::Vector3 tmp(points[i].data[0],points[i].data[1],points[i].data[2]);
			tran.mul(tmp,tmp);
			points[i].Set(tmp.x,tmp.y,tmp.z);
		}
	}
}

void Sphere::readInUnitSphere(){
	ifstream file;
	file.open("data/objects/sphere.tri");
	if(!file.is_open()){
		cout<<"Cannot read in data/objects/sphere.tri!\n"<<flush;
		getchar();
	}
	int nPoint;
	file >> nPoint;
	for(int i = 0; i < nPoint; i++){
		double x,y,z;
		file >> x;
		file >> y;
		file >> z;
		MyPoint3D p(x,y,z);
		points.push_back(p);
	}
	int nIndex;
	file >> nIndex;
	for(int i = 0; i < nIndex; i++){
		int i0,i1,i2;
		file >> i0;
		file >> i1;
		file >> i2;
		TriFaceIndex ti(i0,i1,i2);
		indexes.push_back(ti);
	}
	file.close();
}



void loadWrl(string filename, vector<MyPoint3D>& points, vector<TriFaceIndex>& indexes){
	ifstream file;
	file.open(filename.c_str());
	if(!file.is_open()){
		cout<<"Cannot read in wrl "<<filename<<" geometry!\n"<<flush;
		getchar();
	}

	int colbaseIndex = points.size();
	string line;
	while(!file.eof()){
		string str;
		file >> str;
		if(strcmp(str.c_str(),"point")==0){
			file >> str;
			if(strcmp(str.c_str(),"[")==0){
				double p[3];
				file >> str;
				while(strcmp(str.c_str(),"]") != 0){
					for(int i = 0; i < 3; i++){
						if(strcmp(str.c_str(),",") == 0){
							file >> p[i];
						}else{
							stringstream ss;
							ss << str;
							ss >> p[i];
						}
						file >> str;
					}
					MyPoint3D pt(p[0],p[1],p[2]);
					points.push_back(pt);
				}
			}
		}
		if(strcmp(str.c_str(),"coordIndex")==0){
			file >> str;
			if(strcmp(str.c_str(),"[")==0){
				getline(file,str);
				while(str.empty() || str.length() < 3 || str.find(',')>= str.length()){
					getline(file,str);
				}
				while(str.find("]") < 0 || str.find("]") >= str.length()){
					char* tmp;
					char* strstr = new char[str.length()+1];
					strcpy(strstr,str.c_str());
					tmp = strtok(strstr," ,");
					int p[4];
					int ind = 0;
					while(tmp !=NULL && ind < 4){
						p[ind] = atoi(tmp);
						ind++;
						tmp = strtok(NULL," ,");
						if(ind % 4 == 0){
							if(p[0] == p[1] || p[1] == p[2] || p[0] == p[2]){
//								continue;
							}else{
								{
									TriFaceIndex ti(p[0]+colbaseIndex,p[1]+colbaseIndex,p[2]+colbaseIndex);
									indexes.push_back(ti);
//									cout<<p[0]+colbaseIndex<<","<<p[1]+colbaseIndex<<","<<p[2]+colbaseIndex<<endl;getchar();
								}
							}
							ind = 0;
						}
					}
					delete []strstr;
					getline(file,str);
//					if(p[0] == p[1] || p[1] == p[2] || p[0] == p[2])
//						continue;
//					else{
//						{
//							TriFaceIndex ti(p[0]+colbaseIndex,p[1]+colbaseIndex,p[2]+colbaseIndex);
//							indexes.push_back(ti);
//							cout<<p[0]+colbaseIndex<<","<<p[1]+colbaseIndex<<","<<p[2]+colbaseIndex<<endl;getchar();
//						}
//					}
				}
			}
		}
	}
	file.close();
}

void write2Tri(string filename, vector<MyPoint3D>& points, vector<TriFaceIndex>& indexes) {
	ofstream file;
	if (points.size() > 0) {
		file.open(filename.c_str());
		file << points.size() << endl;
		for (int i = 0; i < points.size(); i++) {
			file << points[i].data[0] << "\t" << points[i].data[1] << "\t"
					<< points[i].data[2] << endl;
		}
		file << indexes.size() << endl;
		for (int i = 0; i < indexes.size(); i++) {
			file << indexes[i].data[0] << "\t" << indexes[i].data[1]
					<< "\t" << indexes[i].data[2] << endl;
		}
		file.close();
	}
}

void loadTri(string filename, vector<MyPoint3D>& points, vector<TriFaceIndex>& indexes){
	ifstream file;
	file.open(filename.c_str());
	if(!file.is_open()){
		cout<<"Cannot read in tri "<<filename<<" geometry!\n"<<flush;
		getchar();
	}
	int colbaseIndex = points.size();
	int nPoint;
	file >> nPoint;
	for(int i = 0; i < nPoint; i++){
		double x,y,z;
		file >> x;
		file >> y;
		file >> z;
		MyPoint3D p(x,y,z);
		points.push_back(p);
	}
	int nIndex;
	file >> nIndex;
	for(int i = 0; i < nIndex; i++){
		int i0,i1,i2;
		file >> i0;
		file >> i1;
		file >> i2;
//		TriFaceIndex ti(i0,i1,i2);
//		if(visible)
//			visindexes.push_back(ti);
//		else
//			colindexes.push_back(ti);
		{
			TriFaceIndex ti(i0+colbaseIndex,i1+colbaseIndex,i2+colbaseIndex);
			indexes.push_back(ti);
		}
	}
	file.close();
}

void write2Wrl(string filename, vector<MyPoint3D>& points, vector<TriFaceIndex>& indexes) {
	ofstream file;
	if (points.size() > 0) {
		file.open(filename.c_str());
		file << "#VRML V2.0 utf8\n\n";
		file << "DEF mesh\n";
		file << "Transform { \n \t children [ \n \t\t Shape { \n";
		//appearance
		file << "\t\t\t appearance Appearance{ \n";
		file << "\t\t\t\t material Material { \n";
		file << "\t\t\t\t\t ambientIntensity 1.000000 \n";
		file << "\t\t\t\t\t diffuseColor 0.800000 0.800000 0.500000 \n";
		file << "\t\t\t\t\t specularColor 0.800000 0.850000 0.800000 \n";
		file << "\t\t\t\t\t shininess 0.300000 \n";
		file << "\t\t\t\t } \n";
		file << "\t\t\t } \n";

		//geometry
		file << "\t\t\t geometry IndexedFaceSet { \n";
		file << "\t\t\t\t coord Coordinate { \n";
		file << "\t\t\t\t\t point [ \n";
		for (int i = 0; i < points.size(); i++) {
			MyPoint3D p(points[i]);
			file << "\t\t\t\t\t" << p.data[0] << "\t" << p.data[1] << "\t"
					<< p.data[2] << ",\n";
		}
		file << "\t\t\t\t\t ] \n";
		file << "\t\t\t\t } \n";
		file << "\t\t\t\t coordIndex [ \n";
		for (int i = 0; i < indexes.size(); i++) {
			TriFaceIndex fi(indexes[i]);
			file << "\t\t\t\t\t";
			for (int j = 0; j < 3; j++) {
				file << fi.data[j] << "\t";
			}
			file << "-1,\n";
		}
		file << "\t\t\t\t ] \n";
		file
				<< "\t\t\t\t ccw TRUE \n	\t\t\t\t solid FALSE \n \t\t\t\t convex TRUE \n";
		file << "\t\t\t } \n";
		file << "\t\t } \n \t ] \n }";

		file.close();
	}
}

}//end of namespace
