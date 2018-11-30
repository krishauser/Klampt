/*
 * orXmlTags.cpp
 *
 *  Created on: Sep 4, 2012
 *      Author: jingru
 */

#include "PrimitiveShape.h"
#include "orXmlTags.h"
#include <sstream>
#include <fstream>
#include <cctype>
#include <math.h>
using namespace std;
using namespace Math3D;

extern string ROBOT_DIR;
extern string MODEL_DIR;
extern string KINBODY_DIR;

Matrix3 TranslateInertia2Origin(const Matrix3& Hc,const Vector3& c,Real mass){
  Matrix3 Ho=Hc;
  Ho(0,0) += mass*(c.y*c.y+c.z*c.z);
  Ho(1,1) += mass*(c.x*c.x+c.z*c.z);
  Ho(2,2) += mass*(c.y*c.y+c.x*c.x);
  Ho(0,1) -= mass*c.x*c.y;
  Ho(0,2) -= mass*c.x*c.z;
  Ho(1,2) -= mass*c.y*c.z;
  Ho(1,0) = Ho(0,1);
  Ho(2,0) = Ho(0,2);
  Ho(2,1) = Ho(1,2);
  return Ho;
}

string FileNameWithoutExt(const string& str){
	int pos = str.find_last_of('.');
	string fname = str.substr(0,pos);
	return fname;
}

void axisAngle2RotationMat(const Vector4& aa, Matrix3& mat){
	double ux,uy,uz,w;
	ux = aa.x;
	uy = aa.y;
	uz = aa.z;
	w = 3.1415926*aa.w/180.0;

	double norm = sqrt(ux*ux + uy*uy + uz*uz);
	ux = ux/norm;
	uy = uy/norm;
	uz = uz/norm;

	mat(0,0) = cos(w) + ux*ux*(1-cos(w));
	mat(0,1) = ux*uy*(1-cos(w)) - uz*sin(w);
	mat(0,2) = ux*uz*(1-cos(w)) + uy*sin(w);
	mat(1,0) = uy*ux*(1-cos(w)) + uz*sin(w);
	mat(1,1) = cos(w) + uy*uy*(1-cos(w));
	mat(1,2) = uy*uz*(1-cos(w)) - ux*sin(w);
	mat(2,0) = uz*ux*(1-cos(w)) - uy*sin(w);
	mat(2,1) = uz*uy*(1-cos(w)) + ux*sin(w);
	mat(2,2) = cos(w) + uz*uz*(1-cos(w));

//	mat.data[0][0] = cos(w) + ux*ux*(1-cos(w));
//	mat.data[0][1] = ux*uy*(1-cos(w)) - uz*sin(w);
//	mat.data[0][2] = ux*uz*(1-cos(w)) + uy*sin(w);
//	mat.data[1][0] = uy*ux*(1-cos(w)) + uz*sin(w);
//	mat.data[1][1] = cos(w) + uy*uy*(1-cos(w));
//	mat.data[1][2] = uy*uz*(1-cos(w)) - ux*sin(w);
//	mat.data[2][0] = uz*ux*(1-cos(w)) - uy*sin(w);
//	mat.data[2][1] = uz*uy*(1-cos(w)) + ux*sin(w);
//	mat.data[2][2] = cos(w) + uz*uz*(1-cos(w));
}

void quat2RotationMat(const Vector4& aa, Matrix3& mat){
	double b,c,d,a;
	a = aa.x;
	b = aa.y;
	c = aa.z;
	d = aa.w;

	double norm = sqrt(a*a + b*b + c*c + d*d);
	a = a/norm;
	b = b/norm;
	c = c/norm;
	d = d/norm;

	mat(0,0) = a*a + b*b - c*c - d*d;
	mat(0,1) = 2*b*c - 2*a*d;
	mat(0,2) = 2*b*d + 2*a*c;
	mat(1,0) = 2*b*c + 2*a*d;
	mat(1,1) = a*a - b*b + c*c - d*d;
	mat(1,2) = 2*c*d - 2*a*b;
	mat(2,0) = 2*b*d - 2*a*c;
	mat(2,1) = 2*c*d + 2*a*b;
	mat(2,2) = a*a - b*b - c*c + d*d;

//	mat.data[0][0] = a*a + b*b - c*c - d*d;
//	mat.data[0][1] = 2*b*c - 2*a*d;
//	mat.data[0][2] = 2*b*d + 2*a*c;
//	mat.data[1][0] = 2*b*c + 2*a*d;
//	mat.data[1][1] = a*a - b*b + c*c - d*d;
//	mat.data[1][2] = 2*c*d - 2*a*b;
//	mat.data[2][0] = 2*b*d - 2*a*c;
//	mat.data[2][1] = 2*c*d + 2*a*b;
//	mat.data[2][2] = a*a - b*b - c*c + d*d;
}

void rotationMat2RPY(const Matrix3& mat, Vector3& vec){
	double alphayaw, betapitch, gamaroll;
//	if((mat(0,0)>=0 && mat(0,0) <= 1e-6) || (mat(2,2)>=0 && mat(2,2) <= 1e-6) ){
//		cout<<"mat(0,0)="<<mat(0,0)<<" or mat(2,2)="<<mat(2,2)<<" are zeros!\n"<<flush;
//		getchar();
//	}
	alphayaw = atan2(mat(1,0), mat(0,0));
	betapitch = atan2(-mat(2,0), sqrt(mat(2,1)*mat(2,1) + mat(2,2)*mat(2,2)));
	gamaroll = atan2(mat(2,1), mat(2,2));
	vec.set(gamaroll, betapitch, alphayaw);
//	cout<<"rotationMat2RPY:\n";cout<<"mat:"<<mat<<"\nvec:"<<vec<<endl<<flush;getchar();
}

//TiXmlElement* getFirstChild(TiXmlElement* e, const char* value){
//	if(!value)
//		cout<<"Error: value is empty in getChild!\n";
//	TiXmlElement* c = e->FirstChildElement(value);
//	if(!c){
//		char firstc = value[0];
//		if(islower(firstc))
//			firstc=toupper(firstc);
//		else
//			firstc = tolower(firstc);
//		char newvalue[strlen(value)+1];
//		strcpy(newvalue,value);
//		newvalue[0] = firstc;
//		c = e->FirstChildElement(newvalue);
//		if(value[0] == 'T')cout<<"***"<<value<<":"<<newvalue<<":"<<strcmp(value,"Translation")<<endl;
//		if(!c && 0 == strcmp(value,"Translation")){
//			cout<<"here "<<e->Value()<<endl;
//			c = e->FirstChildElement("translation");
//			if(!c)
//				cout<<" problem!"<<endl;
//		}
//		if(!c && (0==strcmp(value, "KinBody") || 0 == strcmp(value, "kinBody"))){
//			c = e->FirstChildElement("Kinbody");
//			if(!c)
//				c = e->FirstChildElement("kinbody");
//		}
//		if(!c && (0==strcmp(value, "RotationMat") || 0 == strcmp(value, "rotationMat"))){
//			c = e->FirstChildElement("Rotationmat");
//			if(!c)
//				c = e->FirstChildElement("rotationmat");
//		}
//		if(!c && (0==strcmp(value, "RotationAxis") || 0 == strcmp(value, "rotationAxis"))){
//			c = e->FirstChildElement("Rotationaxis");
//			if(!c)
//				c = e->FirstChildElement("rotationaxis");
//		}
//	}
//	return c;
//}
//
//TiXmlElement* getNextSibling(TiXmlElement* e, char* value){
//	if(!value)
//		cout<<"Error: value is empty in getChild!\n";
//	TiXmlElement* c = e->NextSiblingElement(value);
//	if(!c){
//		char firstc = value[0];
//		if(islower(firstc))
//			firstc=toupper(firstc);
//		else
//			firstc = tolower(firstc);
//		char newvalue[strlen(value)+1];
//		strcpy(newvalue,value);
//		newvalue[0] = firstc;
//		c = e->NextSiblingElement(newvalue);
//		if(!c && (0==strcmp(value, "KinBody") || 0 == strcmp(value, "kinBody"))){
//			c = e->NextSiblingElement("Kinbody");
//			if(!c)
//				c = e->NextSiblingElement("kinbody");
//		}
//		if(!c && (0==strcmp(value, "RotationMat") || 0 == strcmp(value, "rotationMat"))){
//			c = e->NextSiblingElement("Rotationmat");
//			if(!c)
//				c = e->NextSiblingElement("rotationmat");
//		}
//		if(!c && (0==strcmp(value, "RotationAxis") || 0 == strcmp(value, "rotationAxis"))){
//			c = e->NextSiblingElement("Rotationaxis");
//			if(!c)
//				c = e->NextSiblingElement("rotationaxis");
//		}
//	}
//	return c;
//}


int lcstrcmp(const char* a,const char* b)
{
	while(*a && *b) {
		if(tolower(*a)!=tolower(*b)) 
			return tolower(*a)-tolower(*b);
		a++;
		b++;
	}
	return tolower(*a)-tolower(*b);
}

TiXmlElement* getFirstChild(TiXmlElement* e, const char* value){
	if(!value)
		cout<<"Error: value is empty in getChild!\n";

	TiXmlElement* child = e->FirstChildElement();
	while(child){
		const char* childstr = child->Value();
		if(0==lcstrcmp(value,childstr)) return child;
		child = child->NextSiblingElement();
	}
	return NULL;
}

TiXmlElement* getNextSibling(TiXmlElement* e, const char* value){
	if(!value)
		cout<<"Error: value is empty in getChild!\n";

	TiXmlElement* child = e->NextSiblingElement();
	while(child){
		if(0 == lcstrcmp(child->Value(), value)){
			return child;
		}
		child = child->NextSiblingElement();
	}
	return NULL;
}

bool hasTransformation(TiXmlElement* e){
	TiXmlElement* c = getFirstChild(e,"Translation");
	if(c){
		return true;
	}
	c = getFirstChild(e,"rotationmat");
	if(c){
		return true;
	}else{
		c = getFirstChild(e,"rotationMat");
		if(c)
			return true;
	}
	c = getFirstChild(e,"rotationaxis");
	if(c){
		return true;
	}else{
		c = getFirstChild(e,"rotationAxis");
		if(c)
			return true;
	}
	c = getFirstChild(e,"quat");
	if(c){
		return true;
	}
	return false;
}

OrXmlTransformation::OrXmlTransformation(TiXmlElement* element){
	e = element;
	T = 0;
}

OrXmlTransformation::~OrXmlTransformation(){
	for(size_t i = 0; i < this->rotationaxis.size(); i++){
		if(this->rotationaxis[i])
				delete rotationaxis[i];
	}
	for(size_t i = 0; i < this->rotationmat.size(); i++){
		if(this->rotationmat[i])
				delete rotationmat[i];
	}
	for(size_t i = 0; i < this->translation.size(); i++){
		if(this->translation[i])
				delete translation[i];
	}
	for(size_t i = 0; i < this->quat.size(); i++){
		if(this->quat[i])
				delete quat[i];
	}
	if(T){
		delete T;
	}
}

//0: translation; 1: rotationmat; 2: rotationaxis; 3: quat
int OrXmlTransformation::getTransformType(TiXmlElement* et){
	if(!et){
		cout<<"getTransformType: et is null!\n"<<flush;
		getchar();
		return -1;
	}
	if(0==lcstrcmp(et->Value(), "rotationmat"))
		return 1;
	if(0==lcstrcmp(et->Value(), "rotationaxis"))
		return 2;
	if(0==lcstrcmp(et->Value(), "quat"))
		return 3;
	return 0;
}

bool OrXmlTransformation::GetContent(){
	TiXmlElement* child = e->FirstChildElement();
	if(!child)
		return false;
	int rmatindex = -1;
	int raxisindex = -1;
	int rquatindex = -1;
	while(child){
		int rtype = getTransformType(child);
		switch(rtype){
		case 1:
			rotationorder.push_back(rtype);
			rmatindex++;
			rotationindex.push_back(rmatindex);
			break;
		case 2:
			rotationorder.push_back(rtype);
			raxisindex++;
			rotationindex.push_back(raxisindex);
			break;
		case 3:
			rotationorder.push_back(rtype);
			rquatindex++;
			rotationindex.push_back(rquatindex);
			break;
		default:
			break;
		}
		child = child->NextSiblingElement();
	}

	TiXmlElement* c = getFirstChild(e,"quat");
	while(c){
		Vector4* q = new Vector4();
		stringstream ss;
		ss << c->GetText();
		ss >> *q;
		quat.push_back(q);
		c = getNextSibling(c,"quat");
	}
	c = getFirstChild(e,"Translation");
	while(c){
		Vector3* t = new Vector3();
		stringstream ss;
		ss << c->GetText();
		ss >> *t;
		translation.push_back(t);
		c = getNextSibling(c,"Translation");
	}
	c = getFirstChild(e,"rotationMat");
	while(c){
		Matrix3* r = new Matrix3();
		stringstream ss;
		ss << c->GetText();
		ss >> *r;
		this->rotationmat.push_back(r);
		c = getNextSibling(c,"rotationMat");
	}
	c = getFirstChild(e,"rotationAxis");
	while(c){
		Vector4* r = new Vector4();
		stringstream ss;
		ss << c->GetText();
		ss >> *r;
		this->rotationaxis.push_back(r);
		c = getNextSibling(c,"rotationAxis");
	}
	return true;
}

bool OrXmlTransformation::ComputeTransform(){
	if(T)
		return true;
	Vector3 t;
	t.setZero();
	for(int i = 0; i < this->translation.size(); i++){
		t.set(t + *translation[i]);
	}
	Matrix3 R;
	R.setIdentity();
	assert(rotationorder.size() == rotationindex.size());
	int nRotation = rotationorder.size();
	for(int i = 0; i < nRotation; i++){
		int index = rotationindex[i];
		if(rotationorder[i] == 1){
			R.set((*rotationmat[ index ]) * R);
		}else if(rotationorder[i] == 2){
			Matrix3 tmp;
			tmp.setIdentity();
			axisAngle2RotationMat(*rotationaxis[ index ], tmp);
			R.set(tmp * R);
		}else if(rotationorder[i] == 3){
			Matrix3 tmp;
			tmp.setIdentity();
			quat2RotationMat(*quat[ index ], tmp);
			R.set(tmp * R);
		}
	}

	this->T = new RigidTransform(R,t);
	return false;
}

OrXmlAdjacent::OrXmlAdjacent(TiXmlElement* element){
	e = element;
}

OrXmlAdjacent::~OrXmlAdjacent(){}

bool OrXmlAdjacent::GetContent(){
	stringstream ss;
	ss << e->GetText();
	ss >> this->body1;
	ss >> this->body2;
	return true;
}

OrXmlJoint::OrXmlJoint(TiXmlElement* element){
	e = element;
	this->anchor = 0;
	this->axis = 0;
	this->resolution = 0;
	this->enabled = true;
}

OrXmlJoint::~OrXmlJoint(){
	if(anchor)
		delete anchor;
	if(axis)
		delete axis;
	if(resolution)
		delete resolution;
}

bool OrXmlJoint::GetContent(){
	e->QueryValueAttribute("type",&type);
	e->QueryValueAttribute("name",&name);

	string enablestr;
	if(e->QueryValueAttribute("enable",&enablestr) == TIXML_SUCCESS){
		if(0==strcmp(enablestr.c_str(),"false"))
			enabled = false;
	}

	TiXmlElement* c = getFirstChild(e,"Body");
	while(c){
		string tmp;
		stringstream ss;
		ss << c->GetText();
		ss >> tmp;
		bodys.push_back(tmp);
		c = getNextSibling(c,"Body");
	}
	c = getFirstChild(e,"offsetfrom");
	if(c){
		stringstream ss;
		ss << c->GetText();
		ss >> offsetfrom;
	}
	c = getFirstChild(e,"anchor");
	if(c){
		anchor = new Vector3();
		stringstream ss;
		ss << c->GetText();
		ss >> *anchor;
	}
	c = getFirstChild(e,"axis");
	if(c){
		axis = new Vector3();
		stringstream ss;
		ss << c->GetText();
		ss >> *axis;
	}
	c = getFirstChild(e,"limits");
	if(c){
		stringstream ss;
		ss << c->GetText();
		while(!ss.eof()){
			double tmp;
			ss >> tmp;
			limits.push_back(tmp);
		}
	}
	c = getFirstChild(e,"limitsdeg");
	if(c){
		stringstream ss;
		ss << c->GetText();
		while(!ss.eof()){
			double tmp;
			ss >> tmp;
			limitsdeg.push_back(tmp);
		}
	}
	c = getFirstChild(e,"limitsrad");
	if(c){
		stringstream ss;
		ss << c->GetText();
		while(!ss.eof()){
			double tmp;
			ss >> tmp;
			limitsrad.push_back(tmp);
		}
	}
	c = getFirstChild(e,"maxvel");
	if(c){
		stringstream ss;
		ss << c->GetText();
		while(!ss.eof()){
			double tmp;
			ss >> tmp;
			maxvel.push_back(tmp);
		}
	}
	c = getFirstChild(e,"maxveldeg");
	if(c){
		stringstream ss;
		ss << c->GetText();
		while(!ss.eof()){
			double tmp;
			ss >> tmp;
			maxveldeg.push_back(tmp);
		}
	}
	c = getFirstChild(e,"maxaccel");
	if(c){
		stringstream ss;
		ss << c->GetText();
		while(!ss.eof()){
			double tmp;
			ss >> tmp;
			maxaccel.push_back(tmp);
		}
	}
	c = getFirstChild(e,"maxacceldeg");
	if(c){
		stringstream ss;
		ss << c->GetText();
		while(!ss.eof()){
			double tmp;
			ss >> tmp;
			maxacceldeg.push_back(tmp);
		}
	}
	c = getFirstChild(e,"weight");
	if(c){
		stringstream ss;
		ss << c->GetText();
		while(!ss.eof()){
			double tmp;
			ss >> tmp;
			weight.push_back(tmp);
		}
	}
	c = getFirstChild(e,"initial");
	if(c){
		stringstream ss;
		ss << c->GetText();
		while(!ss.eof()){
			double tmp;
			ss >> tmp;
			initial.push_back(tmp);
		}
	}
	c = getFirstChild(e,"maxtorque");
	if(c){
		stringstream ss;
		ss << c->GetText();
		while(!ss.eof()){
			double tmp;
			ss >> tmp;
			maxtorque.push_back(tmp);
		}
	}
	return true;
}

bool OrXmlJoint::GetCleanJointInfo(){
	int nLimits = 0;
	if(this->limits.size() > 0 )
		nLimits++;
	if(this->limitsdeg.size() > 0)
		nLimits++;
	if(this->limitsrad.size() > 0)
		nLimits++;
	if(nLimits >= 2){
		cout<<"More than one type of limits are specified!\n"<<flush;
		getchar();
		return false;
	}
	if(limits.size() > 0){
		for(int i = 0; i < limits.size(); i++)
			cleanLimits.push_back(limits[i]);
	}else if(limitsrad.size() > 0){
		for(int i = 0; i < limitsrad.size(); i++)
			cleanLimits.push_back(limitsrad[i]);
	}else if(limitsdeg.size() > 0){
		for(int i = 0; i < limitsdeg.size(); i++)
			cleanLimits.push_back(limitsdeg[i]*3.1415926/180.0);
	}

	nLimits = 0;
	if(this->maxaccel.size() > 0)
		nLimits++;
	if(this->maxacceldeg.size() > 0)
		nLimits++;
	if(nLimits > 1){
		cout<<"More than one type of acc are specified!\n"<<flush;
		getchar();
		return false;
	}
	if(maxaccel.size() > 0){
		for(int i = 0; i < maxaccel.size(); i++)
			cleanMaxAcc.push_back(maxaccel[i]);
	}else if(maxacceldeg.size() > 0){
		for(int i = 0; i < maxacceldeg.size(); i++)
			cleanMaxAcc.push_back(maxacceldeg[i]*3.1415926/180.0);
	}

	nLimits = 0;
	if(this->maxvel.size() > 0)
		nLimits++;
	if(this->maxveldeg.size() > 0)
		nLimits++;
	if(nLimits > 1){
		cout<<"More than one type of vel are specified!\n"<<flush;
		getchar();
		return false;
	}
	if(maxvel.size() > 0){
		for(int i = 0; i < maxvel.size(); i++)
			cleanMaxVel.push_back(maxvel[i]);
	}else if(maxveldeg.size() > 0){
		for(int i = 0; i < maxveldeg.size(); i++)
			cleanMaxVel.push_back(maxveldeg[i]*3.1415926/180.0);
	}

	if(maxtorque.size() > 0){
		for(int i = 0; i < maxtorque.size(); i++)
			cleanMaxTorque.push_back(maxtorque[i]);
	}
	return true;
}

OrXmlMass::OrXmlMass(TiXmlElement* element){
	this->e = element;
	this->density = 0;
	this->radius = 0;
	this->total = 0;
	this->inertia = 0;
	this->com = 0;
	this->extents = 0;
	this->height = 0;
}

OrXmlMass::OrXmlMass(){
	this->density = 0;
	this->radius = 0;
	this->total = 0;
	this->inertia = 0;
	this->com = 0;
	this->extents = 0;
	this->height = 0;
}

OrXmlMass::OrXmlMass(OrXmlMass& newm){
	this->density = 0;
	this->radius = 0;
	this->total = 0;
	this->inertia = 0;
	this->com = 0;
	this->extents = 0;
	this->height = 0;

	if(newm.density){
		density = new double;
		*density = *newm.density;
	}
	if(newm.radius){
		radius = new double;
		*radius = *newm.radius;
	}
	if(newm.total){
		total = new double;
		*total = *newm.total;
	}
	if(newm.com){
		com = new Vector3;
		com->set(*newm.com);
	}
	if(newm.extents){
		extents = new Vector3;
		extents->set(*newm.extents);
	}
	if(newm.height){
		height = new double;
		*height = *newm.height;
	}
	if(newm.inertia){
		inertia = new Matrix3;
		inertia->set(*newm.inertia);
	}
	type = newm.type;
}

OrXmlMass::~OrXmlMass(){
	if(density)
		delete density;
	if(radius)
		delete radius;
	if(total)
		delete total;
	if(com)
		delete com;
	if(inertia)
		delete inertia;
	if(extents)
		delete extents;
	if(height)
		delete height;
}

void OrXmlMass::GetBoxMass(){
	double tmass = defaultmass;
	if(!extents){
		cout<<"Box Mass has no extents!\n"<<flush;getchar();
	}
	if(total){
		tmass = *total;
	}else if(!density){
		*density = defaultDensity;
		tmass = 8.0*extents->x*extents->y*extents->z*(*density);
	}else if(density)
		tmass = 8.0*extents->x*extents->y*extents->z*(*density);

	if(!total)
		total = new double();
	*total = tmass;

    if(!inertia)
    	inertia = new Matrix3();
    inertia->setIdentity();
    (*inertia)(0,0) = tmass/12.0 * ( extents->y * extents->y + extents->z * extents->z );
    (*inertia)(1,1) = tmass/12.0 * ( extents->x * extents->x + extents->z * extents->z );
    (*inertia)(2,2) = tmass/12.0 * ( extents->x * extents->x + extents->y * extents->y );

    if(!com)
    	com = new Vector3;
    com->setZero();
}
void OrXmlMass::GetSphereMass(){
	double tmass = defaultmass;
	if(!radius){
		cout<<"Sphere Mass has no radius!\n"<<flush;getchar();
	}
	if(total){
		tmass = *total;
	}else if(density){
		tmass = 4.0/3.0*PI*(*radius)*(*radius)*(*radius)*(*density);
	}else if(!density){
		*density = defaultDensity;
		tmass = 4.0/3.0*PI*(*radius)*(*radius)*(*radius)*(*density);
	}
	if(!total)
		total = new double();
	*total = tmass;
	if(!inertia)
		inertia = new Matrix3();
	inertia->setIdentity();
	(*inertia)(0,0) = 0.4 * tmass * (*radius)*(*radius);
	(*inertia)(1,1) = (*inertia)(0,0);
	(*inertia)(2,2) = (*inertia)(0,0);

	if(!com)
		com = new Vector3;
	com->setZero();
//	cout<<tmass<<" sphere inertia:"<<(*inertia)<<endl;
}

void OrXmlMass::GetUnitSphereMass(){
	double tmass = defaultmass;
	if(!total)
		total = new double;
	*total = tmass;

	if(!inertia)
		inertia = new Matrix3();
	inertia->setIdentity();
	(*inertia)(0,0) = 0.4 * tmass;
	(*inertia)(1,1) = (*inertia)(0,0);
	(*inertia)(2,2) = (*inertia)(0,0);

	if(!com)
		com = new Vector3;
	com->setZero();
}

void OrXmlMass::GetCylinderMass(){
	double tmass = defaultmass;
	if(!radius || !height){
		cout<<"Cylinder Mass has no radius or height!\n"<<flush;getchar();
	}
	if(total){
		tmass = *total;
	}else if(!density){
		*density = defaultDensity;
		tmass = PI*(*radius)*(*radius)*(*height)*(*density);
	}else if(density)
		tmass = PI*(*radius)*(*radius)*(*height)*(*density);

	if(!total)
		total = new double();
	*total = tmass;
	if(!inertia)
		inertia = new Matrix3();
	inertia->setIdentity();
	(*inertia)(0,0) = tmass * ( 0.25 * (*radius)*(*radius) + (1.0/12.0) * (*height)*(*height) );
	(*inertia)(1,1) = (*inertia)(0,0);
	(*inertia)(2,2) = tmass * 0.5 * (*radius)*(*radius);

	if(!com)
		com = new Vector3;
	com->setZero();
}

void OrXmlMass::AddMass(OrXmlMass* r){
//    MASS mnew;
//    if( fTotalMass+r.fTotalMass == 0 ) {
//        return mnew;
//    }
//    mnew.fTotalMass = fTotalMass + r.fTotalMass;
//    mnew.t.trans = (fTotalMass*t.trans+r.fTotalMass*r.t.trans)*((dReal)1.0/mnew.fTotalMass);
//    MASS m0=MASS(*this);
//    m0.ChangeCenterOfRotation(mnew.t.trans);
//    MASS m1=MASS(r);
//    m1.ChangeCenterOfRotation(mnew.t.trans);
//    for(int i = 0; i < 3; ++i) {
//        for(int j = 0; j < 3; ++j) {
//            mnew.t.m[4*i+j] = m0.t.m[4*i+j] + m1.t.m[4*i+j];
//        }
//    }
//
//    MASS mnew = operator+(r);
//	t = mnew.t;
//	fTotalMass = mnew.fTotalMass;
	if(!r->inertia || !r->total || !inertia || !total){
		cout<<"total or inertia is empty!\n"<<flush;getchar();
		return;
	}
	if(*total + *(r->total) == 0)
		return;

	OrXmlMass* mnew = new OrXmlMass();
	mnew->total = new double();
	mnew->com = new Vector3;
	mnew->inertia = new Matrix3;

	*mnew->total = *total + *(r->total);

	double tmp = *total/(*mnew->total);
	double tmp2 = *(r->total)/(*mnew->total);

	mnew->com->add(tmp*(*com), tmp2*(*(r->com)));
	OrXmlMass* m0 = new OrXmlMass(*this);
	m0->ChangeCenterOfRotation(*mnew->com);
	OrXmlMass* m1 = new OrXmlMass(*r);
	m1->ChangeCenterOfRotation(*mnew->com);

	mnew->inertia->add(*m0->inertia, *m1->inertia);

	*this->total = *mnew->total;
	this->com->set(*mnew->com);
	this->inertia->set(*mnew->inertia);

	delete m0;
	delete m1;
	delete mnew;
}

/// \brief changes the center of rotation (ie center of mass)
void OrXmlMass::ChangeCenterOfRotation(const Vector3& newcor)
{
//    Vector v = newcor-t.trans;
//    // translate the inertia tensor
//    dReal x2 = v.x*v.x, y2 = v.y*v.y, z2 = v.z*v.z;
//    t.m[0] += fTotalMass * (y2+z2);     t.m[1] -= fTotalMass * v.x * v.y;   t.m[2] -= fTotalMass * v.x * v.z;
//    t.m[4] -= fTotalMass * v.y * v.x;   t.m[5] += fTotalMass * (x2+z2);     t.m[6] -= fTotalMass * v.y * v.z;
//    t.m[8] -= fTotalMass * v.z * v.x;   t.m[9] -= fTotalMass * v.z * v.y;   t.m[10] += fTotalMass * (x2+y2);
//    return *this;
	if(!total || !com || !inertia){
		cout<<"total or com or inertia is empty!\n"<<flush;getchar();
		return;
	}
	Vector3 v(newcor-(*com));
	double x2 = v.x*v.x;
	double y2 = v.y*v.y;
	double z2 = v.z*v.z;

	(*inertia)(0,0) += (*total)*(y2+z2); (*inertia)(0,1) -= (*total) * v.x * v.y;   (*inertia)(0,2) -= (*total) * v.x * v.z;
	(*inertia)(1,0) -= (*total) * v.y * v.x;   (*inertia)(1,1) += (*total) * (x2+z2);     (*inertia)(1,2) -= (*total) * v.y * v.z;
	(*inertia)(2,0) -= (*total) * v.z * v.x;   (*inertia)(2,1) -= (*total) * v.z * v.y;   (*inertia)(2,2) += (*total) * (x2+y2);
}

/// \brief transform the center of mass and inertia matrix by trans
void OrXmlMass::ChangeCoordinateSystem(RigidTransform& trans)
{
//    Vector oldcom = t.trans;
//    TransformMatrix trot = matrixFromQuat(trans.rot);
//    t = trot.rotate(t.rotate(trot.inverse())); // rotate inertia
//    t.trans = trans*oldcom;
	if(!com || !inertia){
		cout<<"No com or inertia specified!"<<flush;
		getchar();
	}
	Vector3 oldcom(*com);
	RigidTransform trot, inverseT, massT, tmpT, finalmassT;
	trot.R.set(trans.R);
	trot.t.setZero();

	massT.R.set(*inertia);
	massT.t.set(*com);

	trot.getInverse(inverseT);

	tmpT.mul(massT, inverseT);
	finalmassT.mul(trot, tmpT);

	trans.mul(oldcom, finalmassT.t);
	com->set(finalmassT.t);
	inertia->set(finalmassT.R);
}

bool OrXmlMass::GetContent(){
	if(e->QueryValueAttribute("type",&type)!=TIXML_SUCCESS){
		cout<<"Error in GetContent! No type specified!\n"<<flush;
		getchar();
		return false;
	}
	TiXmlElement* c = getFirstChild(e,"com");
	if(c){
		com = new Vector3();
		stringstream ss;
		ss << c->GetText();
		ss >> *com;
	}
	c = getFirstChild(e,"inertia");
	if(c){
		inertia = new Matrix3();
		stringstream ss;
		ss << c->GetText();
		ss >> *inertia;
	}
	c = getFirstChild(e,"density");
	if(c){
		density = new double();
		stringstream ss;
		ss << c->GetText();
		ss >> *density;
	}
//	else{
//		density = new double();
//		*density = defaultDensity;
//	}
	c = getFirstChild(e,"total");
	if(c){
		total = new double();
		stringstream ss;
		ss << c->GetText();
		ss >> *total;
	}
//	else{
//		total = new double();
//		*total = defaultmass;
//	}
	c = getFirstChild(e,"radius");
	if(c){
		radius = new double();
		stringstream ss;
		ss << c->GetText();
		ss >> *radius;
	}
	c = getFirstChild(e,"height");
	if(c){
		height = new double();
		stringstream ss;
		ss << c->GetText();
		ss >> *height;
	}
	c = getFirstChild(e,"extents");
	if(c){
		this->extents = new Vector3;
		stringstream ss;
		ss << c->GetText();
		ss >> *extents;
	}

	if(0 == strcmp(type.c_str(), "box")){
		this->GetBoxMass();
	}else if(0 == strcmp(type.c_str(), "sphere")){
		this->GetSphereMass();
	}else if(0 == strcmp(type.c_str(), "cylinder")){
		this->GetCylinderMass();
	}else if(0 == strcmp(type.c_str(), "custom")){
		//by default a custom mass is a unit spherical mass unless specified
		if(!total){
			total = new double();
			*total = defaultmass;
		}
		if(!com){
			com = new Vector3();
			com->setZero();
		}
		if(!inertia){
			inertia = new Matrix3();
			inertia->setIdentity();
			(*inertia)(0,0) = 0.4 * (*total);
			(*inertia)(1,1) = (*inertia)(0,0);
			(*inertia)(2,2) = (*inertia)(0,0);
		}
	}else if(0 == strcmp(type.c_str(), "mimicgeom")){
//		cout << "mimicgeom!\n"<<flush;
	}
	return true;
}

OrXmlGeom::OrXmlGeom(TiXmlElement* element){
	this->e = element;
	this->extents = 0;
	this->height = 0;
	this->transformation = 0;
	this->radius = 0;
	this->renderscale = 0;
	this->datascale = 0;
}

OrXmlGeom::~OrXmlGeom(){
	if(extents)
		delete extents;
	if(height)
		delete height;
	if(transformation)
		delete transformation;
	if(radius)
		delete radius;
	if(renderscale)
		delete renderscale;
	if(datascale)
		delete datascale;
}

bool OrXmlGeom::Convert2Tri(){
	if(this->type.empty()){
		cout<<"Geom has no type!\n"<<flush;
		getchar();
	}
	if(strcmp(type.c_str(), "trimesh") != 0 && !render.empty()){
		cout<<"Not support non trimesh type has render data!\n"<<flush;
		getchar();
	}
	if(!render.empty() && !dorender){
		cout<<"Render=false while render file is not empty!\n"<<flush;
		getchar();
	}
	if(strcmp(type.c_str(), "trimesh") == 0){
		if(!render.empty()){
			this->loadGeom(true);
			applyScale(true);
		}
		if(!data.empty()){
			this->loadGeom(false);
			applyScale(false);
		}
	}else if(strcmp(type.c_str(), "box") == 0){
		if(!extents){
			cout<<"No extent specified for Box!\n"<<flush;
			getchar();
		}
		Box box(extents->x,extents->y,extents->z);
		copyPrimitiveMesh(box.points, box.indexes);
	}else if(strcmp(type.c_str(), "sphere") == 0){
		if(!radius){
			cout<<"No radius specified for sphere!\n"<<flush;
			getchar();
		}
		Sphere sphere(*radius);
		copyPrimitiveMesh(sphere.points, sphere.indexes);
	}else if(strcmp(type.c_str(), "cylinder") == 0){
		if(!radius || !height){
			cout<<"No radius or height specified for cylinder!\n"<<flush;
			getchar();
		}
		Cylinder cylinder(*radius, *height);
		copyPrimitiveMesh(cylinder.points, cylinder.indexes);
	}
	applyTransformation();
	return true;
}

bool OrXmlGeom::applyTransformation(){
	if(!transformation)
		return true;
	transformation->ComputeTransform();

	for(int i = 0; i < vispoints.size(); i++){
		Vector3 v(vispoints[i].data[0],vispoints[i].data[1],vispoints[i].data[2]);
		Vector3 out;
		transformation->T->mul(v, out);
		vispoints[i].Set(out.x, out.y, out.z);
	}
	for(int i = 0; i < colpoints.size(); i++){
		Vector3 v(colpoints[i].data[0],colpoints[i].data[1],colpoints[i].data[2]);
		Vector3 out;
		transformation->T->mul(v, out);
		colpoints[i].Set(out.x, out.y, out.z);
	}
	return true;
}

bool OrXmlGeom::applyScale(bool visible){
	if(!renderscale && visible){
		cout<<"Geom has no renderscale!\n"<<flush;
		getchar();
		return true;
	}
	if(!datascale && !visible){
		cout<<"Geom has no datascale!\n"<<flush;
		getchar();
		return true;
	}
	Matrix3 scaleM;
	scaleM.setIdentity();
	if(visible){
		scaleM.data[0][0] = *renderscale;
		scaleM.data[1][1] = *renderscale;
		scaleM.data[2][2] = *renderscale;
		for(int i = 0; i < vispoints.size(); i++){
			Vector3 v(vispoints[i].data[0],vispoints[i].data[1],vispoints[i].data[2]);
			Vector3 out;
			scaleM.mul(v, out);
			vispoints[i].Set(out.x, out.y, out.z);
		}
	}else{
		scaleM.data[0][0] = *datascale;
		scaleM.data[1][1] = *datascale;
		scaleM.data[2][2] = *datascale;
		for(int i = 0; i < colpoints.size(); i++){
			Vector3 v(colpoints[i].data[0],colpoints[i].data[1],colpoints[i].data[2]);
			Vector3 out;
			scaleM.mul(v, out);
			colpoints[i].Set(out.x, out.y, out.z);
		}
	}
	return true;
}

void OrXmlGeom::copyPrimitiveMesh(const vector<MyPoint3D>& pts, const vector<TriFaceIndex>& tis){
	TriFaceIndex ti;
//	if(dorender){
//		int baseIndex = vispoints.size();
//		for(int i = 0; i < pts.size(); i++){
//			vispoints.push_back(pts[i]);
//		}
//		for(int i = 0; i < tis.size(); i++){
//			ti.Set(tis[i].data[0]+baseIndex, tis[i].data[1]+baseIndex, tis[i].data[2]+baseIndex);
//			visindexes.push_back(ti);
//		}
//	}else{
//		int baseIndex = colpoints.size();
//		for(int i = 0; i < pts.size(); i++){
//			colpoints.push_back(pts[i]);
//		}
//		for(int i = 0; i < tis.size(); i++){
//			ti.Set(tis[i].data[0]+baseIndex, tis[i].data[1]+baseIndex, tis[i].data[2]+baseIndex);
//			colindexes.push_back(ti);
//		}
//	}
	{
		int baseIndex = vispoints.size();
		for(int i = 0; i < pts.size(); i++){
			vispoints.push_back(pts[i]);
		}
		for(int i = 0; i < tis.size(); i++){
			ti.Set(tis[i].data[0]+baseIndex, tis[i].data[1]+baseIndex, tis[i].data[2]+baseIndex);
			visindexes.push_back(ti);
		}
	}
	{
		int baseIndex = colpoints.size();
		for(int i = 0; i < pts.size(); i++){
			colpoints.push_back(pts[i]);
		}
		for(int i = 0; i < tis.size(); i++){
			ti.Set(tis[i].data[0]+baseIndex, tis[i].data[1]+baseIndex, tis[i].data[2]+baseIndex);
			colindexes.push_back(ti);
		}
	}
}

void OrXmlGeom::loadGeom(bool visible){
	if(visible && render.empty()){
		cout<<"Render Geometry is empty!\n"<<flush;
		getchar();
	}
	else if(!visible && data.empty()){
		cout<<"Collision Geometry is empty!\n"<<flush;
		getchar();
	}
	const char* ext;
	if(visible)
		ext = FileExtension(render.c_str());
	else
		ext = FileExtension(data.c_str());
	if(strcmp(ext,"wrl")==0){
		loadWrl(visible);
	}else if(strcmp(ext,"tri")==0){
		loadTri(visible);
	}
}

void OrXmlGeom::loadWrl(bool visible){
	char filename[200];
	if(visible)
		sprintf(filename,"%s%s%s%s",ROBOT_DIR.c_str(),KINBODY_DIR.c_str(),MODEL_DIR.c_str(),render.c_str());
	else
		sprintf(filename,"%s%s%s%s",ROBOT_DIR.c_str(),KINBODY_DIR.c_str(),MODEL_DIR.c_str(),data.c_str());
	ifstream file;
	file.open(filename);
	if(!file.is_open()){
		cout<<"Cannot read in wrl "<<filename<<" geometry!\n"<<flush;
		getchar();
	}
	int visbaseIndex = vispoints.size();
	int colbaseIndex = colpoints.size();
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
					if(visible)
						vispoints.push_back(pt);
					else
						colpoints.push_back(pt);
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
					while(tmp !=NULL && ind < 3){
						p[ind] = atoi(tmp);
						ind++;
						tmp = strtok(NULL," ,");
					}
					delete []strstr;
					getline(file,str);
					if(p[0] == p[1] || p[1] == p[2] || p[0] == p[2])
						continue;
					else{
						if(visible){
							TriFaceIndex ti(p[0]+visbaseIndex,p[1]+visbaseIndex,p[2]+visbaseIndex);
							visindexes.push_back(ti);
							if(ti.data[0] > vispoints.size() || ti.data[1] > vispoints.size() || ti.data[2] > vispoints.size()){
								cout<< "render:"<<render<<endl;
								cout<<vispoints.size()<<";("<<ti.data[0]<<","<<ti.data[1]<<","<<ti.data[2]<<")\n"<<flush;
								getchar();
							}
						}else{
							TriFaceIndex ti(p[0]+colbaseIndex,p[1]+colbaseIndex,p[2]+colbaseIndex);
							colindexes.push_back(ti);
						}
					}
				}
			}
		}
	}
	file.close();
}

void OrXmlGeom::loadTri(bool visible){
	ifstream file;
	char filename[200];
	if(visible)
		sprintf(filename,"%s%s%s%s",ROBOT_DIR.c_str(),KINBODY_DIR.c_str(),MODEL_DIR.c_str(),render.c_str());
	else
		sprintf(filename,"%s%s%s%s",ROBOT_DIR.c_str(),KINBODY_DIR.c_str(),MODEL_DIR.c_str(),data.c_str());
	file.open(filename);
	if(!file.is_open()){
		cout<<"Cannot read in tri "<<filename<<" geometry!\n"<<flush;
		getchar();
	}
	int visbaseIndex = vispoints.size();
	int colbaseIndex = colpoints.size();
	int nPoint;
	file >> nPoint;
	for(int i = 0; i < nPoint; i++){
		double x,y,z;
		file >> x;
		file >> y;
		file >> z;
		MyPoint3D p(x,y,z);
		if(visible)
			vispoints.push_back(p);
		else
			colpoints.push_back(p);
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
		if(visible){
			TriFaceIndex ti(i0+visbaseIndex,i1+visbaseIndex,i2+visbaseIndex);
			visindexes.push_back(ti);
		}else{
			TriFaceIndex ti(i0+colbaseIndex,i1+colbaseIndex,i2+colbaseIndex);
			colindexes.push_back(ti);
		}
	}
	file.close();
}

bool OrXmlGeom::GetContent(){
	if(e->QueryValueAttribute("type",&type)!=TIXML_SUCCESS){
		cout<<"Error in GetContent! No type specified!\n";
		return false;
	}
	dorender = true;
	string renderstr;
	if(e->QueryValueAttribute("render",&renderstr)==TIXML_SUCCESS){
		if(0==strcmp(renderstr.c_str(),"false")){
			dorender = false;
//			return false;
		}
		else if(0==strcmp(renderstr.c_str(),"true"))
			dorender = true;
	}
	TiXmlElement* c = getFirstChild(e,"Render");
	if(c){
		stringstream ss;
		ss << c->GetText();
		ss >> render;
		if(!ss.eof()){
			renderscale = new double;
			ss >> *renderscale;
		}
	}
	c = getFirstChild(e,"Data");
	if(c){
		stringstream ss;
		ss << c->GetText();
		ss >> data;
		if(!ss.eof()){
			datascale = new double;
			ss >> *datascale;
		}
	}
	if(hasTransformation(e)){
		this->transformation = new OrXmlTransformation(e);
		transformation->GetContent();
	}
	if(strcmp(type.c_str(), "box") == 0){
		TiXmlElement* c = getFirstChild(e,"extents");
		if(!c){
			cout<<"Error in GetContent! No extents for box!\n";
			return false;
		}
		this->extents = new Vector3;
		stringstream ss;
		ss << c->GetText();
		ss >> *extents;
	}
	if(strcmp(type.c_str(), "sphere") == 0){
		TiXmlElement* c = getFirstChild(e,"radius");
		if(!c){
			cout<<"Error in GetContent! No radius for sphere!\n";
			return false;
		}
		this->radius = new double;
		stringstream ss;
		ss << c->GetText();
		ss >> *radius;
	}
	if(strcmp(type.c_str(), "cylinder") == 0){
		TiXmlElement* c = getFirstChild(e,"radius");
		if(!c){
			cout<<"Error in GetContent! No radius for cylinder!\n";
			return false;
		}
		this->radius = new double;
		stringstream ss;
		ss << c->GetText();
		ss >> *radius;
		c = getFirstChild(e,"height");
		if(!c){
			cout<<"Error in GetContent! No radius for cylinder!\n";
			return false;
		}
		this->height = new double;
		stringstream ss2;
		ss2 << c->GetText();
		ss2 >> *height;
	}
	return true;
}

std::ostream& operator << (std::ostream& out, const OrXmlAdjacent& adjacent){
	out <<"adjacent body1:" <<adjacent.body1<<endl;
	out <<"adjacent body2:" <<adjacent.body2<<endl;
	return out;
}

std::ostream& operator << (std::ostream& out, const OrXmlJoint& joint){
	out<<"----joint name:"<<joint.name<<endl;
	out<<"----joint type:"<<joint.type<<endl;
	if(joint.anchor)
		out <<"----joint anchor:"<<*joint.anchor<<endl;
	if(joint.axis)
			out <<"----joint axis:"<<*joint.axis<<endl;
	for(int i = 0; i < joint.bodys.size(); i++)
		out <<"----joint body:"<< joint.bodys[i]<<endl;
	return out;
}

std::ostream& operator << (std::ostream& out, const OrXmlMass& mass){
	out<<"----mass type:"<<mass.type<<endl;
	if(mass.com)
		out <<"----mass com:"<<*mass.com<<endl;
	if(mass.density)
		out <<"----mass density:"<<*mass.density<<endl;
	if(mass.inertia)
		out <<"----mass inertia:"<<*mass.inertia<<endl;
	if(mass.radius)
		out <<"----mass radius:"<<*mass.radius<<endl;
	if(mass.total)
		out <<"----mass total:"<<*mass.total<<endl;
	return out;
}
std::ostream& operator << (std::ostream& out, const OrXmlGeom& geom){
	out<<"----geom type:"<<geom.type<<endl;
	if(geom.extents)
		out<<"----geom extents:"<<*geom.extents<<endl;
	if(geom.height)
		out<<"----geom height:"<<*geom.height<<endl;
	if(geom.radius)
		out<<"----geom radius:"<<*geom.radius<<endl;
	if(geom.transformation)
		out<<*geom.transformation<<endl;
	return out;
}
std::ostream& operator << (std::ostream& out, const OrXmlTransformation& tran){
	for(int i = 0; i < tran.translation.size(); i++){
		if(tran.translation[i])
			out<<"-----"<<i<<" translation:"<<*tran.translation[i]<<endl;
	}
	for(int i = 0; i < tran.rotationaxis.size(); i++){
		if(tran.rotationaxis[i])
			out<<"-----"<<i<<" rotationaxis:"<<*tran.rotationaxis[i]<<endl;
	}
	for(int i = 0; i < tran.rotationmat.size(); i++){
		if(tran.rotationmat[i])
			out<<"-----"<<i<<" rotationmat:"<<*tran.rotationmat[i]<<endl;
	}
	for(int i = 0; i < tran.quat.size(); i++){
		if(tran.quat[i])
			out<<"-----"<<i<<" quat:"<<*tran.quat[i]<<endl;
	}
	return out;
}
