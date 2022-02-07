#include "orXmlEnvironment.h"
#include "Modeling/Mass.h"
#include <sstream>
#include <fstream>
using namespace std;
namespace Klampt {

string ToLowercase(string str){
	string lowerstr = "";
	  int i = 0;
	  char c;
	  while (str.c_str()[i])
	  {
	    c=str.c_str()[i];
	    c = tolower(c);
	    lowerstr += c;
	    i++;
	  }
	  return lowerstr;
}

OrXmlKinbody::OrXmlKinbody(TiXmlElement* element) {
	this->doc = 0;
	this->e = element;
	this->mass = 0;
	this->transformation = 0;
	this->isGetAllBodys = false;
	this->hasSetPrefix = false;
	isGetAllCleanBody = false;
	kin_dir_str = "";
}

OrXmlKinbody::~OrXmlKinbody() {
	if (doc)
		delete doc;
	if (mass)
		delete mass;
	if (transformation)
		delete transformation;
	for (size_t i = 0; i < this->xmlBodys.size(); i++) {
		if (xmlBodys[i])
			delete xmlBodys[i];
	}
	for (size_t i = 0; i < this->xmlJoints.size(); i++) {
		if (xmlJoints[i])
			delete xmlJoints[i];
	}
	for (size_t i = 0; i < this->xmlAdjacents.size(); i++) {
		if (xmlAdjacents[i])
			delete xmlAdjacents[i];
	}
	for (size_t i = 0; i < this->xmlKinbodys.size(); i++) {
		if (xmlKinbodys[i])
			delete xmlKinbodys[i];
	}
}

bool OrXmlKinbody::RemoveSameBody() {
	vector<OrXmlBody*> newBodys;
	for (size_t i = 0; i < xmlBodys.size(); i++) {
		bool repeat = false;
		for (size_t j = 0; j < newBodys.size(); j++) {
			if (0
					== strcmp(xmlBodys[i]->name.c_str(),
							newBodys[j]->name.c_str())) {
				combineTwoBody(newBodys[j], xmlBodys[i]);
//				cout<<newBodys[j]->name<<";"<<newBodys[j]->offsetfrom<<endl;
				repeat = true;
				break;
			}
		}
		if (false == repeat)
			newBodys.push_back(xmlBodys[i]);
	}
	xmlBodys.clear();
	for (size_t i = 0; i < newBodys.size(); i++) {
		xmlBodys.push_back(newBodys[i]);
	}
	return true;
}

//copy extra properties of b2 to b1
void OrXmlKinbody::combineTwoBody(OrXmlBody* b1, OrXmlBody* b2) {
	if (0 != strcmp(b1->name.c_str(), b2->name.c_str())) {
		cout << "Error: Bodies' names are not the same!\n" << flush;
		getchar();
	}
	if ((b1->xmlGeoms.size() > 0 && b2->xmlGeoms.size() > 0)\

			|| (b1->mass && b2->mass)
			|| (b1->transformation && b2->transformation)
			|| (!b1->offsetfrom.empty() && !b2->offsetfrom.empty())\

			|| (b1->vispoints.size() > 0 && b2->vispoints.size() > 0)\

			|| (b1->colpoints.size() > 0 && b2->colpoints.size() > 0)) {
		cout << "Error: both bodies have same properties!\n" << flush;
		getchar();
	}
	if (!b2->offsetfrom.empty() && b1->offsetfrom.empty()) {
		b1->offsetfrom = b2->offsetfrom;
	}
	if (!b2->vis_filename.empty() && b1->vis_filename.empty()) {
		b1->vis_filename = b2->vis_filename;
	}
	if (!b2->col_filename.empty() && b1->col_filename.empty()) {
		b1->col_filename = b2->col_filename;
	}
	if (b2->mass && !b1->mass) {
		b1->mass = b2->mass;
		b2->mass = 0;
	}
	if (b2->transformation && !b1->transformation) {
		b1->transformation = b2->transformation;
		b2->transformation = 0;
	}
	if (!b2->type.empty() && b1->type.empty())
		b1->type = b2->type;
	if (b2->xmlGeoms.size() > 0 && b1->xmlGeoms.size() == 0) {
		for (size_t i = 0; i < b2->xmlGeoms.size(); i++) {
			b1->xmlGeoms.push_back(b2->xmlGeoms[i]);
			b2->xmlGeoms[i] = 0;
		}
	}
	if (b2->vispoints.size() > 0 && b1->vispoints.size() == 0) {
		for (size_t i = 0; i < b2->vispoints.size(); i++)
			b1->vispoints.push_back(b2->vispoints[i]);
		for (size_t i = 0; i < b2->visindexes.size(); i++)
			b1->visindexes.push_back(b2->visindexes[i]);
		b2->vispoints.clear();
		b2->visindexes.clear();
	}
	if (b2->colpoints.size() > 0 && b1->colpoints.size() == 0) {
		for (size_t i = 0; i < b2->colpoints.size(); i++)
			b1->colpoints.push_back(b2->colpoints[i]);
		for (size_t i = 0; i < b2->colindexes.size(); i++)
			b1->colindexes.push_back(b2->colindexes[i]);
		b2->colpoints.clear();
		b2->colindexes.clear();
	}
}

bool OrXmlKinbody::appyTran2Tparent(OrXmlTransformation* tran) {
	if (!tran) {
		return true;
	}
	if (!isGetAllBodys) {
		cout << "Should call GetAllBodyJoints first!\n" << flush;
		getchar();
		return false;
	}
	if (!hasSetPrefix) {
		cout << name << " hasSetPrefix == false!\n" << flush;
		getchar();
		return false;
	}
	tran->ComputeTransform();
	for (size_t i = 0; i < this->xmlBodys.size(); i++) {
//		if(0==strcmp(xmlBodys[i]->name.c_str(),"rightIndexProximal")){
//			cout<<"here1! "<<name<<":"<<xmlBodys[i]->name<<endl;
//			if(xmlBodys[i]->transformation){
//				xmlBodys[i]->transformation->ComputeTransform();
//				cout<<*xmlBodys[i]->transformation->T<<endl<<flush;
//			}
//			cout<<"tran:"<<*tran->T<<endl;
//			getchar();
//		}
		if (xmlBodys[i]->offsetfrom.empty()){
			xmlBodys[i]->applyTran2Tparent(tran);
		}else {
			bool offsetfromhere = false;
			for (size_t j = 0; j < this->xmlBodys.size(); j++) {
				if (i != j
						&& 0
								== strcmp(xmlBodys[i]->offsetfrom.c_str(),
										xmlBodys[j]->name.c_str())) {
					offsetfromhere = true;
					break;
				}
			}
			if (offsetfromhere == false) {
				xmlBodys[i]->applyTran2Tparent(tran);
//				cout << "apply Kinbody tran:" << *tran->T << endl
//						<< xmlBodys[i]->name << endl << xmlBodys[i]->offsetfrom
//						<< flush;
//				getchar();
			}
		}
//		if(0==strcmp(xmlBodys[i]->name.c_str(),"rightIndexProximal")){
//			cout<<"here2! "<<name<<":"<<xmlBodys[i]->name<<endl;
//			if(xmlBodys[i]->transformation){
////				xmlBodys[i]->transformation->ComputeTransform();
//				cout<<*xmlBodys[i]->transformation->T<<flush;
//			}
//			getchar();
//		}
	}
	return true;
}

void OrXmlKinbody::GetAllBodyGeom() {
	if (!isGetAllBodys) {
		cout << "Should call GetAllBodyJoints first!\n" << flush;
		getchar();
	}
	for (size_t i = 0; i < this->xmlBodys.size(); i++) {
		xmlBodys[i]->GetMimicGeomMass();
		xmlBodys[i]->GetFinalGeom();
		if (xmlBodys[i]->transformation)
			xmlBodys[i]->transformation->ComputeTransform();
		cout<<"kinbody:GetAllBodyGeom:"<<xmlBodys[i]->name<<endl;
	}
}

void OrXmlKinbody::setPrefixName() {
	if (this->hasSetPrefix || prefix.empty()) {
		hasSetPrefix = true;
		return;
	}
	for (size_t i = 0; i < this->xmlKinbodys.size(); i++) {
		string tmp = prefix;
		tmp.append(xmlKinbodys[i]->prefix.c_str());
		xmlKinbodys[i]->prefix = tmp;
	}
	for (size_t i = 0; i < this->xmlBodys.size(); i++) {
		string tmp = prefix;
		tmp.append(xmlBodys[i]->name.c_str());
		xmlBodys[i]->name = tmp;
		if (!xmlBodys[i]->offsetfrom.empty()) {
			tmp = prefix;
			tmp.append(xmlBodys[i]->offsetfrom.c_str());
			xmlBodys[i]->offsetfrom = tmp;
		}
	}
	for (size_t i = 0; i < this->xmlJoints.size(); i++) {
		for (size_t j = 0; j < this->xmlJoints[i]->bodys.size(); j++) {
			string tmp = prefix;
			tmp.append(xmlJoints[i]->bodys[j].c_str());
			xmlJoints[i]->bodys[j] = tmp;
		}
		if (!xmlJoints[i]->offsetfrom.empty()) {
			string tmp = prefix;
			tmp.append(xmlJoints[i]->offsetfrom.c_str());
			xmlJoints[i]->offsetfrom = tmp;
		}
	}
	this->hasSetPrefix = true;
}

bool OrXmlKinbody::GetAllCleanBodyJoints() {
	this->setPrefixName();

	for (size_t i = 0; i < this->xmlKinbodys.size(); i++) {
		xmlKinbodys[i]->GetAllCleanBodyJoints();
		for (size_t j = 0; j < xmlKinbodys[i]->xmlBodys.size(); j++) {
			OrXmlBody* body = xmlKinbodys[i]->xmlBodys[j];
			xmlBodys.push_back(body);
			xmlKinbodys[i]->xmlBodys[j] = 0;
		}
		for (size_t j = 0; j < xmlKinbodys[i]->xmlJoints.size(); j++) {
			OrXmlJoint* joint = xmlKinbodys[i]->xmlJoints[j];
			xmlJoints.push_back(joint);
			xmlKinbodys[i]->xmlJoints[j] = 0;
		}
		for (size_t j = 0; j < xmlKinbodys[i]->xmlAdjacents.size(); j++) {
			OrXmlAdjacent* adjacent = xmlKinbodys[i]->xmlAdjacents[j];
			xmlAdjacents.push_back(adjacent);
			xmlKinbodys[i]->xmlAdjacents[j] = 0;
		}
		for (size_t j = 0; j < xmlKinbodys[i]->jointvalues.size(); j++) {
			double jv = xmlKinbodys[i]->jointvalues[j];
			jointvalues.push_back(jv);
			xmlKinbodys[i]->jointvalues[j] = 0;
		}
	}

	this->isGetAllBodys = true;

	this->GetAllBodyGeom();

	this->RemoveSameBody();

	this->appyTran2Tparent(this->transformation);
	isGetAllCleanBody = true;
	return true;
}

bool OrXmlKinbody::GetContent() {
	if (e->QueryValueAttribute("file", &file) == TIXML_SUCCESS) {
		doc = new TiXmlDocument();
		char path[100];
		GetFilePath(file.c_str(), path);
		const char* name = GetFileName(file.c_str());
		string tmp(path);
		char filename[200];

//		if (tmp.find('/') >= 0 && tmp.find('/') < tmp.length()) {
//			KINBODY_DIR.assign(path);
//			sprintf(filename, "%s%s", ROBOT_DIR.c_str(), file.c_str());
//		} else
//			sprintf(filename, "%s%s%s", ROBOT_DIR.c_str(), KINBODY_DIR.c_str(),
//					file.c_str());

		if (tmp.find('/') >= 0 && tmp.find('/') < tmp.length()) {
			kin_dir_str.append(path);
		}
		sprintf(filename, "%s%s%s", ROBOT_DIR.c_str(), kin_dir_str.c_str(), name);


//		cout<<"ROBOT_DIR:"<<ROBOT_DIR<<endl;
//		cout<<"KINBODY_DIR:"<<KINBODY_DIR<<endl;
//		cout<<"file:"<<file<<endl;
//		cout<<"path:"<<path<<endl;
//		cout<<"name:"<<name<<endl;
//		cout<<"filename:"<<filename<<endl;
		if (!doc->LoadFile(filename))
			cout << "Error in loading file " << filename << endl;
		TiXmlElement* originalE = e;
		e = doc->RootElement();
		if (!this->getRealContent()) {
			cout << "Error in read file of Kinbody!\n";
			return false;
		}
		e = originalE;
		this->getRealContent();
	} else {
		this->getRealContent();
	}
	return true;
}

bool OrXmlKinbody::getRealContent() {
	string bodystr = "Body";
	string kinstr = "KinBody";
	string jointstr = "Joint";
	string adjacentstr = "adjacent";
	e->QueryValueAttribute("name", &name);
	e->QueryValueAttribute("type", &type);
	e->QueryValueAttribute("prefix", &prefix);

	TiXmlElement* c = getFirstChild(e, "offsetfrom");
	if (c) {
		cout << "Error: no support for offsetfrom tag in Kinbody!\n" << flush;
		getchar();
		return false;
	}
	TiXmlElement* c2 = getFirstChild(e, "KinBody");
	while (c2) {
		OrXmlKinbody* kinxml = new OrXmlKinbody(c2);
		kinxml->kin_dir_str = this->kin_dir_str;
		kinxml->GetContent();
		this->xmlKinbodys.push_back(kinxml);
		c2 = getNextSibling(c2, "KinBody");
	}
	TiXmlElement* c1 = getFirstChild(e, "Body");
	while (c1) {
		OrXmlBody* bodxml = new OrXmlBody(c1);
		bodxml->GetContent();
		this->xmlBodys.push_back(bodxml);
		c1 = getNextSibling(c1, "Body");
	}
	TiXmlElement* c3 = getFirstChild(e, "Joint");
	while (c3) {
		OrXmlJoint* orxml = new OrXmlJoint(c3);
		orxml->GetContent();
		this->xmlJoints.push_back(orxml);
		c3 = getNextSibling(c3, "Joint");
	}
	TiXmlElement* c4 = getFirstChild(e, "adjacent");
	while (c4) {
		OrXmlAdjacent* orxml = new OrXmlAdjacent(c4);
		orxml->GetContent();
		this->xmlAdjacents.push_back(orxml);
		c4 = getNextSibling(c4, "adjacent");
	}
	if (hasTransformation(e)) {
		transformation = new OrXmlTransformation(e);
		transformation->GetContent();
	}
	c = getFirstChild(e, "Mass");
	if (c) {
		mass = new OrXmlMass(c);
		mass->GetContent();
	}
	c = getFirstChild(e, "modelsdir");
	if (c) {
		stringstream ss;
		ss << c->GetText();
		ss >> modelsdir;

		MODEL_DIR = this->kin_dir_str + modelsdir;
		MODEL_DIR.append("/");
	}
	c = getFirstChild(e, "offsetfrom");
	if (c) {
		stringstream ss;
		ss << c->GetText();
		ss >> this->offsetfrom;
	}
	c = getFirstChild(e, "jointvalues");
	if (c) {
		size_t nJoints = this->xmlJoints.size();
		stringstream ss;
		ss << c->GetText();
		for (size_t i = 0; i < nJoints; i++) {
			double tmp;
			ss >> tmp;
			this->jointvalues.push_back(tmp);
		}
	}
	return true;
}

bool OrXmlKinbody::GetObjectOrTerrain(WorldModel& world) {
	if (!isGetAllCleanBody) {
		cout
				<< "Error: should first process all the contents by GetAllCleanBodyJoints!\n"
				<< endl;
		getchar();
		return false;
	}
	for (size_t i = 0; i < this->xmlBodys.size(); i++) {
//		if()
	}
//	if(0 == strcmp(xmlKinbodys[i]->type.c_str(), "static")){
//		Environment* t = new Environment;
//		int i = world.AddTerrain(xmlKinbodys[i]->name, t);
//	}else{
//		RigidObject* o = new RigidObject;
//		int i = world.AddRigidObject(xmlKinbodys[i]->name, o);
//	}
	return true;
}

OrXmlBody::OrXmlBody(TiXmlElement* element) {
	e = element;
	this->mass = 0;
	this->transformation = 0;
	Tworld = 0;
	Tparent = 0;
//	this->combinedGeom = 0;
}

OrXmlBody::~OrXmlBody() {
//	if(combinedGeom)
//		delete combinedGeom;
	if (mass)
		delete mass;
	if (transformation)
		delete transformation;
	if (Tworld)
		delete Tworld;
	if (Tparent)
		delete Tparent;
	for (size_t i = 0; i < xmlGeoms.size(); i++) {
		if (xmlGeoms[i])
			delete xmlGeoms[i];
	}
}

bool OrXmlBody::GetContent() {
	e->QueryValueAttribute("type", &type);
	e->QueryValueAttribute("name", &name);

	TiXmlElement* c = getFirstChild(e, "Geom");
	while (c) {
		OrXmlGeom* geom = new OrXmlGeom(c);
		if (geom->GetContent())
			xmlGeoms.push_back(geom);
		else
			delete geom;
		c = getNextSibling(c, "Geom");
	}

	c = getFirstChild(e, "Mass");
	if (c) {
		mass = new OrXmlMass(c);
		mass->GetContent();
	}
	c = getFirstChild(e, "offsetfrom");
	if (c) {
		stringstream ss;
		ss << c->GetText();
		ss >> offsetfrom;
	}
	cout<<name<<endl;

	if (hasTransformation(e)) {
		cout<<"hasTransformation:"<<hasTransformation(e)<<endl;
		transformation = new OrXmlTransformation(e);
		transformation->GetContent();
		cout<<transformation->translation.size();
	}
	return true;
}

void OrXmlBody::output2Wrl() {
	if (name.empty()) {
		cout << "No body name specified!Name it body_" << vispoints.size()
				<< ".tri\n" << flush;
	}
	ofstream file;
	if (vispoints.size() > 0) {
		vis_filename = ROBOT_DIR + KINBODY_DIR + MODEL_DIR + name;
		vis_filename.append("_vis.wrl");
		file.open(vis_filename.c_str());
		file << "#VRML V2.0 utf8\n\n";
		file << "DEF " << name << "\n";
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
		for (size_t i = 0; i < this->vispoints.size(); i++) {
			MyPoint3D p(vispoints[i]);
			file << "\t\t\t\t\t" << p.data[0] << "\t" << p.data[1] << "\t"
					<< p.data[2] << ",\n";
		}
		file << "\t\t\t\t\t ] \n";
		file << "\t\t\t\t } \n";
		file << "\t\t\t\t coordIndex [ \n";
		for (size_t i = 0; i < this->visindexes.size(); i++) {
			TriFaceIndex fi(visindexes[i]);
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
	if (colpoints.size() > 0) {
		col_filename = ROBOT_DIR + KINBODY_DIR + MODEL_DIR + name;
		col_filename.append("_col.wrl");
		file.open(col_filename.c_str());
		file << "#VRML V2.0 utf8\n\n";
		file << "DEF " << name << "\n";
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
		for (size_t i = 0; i < this->colpoints.size(); i++) {
			MyPoint3D p(colpoints[i]);
			file << "\t\t\t\t\t" << p.data[0] << "\t" << p.data[1] << "\t"
					<< p.data[2] << ",\n";
		}
		file << "\t\t\t\t\t ] \n";
		file << "\t\t\t\t } \n";
		file << "\t\t\t\t coordIndex [ \n";
		for (size_t i = 0; i < this->colindexes.size(); i++) {
			TriFaceIndex fi(colindexes[i]);
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

void OrXmlBody::output2Tri() {
	if (name.empty()) {
		cout << "No body name specified!Name it body_" << vispoints.size()
				<< ".tri\n" << flush;
	}
	ofstream file;
	if (vispoints.size() > 0) {
		vis_filename = ROBOT_DIR + KINBODY_DIR + MODEL_DIR + name;
		vis_filename.append("_vis.tri");

		file.open(vis_filename.c_str());
		file << this->vispoints.size() << endl;
		for (size_t i = 0; i < vispoints.size(); i++) {
			file << vispoints[i].data[0] << "\t" << vispoints[i].data[1] << "\t"
					<< vispoints[i].data[2] << endl;
		}
		file << this->visindexes.size() << endl;
		for (size_t i = 0; i < visindexes.size(); i++) {
			file << visindexes[i].data[0] << "\t" << visindexes[i].data[1]
					<< "\t" << visindexes[i].data[2] << endl;
		}
		file.close();
	}

	if (colpoints.size() > 0) {
		col_filename = ROBOT_DIR + KINBODY_DIR + MODEL_DIR + name;
		col_filename.append("_col.tri");
		file.open(col_filename.c_str());
		file << this->colpoints.size() << endl;
		for (size_t i = 0; i < colpoints.size(); i++) {
			file << colpoints[i].data[0] << "\t" << colpoints[i].data[1] << "\t"
					<< colpoints[i].data[2] << endl;
		}
		file << this->colindexes.size() << endl;
		for (size_t i = 0; i < colindexes.size(); i++) {
			file << colindexes[i].data[0] << "\t" << colindexes[i].data[1]
					<< "\t" << colindexes[i].data[2] << endl;
		}
		file.close();
	}
}

void OrXmlBody::GetMimicGeomMass() {
	if (mass && 0 == strcmp(mass->type.c_str(), "mimicgeom")) {
//		cout<<"body "<<name<<" size "<<xmlGeoms.size()<<flush;getchar();
		OrXmlMass* finalMass = new OrXmlMass();
		finalMass->com = new Vector3;
		finalMass->com->setZero();
		finalMass->inertia = new Matrix3;
		finalMass->inertia->setZero();
		finalMass->total = new double;
		*(finalMass->total) = 0.0;

		for (size_t i = 0; i < this->xmlGeoms.size(); i++) {
			OrXmlMass* tmpmass = new OrXmlMass();
			tmpmass->com = new Vector3;
			tmpmass->com->setZero();
			tmpmass->inertia = new Matrix3;
			tmpmass->inertia->setZero();
			tmpmass->total = new double;
			*tmpmass->total = 0.0;

			RigidTransform tran;
			tran.setIdentity();

			if (xmlGeoms[i]->transformation) {
				xmlGeoms[i]->transformation->ComputeTransform();
				tran.set(*(xmlGeoms[i]->transformation->T));
			}
			if (mass->total) {
				*tmpmass->total = *mass->total;
			}
			if (mass->density) {
				tmpmass->density = new double();
				*tmpmass->density = *mass->density;
			}
			if (0 == strcmp(xmlGeoms[i]->type.c_str(), "box")) {
				tmpmass->extents = new Vector3(*(xmlGeoms[i]->extents));
				tmpmass->GetBoxMass();
			} else if (0 == strcmp(xmlGeoms[i]->type.c_str(), "sphere")) {
				tmpmass->radius = new double();
				*tmpmass->radius = *xmlGeoms[i]->radius;
				tmpmass->GetSphereMass();
			} else if (0 == strcmp(xmlGeoms[i]->type.c_str(), "cylinder")) {
				tmpmass->radius = new double();
				*tmpmass->radius = *xmlGeoms[i]->radius;
				tmpmass->height = new double();
				*tmpmass->height = *xmlGeoms[i]->height;
				tmpmass->GetCylinderMass();
			}

			tmpmass->ChangeCoordinateSystem(tran);

			finalMass->AddMass(tmpmass);

			delete tmpmass;
		}
		if (!mass->com)
			mass->com = new Vector3;
		if (!mass->inertia)
			mass->inertia = new Matrix3;
		if (!mass->total)
			mass->total = new double;
		mass->com->set(*(finalMass->com));
		mass->inertia->set(*(finalMass->inertia));
		*mass->total = *finalMass->total;

		delete finalMass;
	}
}

bool OrXmlBody::GetFinalGeom() {
	if (this->xmlGeoms.size() == 0) {
		return true;
	}

	for (size_t i = 0; i < xmlGeoms.size(); i++) {
		xmlGeoms[i]->Convert2Tri();
	}
//	int count = 0;
	for (size_t i = 0; i < xmlGeoms.size(); i++) {
//		if (xmlGeoms[i]->visindexes.size() > 0)
//			count++;
		addMesh(xmlGeoms[i]->vispoints, xmlGeoms[i]->visindexes, true);
		addMesh(xmlGeoms[i]->colpoints, xmlGeoms[i]->colindexes, false);
		xmlGeoms[i]->vispoints.resize(0);
		xmlGeoms[i]->visindexes.resize(0);
		xmlGeoms[i]->colpoints.resize(0);
		xmlGeoms[i]->colindexes.resize(0);
	}

//	if (count >= 2) {
//		cout << "Body:" << name << " :" << count << endl << flush;
//		getchar();
//	}
	this->output2Tri();
	return true;
}

bool OrXmlBody::applyTran2Tparent(OrXmlTransformation* tran) {
	if (!tran)
		return true;
	tran->ComputeTransform();

	if (!this->transformation) {
		transformation = new OrXmlTransformation();
	}
	transformation->ComputeTransform();
	RigidTransform finalTran;
	finalTran.mul(*tran->T, *transformation->T);
	transformation->T->set(finalTran);
	return true;
}

void OrXmlBody::addMesh(const vector<MyPoint3D>& pts,
		const vector<TriFaceIndex>& tis, bool visible) {
	if (visible) {
		int baseIndex = vispoints.size();
		for (size_t i = 0; i < pts.size(); i++) {
			vispoints.push_back(pts[i]);
		}
		for (size_t i = 0; i < tis.size(); i++) {
			TriFaceIndex t1(tis[i]);
			TriFaceIndex t2(t1.data[0] + baseIndex, t1.data[1] + baseIndex,
					t1.data[2] + baseIndex);
			visindexes.push_back(t2);
		}
	} else {
		int baseIndex = colpoints.size();
		for (size_t i = 0; i < pts.size(); i++) {
			colpoints.push_back(pts[i]);
		}
		for (size_t i = 0; i < tis.size(); i++) {
			TriFaceIndex t1(tis[i]);
			TriFaceIndex t2(t1.data[0] + baseIndex, t1.data[1] + baseIndex,
					t1.data[2] + baseIndex);
			colindexes.push_back(t2);
		}
	}
}

OrXmlRobot::OrXmlRobot(TiXmlElement* element) {
	this->e = element;
	this->transformation = 0;
	this->doc = 0;
	this->hasSetPrefix = false;
	this->allclean = false;
}

OrXmlRobot::~OrXmlRobot() {
	if (doc)
		delete doc;
	if (transformation)
		delete transformation;
	////////////////////////////
	for (size_t i = 0; i < this->xmlAdjacents.size(); i++) {
		if (xmlAdjacents[i])
			delete xmlAdjacents[i];
	}
	for (size_t i = 0; i < this->xmlJoints.size(); i++) {
		if (xmlJoints[i])
			delete xmlJoints[i];
	}
	for (size_t i = 0; i < this->xmlBodys.size(); i++) {
		if (xmlBodys[i])
			delete xmlBodys[i];
	}
	////////////////////////////
	for (size_t i = 0; i < this->xmlKinbodys.size(); i++) {
		if (xmlKinbodys[i])
			delete xmlKinbodys[i];
	}
	for (size_t i = 0; i < this->xmlRobots.size(); i++) {
		if (xmlRobots[i])
			delete xmlRobots[i];
	}
}

bool OrXmlRobot::ReSetBodyJointOrder() {
///////////////////////////////////////Reset body Order
	vector<OrXmlBody*> newbodys;
	vector<int> bodystack;

	bodystack.push_back(linkroot);
	while (bodystack.size() > 0) {
		int bodyindex = bodystack.back();
		bodystack.pop_back();

		newbodys.push_back(xmlBodys[bodyindex]);

		for (size_t i = 0; i < children[bodyindex].size(); i++) {
			bodystack.push_back(children[bodyindex][i]);
		}
	}

	size_t nbody = xmlBodys.size();
	vector<int> newparents;
	vector<vector<int> > newchildren;
	newparents.resize(nbody);
	newchildren.resize(nbody);

	for (size_t i = 0; i < nbody; i++) {
		int bodyindex = this->getLinkIndex(newbodys[i]->name);
		if (parents[bodyindex] == -1) {
			newparents[i] = -1;
		} else {
			string parentname = xmlBodys[parents[bodyindex]]->name;
			for (size_t j = 0; j < nbody; j++) {
				if (0
						== strcmp(newbodys[j]->name.c_str(),
								parentname.c_str())) {
					newparents[i] = j;
					break;
				}
			}
		}
		for (size_t j = 0; j < children[bodyindex].size(); j++) {
			string childname = xmlBodys[children[bodyindex][j]]->name;
			for (size_t k = 0; k < nbody; k++) {
				if (0 == strcmp(newbodys[k]->name.c_str(), childname.c_str())) {
					newchildren[i].push_back(k);
					break;
				}
			}
		}
	}

	xmlBodys.clear();
	parents.clear();
	for (size_t i = 0; i < children.size(); i++)
		children[i].clear();
	children.clear();
	children.resize(nbody);

	for (size_t i = 0; i < nbody; i++) {
		xmlBodys.push_back(newbodys[i]);
		parents.push_back(newparents[i]);
		for (size_t j = 0; j < newchildren[i].size(); j++)
			children[i].push_back(newchildren[i][j]);
	}
///////////////////////////////////////Reset Joints Order
	for (size_t i = 0; i < xmlJoints.size(); i++) {
		assert(xmlJoints[i]->bodys.size() == 2);
		int b1 = getLinkIndex(xmlJoints[i]->bodys[0]);
		int b2 = getLinkIndex(xmlJoints[i]->bodys[1]);
		assert(b1 != -1);
		assert(b2 != -1);
		if (parents[b1] == b2) {
			xmlJoints[i]->linkI = b1 + 5;
		} else if (parents[b2] == b1) {
			xmlJoints[i]->linkI = b2 + 5;
		} else {
			cout << "Invalid joint " << xmlJoints[i]->name << ";"
					<< xmlJoints[i]->bodys[0] << ";" << xmlJoints[i]->bodys[1]
					<< endl;
			getchar();
			return false;
		}
	}
	size_t nJoint = xmlJoints.size();
	for (size_t i = 0; i < nJoint; i++) {
		for (size_t j = 0; j < i; j++) {
			if (xmlJoints[i]->linkI < xmlJoints[j]->linkI) {
				OrXmlJoint* tmp = xmlJoints[i];
				xmlJoints[i] = xmlJoints[j];
				xmlJoints[j] = tmp;
			}
		}
	}

	return true;
}

bool OrXmlRobot::GetParentChildRelation() {
	if (xmlBodys.size() == 0)
		return true;
	parents.resize(xmlBodys.size());
	children.resize(xmlBodys.size());
	for (size_t i = 0; i < this->xmlBodys.size(); i++) {
		parents[i] = -1;
	}
	for (size_t i = 0; i < this->xmlBodys.size(); i++) {
		if (xmlBodys[i]->offsetfrom.empty())
			continue;
		for (size_t j = 0; j < this->xmlBodys.size(); j++) {
			if (strcmp(xmlBodys[i]->offsetfrom.c_str(),
					xmlBodys[j]->name.c_str()) == 0) {
				parents[i] = j;
				children[j].push_back(i);
				break;
			}
		}
	}
	//infer parent children relationship from joints
	for (size_t i = 0; i < this->xmlJoints.size(); i++) {
		if (xmlJoints[i]->bodys.size() != 2) {
			cout << "Error: joint " << xmlJoints[i]->name << " bodys size="\

					<< xmlJoints[i]->bodys.size() << endl << flush;
			getchar();
			return false;
		}
		string b1 = xmlJoints[i]->bodys[0];
		string b2 = xmlJoints[i]->bodys[1];
		int index1 = -1;
		int index2 = -1;
		for (size_t j = 0; j < this->xmlBodys.size(); j++) {
			if (0 == strcmp(xmlBodys[j]->name.c_str(), b1.c_str()))
				index1 = j;
			if (0 == strcmp(xmlBodys[j]->name.c_str(), b2.c_str()))
				index2 = j;
			if (index1 != -1 && index2 != -1)
				break;
		}
		if (index1 == -1 && index2 == -1) {
			cout << "Error: Joint " << xmlJoints[i]->name << " invalid!\n";
			cout << "b1:" << b1 << ";b2:" << b2 << flush;
			getchar();
			return false;
		}
		//if b1 has no parent and b1 is defined after b2
		if (parents[index1] == -1 && index1 > index2) {
			parents[index1] = index2;
			children[index2].push_back(index1);
		} else if (parents[index2] == -1 && index2 > index1) {
			parents[index2] = index1;
			children[index1].push_back(index2);
		}
	}

	linkroot = -1;
	for (size_t i = 0; i < parents.size(); i++) {
		if (parents[i] == -1) {
			linkroot = i;
			break;
		}
	}
	return true;
}

bool OrXmlRobot::applyTran2Tparent() {
	if (!transformation)
		return true;
	if (!allclean) {
		cout
				<< "Error: robot transformation should be applied after everything else is processed!\n"
				<< flush;
		getchar();
		return false;
	}
	transformation->ComputeTransform();
	for (size_t i = 0; i < this->xmlBodys.size(); i++) {
		if (parents[i] == -1) {
			RigidTransform oldT(*xmlBodys[i]->Tparent);
			xmlBodys[i]->Tparent->mul(*(transformation->T), oldT);
			xmlBodys[i]->Tworld->set(*(xmlBodys[i]->Tparent));
		}
	}
	return true;
}

bool OrXmlRobot::GetContent() {
	if (e->QueryValueAttribute("file", &file) == TIXML_SUCCESS) {
		string filename_string = ROBOT_DIR + file;
		const char* filename = filename_string.c_str();
		doc = new TiXmlDocument();
		if (!doc->LoadFile(filename))
			cout << "Error in loading file " << filename << endl;
		TiXmlElement* originalE = e;
		e = doc->RootElement();
		if (!this->getRealContent()) {
			cout << "Error in read file of Kinbody!\n";
			return false;
		}
		e = originalE;
		this->getRealContent();
	} else {
		this->getRealContent();
	}
	return true;
}

bool OrXmlRobot::getRealContent() {
	if (e->QueryValueAttribute("name", &name) != TIXML_SUCCESS)
		name.assign("robot");
	e->QueryValueAttribute("type", &type);
	e->QueryValueAttribute("prefix", &prefix);
	TiXmlElement* c1 = getFirstChild(e, "KinBody");
	while (c1) {
		OrXmlKinbody* kinxml = new OrXmlKinbody(c1);
		kinxml->GetContent();
		this->xmlKinbodys.push_back(kinxml);
		c1 = getNextSibling(c1, "KinBody");
	}
	TiXmlElement* c2 = getFirstChild(e, "Robot");
	while (c1) {
		OrXmlRobot* robxml = new OrXmlRobot(c2);
		robxml->getRealContent();
		this->xmlRobots.push_back(robxml);
		c2 = getNextSibling(c2, "Robot");
	}
	if (hasTransformation(e)) {
		transformation = new OrXmlTransformation(e);
		transformation->GetContent();
	}
	TiXmlElement*c = getFirstChild(e, "jointvalues");
	if (c) {
		stringstream ss;
		ss << c->GetText();
		while (!ss.eof()) {
			double tmp;
			ss >> tmp;
			this->jointvalues.push_back(tmp);
		}
	}
	return true;
}

void OrXmlRobot::setPrefixName() {
	if (this->hasSetPrefix || prefix.empty())
		return;
	for (size_t i = 0; i < this->xmlKinbodys.size(); i++) {
		string tmp = prefix;
		tmp.append(xmlKinbodys[i]->prefix.c_str());
		xmlKinbodys[i]->prefix = tmp;
	}
	for (size_t i = 0; i < this->xmlRobots.size(); i++) {
		string tmp = prefix;
		tmp.append(xmlRobots[i]->prefix.c_str());
		xmlRobots[i]->prefix = tmp;
	}
	this->hasSetPrefix = true;
}

//copy extra properties of b2 to b1
void OrXmlRobot::combineTwoBody(OrXmlBody* b1, OrXmlBody* b2) {
	if (0 != strcmp(b1->name.c_str(), b2->name.c_str())) {
		cout << "Error: Bodies' names are not the same!\n" << flush;
		getchar();
	}
	if ((b1->xmlGeoms.size() > 0 && b2->xmlGeoms.size() > 0)\

			|| (b1->mass && b2->mass)
			|| (b1->transformation && b2->transformation)
			|| (!b1->offsetfrom.empty() && !b2->offsetfrom.empty())\

			|| (b1->vispoints.size() > 0 && b2->vispoints.size() > 0)\

			|| (b1->colpoints.size() > 0 && b2->colpoints.size() > 0)) {
		cout << "Error: both bodies have same properties!\n" << flush;
		getchar();
	}
	if (!b2->offsetfrom.empty() && b1->offsetfrom.empty()) {
		b1->offsetfrom = b2->offsetfrom;
	}
	if (!b2->vis_filename.empty() && b1->vis_filename.empty()) {
		b1->vis_filename = b2->vis_filename;
	}
	if (!b2->col_filename.empty() && b1->col_filename.empty()) {
		b1->col_filename = b2->col_filename;
	}
	if (b2->mass && !b1->mass) {
		b1->mass = b2->mass;
		b2->mass = 0;
	}
	if (b2->transformation && !b1->transformation) {
		b1->transformation = b2->transformation;
		b2->transformation = 0;
	}
	if (!b2->type.empty() && b1->type.empty())
		b1->type = b2->type;
	if (b2->xmlGeoms.size() > 0 && b1->xmlGeoms.size() == 0) {
		for (size_t i = 0; i < b2->xmlGeoms.size(); i++) {
			b1->xmlGeoms.push_back(b2->xmlGeoms[i]);
			b2->xmlGeoms[i] = 0;
		}
	}
	if (b2->vispoints.size() > 0 && b1->vispoints.size() == 0) {
		for (size_t i = 0; i < b2->vispoints.size(); i++)
			b1->vispoints.push_back(b2->vispoints[i]);
		for (size_t i = 0; i < b2->visindexes.size(); i++)
			b1->visindexes.push_back(b2->visindexes[i]);
		b2->vispoints.clear();
		b2->visindexes.clear();
	}
	if (b2->colpoints.size() > 0 && b1->colpoints.size() == 0) {
		for (size_t i = 0; i < b2->colpoints.size(); i++)
			b1->colpoints.push_back(b2->colpoints[i]);
		for (size_t i = 0; i < b2->colindexes.size(); i++)
			b1->colindexes.push_back(b2->colindexes[i]);
		b2->colpoints.clear();
		b2->colindexes.clear();
	}
}

bool OrXmlRobot::RemoveSameBody() {
	vector<OrXmlBody*> newBodys;
	for (size_t i = 0; i < xmlBodys.size(); i++) {
		bool repeat = false;
		for (size_t j = 0; j < newBodys.size(); j++) {
			if (0
					== strcmp(xmlBodys[i]->name.c_str(),
							newBodys[j]->name.c_str())) {
				combineTwoBody(newBodys[j], xmlBodys[i]);
//				cout<<newBodys[j]->name<<";"<<newBodys[j]->offsetfrom<<endl;
				repeat = true;
				break;
			}
		}
		if (false == repeat)
			newBodys.push_back(xmlBodys[i]);
	}
	xmlBodys.clear();
	for (size_t i = 0; i < newBodys.size(); i++) {
		xmlBodys.push_back(newBodys[i]);
	}
	return true;
}

bool OrXmlRobot::GetAllCleanBodyJoints() {
	this->setPrefixName();
	for (size_t i = 0; i < this->xmlKinbodys.size(); i++) {
		xmlKinbodys[i]->GetAllCleanBodyJoints();
		for (size_t j = 0; j < xmlKinbodys[i]->xmlBodys.size(); j++) {
			OrXmlBody* body = xmlKinbodys[i]->xmlBodys[j];
			xmlBodys.push_back(body);
			xmlKinbodys[i]->xmlBodys[j] = 0;
		}
		for (size_t j = 0; j < xmlKinbodys[i]->xmlJoints.size(); j++) {
			OrXmlJoint* joint = xmlKinbodys[i]->xmlJoints[j];
			xmlJoints.push_back(joint);
			xmlKinbodys[i]->xmlJoints[j] = 0;
		}
		for (size_t j = 0; j < xmlKinbodys[i]->xmlAdjacents.size(); j++) {
			OrXmlAdjacent* adjacent = xmlKinbodys[i]->xmlAdjacents[j];
			xmlAdjacents.push_back(adjacent);
			xmlKinbodys[i]->xmlAdjacents[j] = 0;
		}
		if (jointvalues.size() == 0) {
			for (size_t j = 0; j < xmlKinbodys[i]->jointvalues.size(); j++) {
				double jv = xmlKinbodys[i]->jointvalues[j];
				jointvalues.push_back(jv);
				xmlKinbodys[i]->jointvalues[j] = 0;
			}
		}
	}
	for (size_t i = 0; i < this->xmlRobots.size(); i++) {
		xmlRobots[i]->GetAllCleanBodyJoints();
		for (size_t j = 0; j < xmlRobots[i]->xmlBodys.size(); j++) {
			OrXmlBody* body = xmlRobots[i]->xmlBodys[j];
			xmlBodys.push_back(body);
			xmlRobots[i]->xmlBodys[j] = 0;
		}
		for (size_t j = 0; j < xmlRobots[i]->xmlJoints.size(); j++) {
			OrXmlJoint* joint = xmlRobots[i]->xmlJoints[j];
			xmlJoints.push_back(joint);
			xmlRobots[i]->xmlJoints[j] = 0;
		}
		for (size_t j = 0; j < xmlRobots[i]->xmlAdjacents.size(); j++) {
			OrXmlAdjacent* adjacent = xmlRobots[i]->xmlAdjacents[j];
			xmlAdjacents.push_back(adjacent);
			xmlRobots[i]->xmlAdjacents[j] = 0;
		}
		for (size_t j = 0; j < xmlRobots[i]->jointvalues.size(); j++) {
			double jv = xmlRobots[i]->jointvalues[j];
			jointvalues.push_back(jv);
			xmlRobots[i]->jointvalues[j] = 0;
		}
	}
	for (size_t i = 0; i < this->xmlJoints.size(); i++) {
		xmlJoints[i]->GetCleanJointInfo();
	}
	this->RemoveSameBody();
	this->GetParentChildRelation();
	this->ReSetBodyJointOrder();
	this->GetTransformAnchorAxis();
	this->ProcessJointAnchorInitial();
	allclean = true;
	this->applyTran2Tparent();

	nDOF = xmlJoints.size();
	for (size_t i = 0; i < this->xmlJoints.size(); i++) {
		if (xmlJoints[i]->cleanLimits.size() == 2) {
			if (xmlJoints[i]->cleanLimits[0] == xmlJoints[i]->cleanLimits[1])
				nDOF--;
		}
	}
	nDOF += 5; //floating base
	return true;
}

bool OrXmlEnvironment::Load(const string &fn) {
	return doc.LoadFile(fn.c_str());
}

TiXmlElement* OrXmlEnvironment::GetElement(const string& name) {
	if (!doc.RootElement())
		return NULL;
	return doc.RootElement()->FirstChildElement(name);
}

TiXmlElement* OrXmlEnvironment::GetElement(const string& name, int index) {
	TiXmlElement* e = doc.RootElement()->FirstChildElement(name);
	while (index > 0) {
		if (!e)
			return NULL;
		index--;
		e = e->NextSiblingElement(name);
	}
	return e;
}

OrXmlEnvironment::OrXmlEnvironment() {
	this->xmlKinbodys.resize(0);
	this->xmlRobots.resize(0);
}

OrXmlEnvironment::~OrXmlEnvironment() {
	this->clear();
}

void OrXmlEnvironment::clear() {
	for (size_t i = 0; i < xmlKinbodys.size(); i++) {
		if (xmlKinbodys[i])
			delete xmlKinbodys[i];
	}
	for (size_t i = 0; i < xmlRobots.size(); i++) {
		if (xmlRobots[i])
			delete xmlRobots[i];
	}
}


bool OrXmlEnvironment::getWorld() {
	string file;
	if (doc.RootElement()->QueryValueAttribute("file", &file)
			== TIXML_SUCCESS) {
		cout << "Environment Tag from file is Not supported!\n";
		return false;
	}
	string envstr = "environment";
	string kinstr = "kinBody";
	string robstr = "robot";
	string bodstr = "body";
	TiXmlElement* root;
	root = this->doc.RootElement();
	TiXmlElement* e;
	string rootvalue = root->Value();
	rootvalue = ToLowercase(rootvalue);

	if (0 == strcmp(rootvalue.c_str(), robstr.c_str())) {
		OrXmlRobot* robxml = new OrXmlRobot(root);
		if (!robxml->GetContent()) {
			cout << "Robot read failed!\n" << flush;
			return false;
		}
		this->xmlRobots.push_back(robxml);
		return true;
	} else if (0 == strcmp(rootvalue.c_str(), kinstr.c_str())) {
		OrXmlKinbody* kinxml = new OrXmlKinbody(root);
		if (!kinxml->GetContent()) {
			cout << "Kinbody read failed!\n" << flush;
			return false;
		}
		this->xmlKinbodys.push_back(kinxml);
		return true;
	} else if (0 == strcmp(rootvalue.c_str(), envstr.c_str())) {
		e = GetElement(robstr);
		while (e) {
			OrXmlRobot* robxml = new OrXmlRobot(e);
			robxml->GetContent();
			this->xmlRobots.push_back(robxml);
			e = getNextSibling(e, "Robot");
		}
		e = GetElement(kinstr);
		while (e) {
			OrXmlKinbody* kinxml = new OrXmlKinbody(e);
			kinxml->GetContent();
			this->xmlKinbodys.push_back(kinxml);
			e = getNextSibling(e, "KinBody");
		}
		return true;
	}
	if(this->xmlRobots.size() == 0){
		cout<<"No robot found!\n"<<flush;
		return false;
	}
	return true;
}

bool OrXmlEnvironment::Convert2URDF() {
	//parse xml first
	if (!getWorld()) {
		cout << "getWorld failed!\n" << flush;
		return false;
	}
	cout << "*************Read in OR xml info!*************\n" << flush;
	for (size_t i = 0; i < this->xmlRobots.size(); i++) {
		if (TOURDF) {
			cout << "Write to URDF format!\n" << endl;
			if (!xmlRobots[i]->ConvertToURDF()) {
				cout << "ConvertToURDF failed!\n" << flush;
				return false;
			}
		}
	}

	return true;
}

bool OrXmlEnvironment::GetWorld(WorldModel& world) {
	//parse xml first
	if (!getWorld()) {
		cout << "getWorld failed!\n" << flush;
		return false;
	}
	cout << "*************Read in OR xml info!*************\n" << flush;
	for (size_t i = 0; i < this->xmlRobots.size(); i++) {
		Robot* r = new Robot;
		if (!xmlRobots[i]->GetRobot(*r)) {
			cout << "Get robot failed!\n" << flush;
			delete r;
			return false;
		}
		world.AddRobot(xmlRobots[i]->name, r);
		xmlRobots[i]->Write2Rob(*r);
	}

	return true;
}

bool OrXmlEnvironment::Convert2Rob() {
	//parse xml first
	if (!getWorld()) {
		cout << "getWorld failed!\n" << flush;
		return false;
	}
	//create the world
	for (size_t i = 0; i < this->xmlRobots.size(); i++) {
		Robot* r = new Robot;
		if (!xmlRobots[i]->GetRobot(*r)) {
			cout << "Get robot failed!\n" << flush;
			delete r;
			return false;
		}
		xmlRobots[i]->Write2Rob(*r);
	}
	return true;
}

bool OrXmlRobot::GetFixedBaseRobot(Robot& robot) {
	size_t nBody = xmlBodys.size();

	robot.accMax.resize(nBody);
	robot.accMax.setZero();
	robot.q.resize(nBody);
	robot.q.setZero();
	robot.dq.resize(nBody);
	robot.dq.setZero();
	robot.qMin.resize(nBody);
	robot.qMin.set(-Inf);
	robot.qMax.resize(nBody);
	robot.qMin.set(Inf);
	robot.velMin.resize(nBody);
	robot.velMin.set(-Inf);
	robot.velMax.resize(nBody);
	robot.velMax.set(-Inf);
	robot.torqueMax.resize(nBody);
	robot.torqueMax.set(Inf);
	robot.powerMax.resize(nBody);
	robot.powerMax.set(Inf);

	robot.parents.resize(nBody);
	robot.geometry.resize(nBody);
	robot.linkNames.resize(nBody);
	robot.links.resize(nBody);

	for (size_t i = 0; i < nBody; i++) {
		robot.parents[i] = parents[i];
//		if (!xmlBodys[i]->vis_filename.empty()) {
//			robot.LoadGeometry(i, xmlBodys[i]->vis_filename.c_str());
//		}

		if (LOADSimpleGEOM && !(xmlBodys[i]->col_filename.empty()))
			robot.LoadGeometry(i, xmlBodys[i]->col_filename.c_str());
		else
			robot.LoadGeometry(i, xmlBodys[i]->vis_filename.c_str());

		robot.linkNames[i] = xmlBodys[i]->name;

		robot.links[i].type = RobotLink3D::Revolute;
		robot.links[i].T0_Parent.set(*(xmlBodys[i]->Tparent));
		robot.links[i].w.set(0, 0, 1);

		if(!xmlBodys[i]->mass && xmlBodys[i]->xmlGeoms.size() > 0){
			OrXmlMass* ormass = new OrXmlMass();
			ormass->GetUnitSphereMass();
			robot.links[i].com.set(*ormass->com);
			robot.links[i].inertia.set(*ormass->inertia);
			robot.links[i].mass = (*ormass->total);
			delete ormass;
		}else{
			if (xmlBodys[i]->mass && xmlBodys[i]->mass->com)
				robot.links[i].com.set(*xmlBodys[i]->mass->com);
			else
				robot.links[i].com.setZero();

			if (xmlBodys[i]->mass && xmlBodys[i]->mass->inertia)
				robot.links[i].inertia.set(*xmlBodys[i]->mass->inertia);
			else
				robot.links[i].inertia.setZero();
			if (xmlBodys[i]->mass && xmlBodys[i]->mass->total)
				robot.links[i].mass = *xmlBodys[i]->mass->total;
			else
				robot.links[i].mass = 0;
		}
	}
	size_t nJoint = xmlJoints.size();
	robot.joints.resize(0);
	for (size_t i = 0; i < nJoint; i++) {
		if (xmlJoints[i]->enabled == false)
			continue;
		RobotModelJoint robjoint;
		robjoint.type = RobotModelJoint::Normal;
		int b1 = -1, b2 = -1;
		assert(xmlJoints[i]->bodys.size() == 2);
		for (size_t j = 0; j < xmlBodys.size(); j++) {
			if (0
					== strcmp(xmlBodys[j]->name.c_str(),
							xmlJoints[i]->bodys[0].c_str()))
				b1 = j;
			if (0
					== strcmp(xmlBodys[j]->name.c_str(),
							xmlJoints[i]->bodys[1].c_str()))
				b2 = j;
		}
		assert(b1 != -1);
		assert(b2 != -1);
		int linkI;
		if (parents[b1] == b2) {
			linkI = b1;
		} else if (parents[b2] == b1) {
			linkI = b2;
		} else {
			cout << "Invalid joint " << xmlJoints[i]->name << ";"
					<< xmlJoints[i]->bodys[0] << ";" << xmlJoints[i]->bodys[1]
					<< endl;
			getchar();
			return false;
		}
		robjoint.linkIndex = linkI;

		if (xmlJoints[i]->cleanLimits.size() == 2) {
			if (xmlJoints[i]->cleanLimits[0] == xmlJoints[i]->cleanLimits[1])
				robjoint.type = RobotModelJoint::Weld;
			else {
				robot.qMin[linkI] = xmlJoints[i]->cleanLimits[0];
				robot.qMax[linkI] = xmlJoints[i]->cleanLimits[1];
			}
		} else {
			robot.qMin[linkI] = -Inf;
			robot.qMax[linkI] = Inf;
		}
		if (xmlJoints[i]->cleanMaxVel.size() == 1) {
			robot.velMin[linkI] = -(xmlJoints[i]->cleanMaxVel[0]);
			robot.velMax[linkI] = xmlJoints[i]->cleanMaxVel[0];
		} else {
			robot.velMin[linkI] = -Inf;
			robot.velMax[linkI] = Inf;
		}
		if (xmlJoints[i]->cleanMaxAcc.size() == 1) {
			robot.accMax[linkI] = xmlJoints[i]->cleanMaxAcc[0];
		} else {
			robot.accMax[linkI] = -Inf;
		}
		if (xmlJoints[i]->cleanMaxTorque.size() == 1) {
			robot.torqueMax[linkI] = xmlJoints[i]->cleanMaxTorque[0];
		} else {
			robot.torqueMax[linkI] = Inf;
		}
		if (xmlJoints[i]->axis)
			robot.links[linkI].w.set(*xmlJoints[i]->axis);

		robot.joints.push_back(robjoint);
	}

	robot.driverNames.resize(0);
	robot.drivers.resize(0);
	for (size_t i = 0; i < nJoint; i++) {
		if (robot.joints[i].type == RobotModelJoint::Normal) {
			robot.driverNames.push_back(xmlJoints[i]->name);
			int linkI = robot.joints[i].linkIndex;
			RobotModelDriver d;
			d.type = RobotModelDriver::Normal;
			d.linkIndices.push_back(linkI);
			d.qmin = robot.qMin(linkI);
			d.qmax = robot.qMax(linkI);
			d.vmin = robot.velMin(linkI);
			d.vmax = robot.velMax(linkI);
			d.tmin = -robot.torqueMax(linkI);
			d.tmax = robot.torqueMax(linkI);
			d.amin = -robot.accMax(linkI);
			d.amax = robot.accMax(linkI);
			d.servoP = 0;
			d.servoI = 0;
			d.servoD = 0;
			d.dryFriction = 0;
			robot.drivers.push_back(d);
		}
	}
	robot.selfCollisions.resize(nBody, nBody, NULL);
	robot.envCollisions.resize(nBody, NULL);
	//robot.InitCollisions();
	robot.InitAllSelfCollisions();

	return true;
}

int OrXmlRobot::getLinkIndex(const string& linkname) {
	for (size_t i = 0; i < xmlBodys.size(); i++) {
		if (0 == strcmp(linkname.c_str(), xmlBodys[i]->name.c_str()))
			return i;
	}
	return -1;
}

void OrXmlRobot::GetTransformAnchorAxis() {
	vector<int> needprocess;
	needprocess.push_back(linkroot);
	while (needprocess.size() > 0) {
		int bodyindex = needprocess.back();
		needprocess.pop_back();

		OrXmlBody* body = xmlBodys[bodyindex];

		if (bodyindex == linkroot) {
			if (body->transformation && body->transformation->T) {
				body->Tparent = new RigidTransform(*(body->transformation->T));
				body->Tworld = new RigidTransform(*(body->transformation->T));
			} else {
				body->Tparent = new RigidTransform();
				body->Tworld = new RigidTransform();
				body->Tparent->setIdentity();
				body->Tworld->setIdentity();
			}
		} else {
			OrXmlBody* parentbody = xmlBodys[parents[bodyindex]];
			if (body->transformation && body->transformation->T) {
				if (body->offsetfrom.empty()) {
					RigidTransform inverseT;
					inverseT.setInverse(
							*(xmlBodys[parents[bodyindex]]->Tparent));
					body->Tparent = new RigidTransform();
					body->Tparent->mul(inverseT, *(body->transformation->T));
				} else {
					body->Tparent = new RigidTransform(
							*body->transformation->T);
				}
			} else {
				if (body->offsetfrom.empty()) {
					RigidTransform inverseT;
					inverseT.setInverse(*xmlBodys[parents[bodyindex]]->Tworld);
					body->Tparent = new RigidTransform(inverseT);
				} else {
					body->Tparent = new RigidTransform();
					body->Tparent->setIdentity();
				}
			}
			body->Tworld = new RigidTransform();
			body->Tworld->mul(*parentbody->Tworld, *body->Tparent);
		}

		for (size_t i = 0; i < children[bodyindex].size(); i++) {
			needprocess.push_back(children[bodyindex][i]);
		}
	}
	for (size_t i = 0; i < xmlBodys.size(); i++) {
		if (!xmlBodys[i]->Tparent || !xmlBodys[i]->Tworld) {
			cout << "Error: body " << xmlBodys[i]->name
					<< " no Tparent or Tworld!\n" << flush;
			getchar();
		}
	}

//	ofstream file;
//	file.open("bodylist.txt");
//	for(size_t i = 0; i < parents.size(); i++){
//		file<<i<<" "<<xmlBodys[i]->name<<":offsetfrom="<<xmlBodys[i]->offsetfrom<<": parent="<<parents[i]<<";children=(";
//		for(size_t j = 0; j < children[i].size(); j++)
//			file<<children[i][j]<<",";
//		file<<")"<<endl;
//		if(0==strcmp(xmlBodys[i]->name.c_str(),"rightIndexProximal")){
//			cout<<xmlBodys[i]->name<<endl;
//			if(xmlBodys[i]->Tparent)cout<<*xmlBodys[i]->Tparent<<flush;getchar();
//		}
//	}
//	file.close();

	//////////////////////////////////////////////////////
	for (size_t i = 0; i < xmlJoints.size(); i++) {
		int b1 = this->getLinkIndex(xmlJoints[i]->bodys[0]);
		int b2 = this->getLinkIndex(xmlJoints[i]->bodys[1]);
		if (xmlJoints[i]->anchor) {
			if (xmlJoints[i]->offsetfrom.empty()) {
				cout << xmlJoints[i]->name << " anchor without offsetfrom!\n"
						<< flush;
				getchar();
			}
			int linkindex = this->getLinkIndex(xmlJoints[i]->offsetfrom);
			int tmpparent = b1;
			int tmpchild = b2;
			if (parents[b1] == b2) {
				tmpparent = b2;
				tmpchild = b1;
			} else if (parents[b2] == b1) {
				tmpparent = b1;
				tmpchild = b2;
			} else {
				cout << "Error: b1 b2 invalid!\n" << flush;
				getchar();
			}
			//transform anchor to child's frame
			if (linkindex == tmpparent) {
//				cout<<"offset from parents"<<endl;getchar();
				Vector3 anchor2;
				RigidTransform inverseT;
				inverseT.setInverse(*(xmlBodys[tmpchild]->Tparent));
				inverseT.mul(*xmlJoints[i]->anchor, anchor2);
				xmlJoints[i]->anchor->set(anchor2);
				if (xmlJoints[i]->axis) {
					Vector3 v1, v0, v;
					inverseT.setInverse(*(xmlBodys[tmpchild]->Tparent));
					inverseT.mul(*(xmlJoints[i]->axis), v1);
					v0.set(inverseT.t);
					v = v1 - v0;
					double norm = v.norm();
					xmlJoints[i]->axis->set(v[0] / norm, v[1] / norm,
							v[2] / norm);
				}
			}else{
//				cout<<"offset from child"<<endl;getchar();
			}
		}
	}
}

void OrXmlRobot::ProcessJointAnchorInitial() {
	//////////////////////////////////////////////////////
	for (size_t i = 0; i < xmlJoints.size(); i++) {
		int b1 = this->getLinkIndex(xmlJoints[i]->bodys[0]);
		int b2 = this->getLinkIndex(xmlJoints[i]->bodys[1]);
		int tmpparent = b1;
		int tmpchild = b2;
		if (xmlJoints[i]->anchor) {
			if (xmlJoints[i]->offsetfrom.empty()) {
				cout << xmlJoints[i]->name << " anchor without offsetfrom!\n"
						<< flush;
				getchar();
			}
			int linkindex = this->getLinkIndex(xmlJoints[i]->offsetfrom);
			if (parents[b1] == b2) {
				tmpparent = b2;
				tmpchild = b1;
			} else if (parents[b2] == b1) {
				tmpparent = b1;
				tmpchild = b2;
			} else {
				cout << "Error: b1 b2 invalid!\n" << flush;
				getchar();
			}
//			if (!xmlBodys[tmpchild]->vis_filename.empty()) {
//
//			}
			this->rewriteTriMesh(tmpchild, *xmlJoints[i]->anchor, true);
			if (TOURDF || LOADSimpleGEOM)
				this->rewriteTriMesh(tmpchild, *xmlJoints[i]->anchor, false);
			this->setNewRelativeTran(tmpchild, *xmlJoints[i]->anchor);
		}
		if(xmlJoints[i]->initial.size() == 1){
			RigidTransform T;
			double qi = xmlJoints[i]->initial[0];
			  if(xmlJoints[i]->type == "slider"){
			    T.R.setIdentity();
			    T.t.mul(*xmlJoints[i]->axis,qi);
			  }else if(xmlJoints[i]->type == "hinge"){
			    T.t.setZero();
			    if(xmlJoints[i]->axis->x == One)
			      T.R.setRotateX(qi);
			    else if(xmlJoints[i]->axis->y == One)
			      T.R.setRotateY(qi);
			    else if(xmlJoints[i]->axis->z == One)
			      T.R.setRotateZ(qi);
			    else {
			      //cout<<"Not a standard axis: "<<w<<endl;
			      MomentRotation r(qi*(*xmlJoints[i]->axis));
			      r.getMatrix(T.R);
			    }
			  }else{
				  cerr<<"Invalid joint type "<<xmlJoints[i]->type<<endl;
			    getchar();
			  }
			  RigidTransform newTP;
			  newTP.mul(*this->xmlBodys[tmpchild]->Tparent, T);
			  this->xmlBodys[tmpchild]->Tparent->set(newTP);
		}else if(xmlJoints[i]->initial.size() != 1 && xmlJoints[i]->initial.size() > 0){
			cout<<"No code for handling this situation where initial tag contains multiple values!"<<endl;
			getchar();
		}
	}
}

void OrXmlRobot::setNewRelativeTran(int tmpchild, const Vector3& an) {
	RigidTransform tmpT;
	tmpT.setIdentity();
	tmpT.t.set(an);
	RigidTransform oldT(*xmlBodys[tmpchild]->Tparent);
	xmlBodys[tmpchild]->Tparent->mul(oldT, tmpT);
	xmlBodys[tmpchild]->Tworld->mul(*xmlBodys[parents[tmpchild]]->Tworld,
			*xmlBodys[tmpchild]->Tparent);

	if (xmlBodys[tmpchild]->mass && xmlBodys[tmpchild]->mass->com) {
		Vector3 oldCom(*xmlBodys[tmpchild]->mass->com);
		xmlBodys[tmpchild]->mass->com->set(oldCom - an);
	}
	// if convert to URDF, no need to change the inertia because it's around the COM not origin of link frame
	if (false == TOURDF && xmlBodys[tmpchild]->mass
			&& xmlBodys[tmpchild]->mass->inertia) {
		Matrix3 tmpInertia(*xmlBodys[tmpchild]->mass->inertia);
		TranslateInertia2Origin(tmpInertia,
				*(xmlBodys[tmpchild]->mass->com),
				*(xmlBodys[tmpchild]->mass->total));
		xmlBodys[tmpchild]->mass->inertia->set(tmpInertia);
	}

	for (size_t j = 0; j < children[tmpchild].size(); j++) {
		int bodyindex = children[tmpchild][j];
		oldT.set(*xmlBodys[bodyindex]->Tparent);
		xmlBodys[bodyindex]->Tparent->set(oldT - an);
		xmlBodys[bodyindex]->Tworld->mul(*xmlBodys[parents[bodyindex]]->Tworld,
				*xmlBodys[bodyindex]->Tparent);
	}
}

void OrXmlRobot::rewriteTriMesh(int tmpchild, const Vector3& an, bool visible) {
	if (true == visible && xmlBodys[tmpchild]->vis_filename.empty())
		return;
	if (false == visible && xmlBodys[tmpchild]->col_filename.empty())
		return;
	vector<MyPoint3D> points;
	vector<TriFaceIndex> indexes;
	ifstream infile;
	if (visible) {
		infile.open(xmlBodys[tmpchild]->vis_filename.c_str());
	} else {
		infile.open(xmlBodys[tmpchild]->col_filename.c_str());
	}
	int npoints, nindexes;
	infile >> npoints;
	MyPoint3D pt;
	for (int i = 0; i < npoints; i++) {
		double p0, p1, p2;
		infile >> p0;
		infile >> p1;
		infile >> p2;
		pt.Set(p0 - an[0], p1 - an[1], p2 - an[2]);
		points.push_back(pt);
	}
	infile >> nindexes;
	TriFaceIndex ti;
	for (int i = 0; i < nindexes; i++) {
		int i0, i1, i2;
		infile >> i0;
		infile >> i1;
		infile >> i2;
		ti.Set(i0, i1, i2);
		indexes.push_back(ti);
	}
	infile.close();

	ofstream outfile;
	if (visible) {
		outfile.open(xmlBodys[tmpchild]->vis_filename.c_str());
	} else {
		outfile.open(xmlBodys[tmpchild]->col_filename.c_str());
	}
	outfile << npoints << endl;
	for (int i = 0; i < npoints; i++) {
		outfile << points[i].data[0] << "\t" << points[i].data[1] << "\t"
				<< points[i].data[2] << endl;
	}
	outfile << nindexes << endl;
	for (int i = 0; i < nindexes; i++) {
		outfile << indexes[i].data[0] << "\t" << indexes[i].data[1] << "\t"
				<< indexes[i].data[2] << endl;
	}
	outfile.close();
}

void OrXmlRobot::WriteMesh2Wrl() {
	ofstream cmdfile;
	string tmpfname(URDF_DIR);
	tmpfname.append("cmd.txt");
	cmdfile.open(tmpfname.c_str());
	for (size_t i = 0; i < xmlBodys.size(); i++) {
		OrXmlBody* body = xmlBodys[i];
		//render file
		if (!body->vis_filename.empty()) {
			vector<MyPoint3D> points;
			vector<TriFaceIndex> indexes;
			ifstream infile;
			infile.open(body->vis_filename.c_str());
			int npoints, nindexes;
			infile >> npoints;
			MyPoint3D pt;
			for (int i = 0; i < npoints; i++) {
				double p0, p1, p2;
				infile >> p0;
				infile >> p1;
				infile >> p2;
				pt.Set(p0, p1, p2);
				points.push_back(pt);
			}
			infile >> nindexes;
			TriFaceIndex ti;
			for (int i = 0; i < nindexes; i++) {
				int i0, i1, i2;
				infile >> i0;
				infile >> i1;
				infile >> i2;
				ti.Set(i0, i1, i2);
				indexes.push_back(ti);
			}
			infile.close();
			ofstream outfile;

			const char* filename;
			filename = GetFileName(body->vis_filename.c_str());
			string newfilename(URDF_DIR);
			newfilename.append(filename);
			char fnamechar[100];
			strcpy(fnamechar, newfilename.c_str());
			ChangeFileExtension(fnamechar, "wrl");
			newfilename.assign(fnamechar);

			const char* filename2;
			filename2 = GetFileName(newfilename.c_str());
			strcpy(fnamechar, filename2);
			StripExtension(fnamechar);
			cmdfile << "meshconv.exe -c stl " << filename2 << " -o stl/"
					<< fnamechar << endl;

			outfile.open(newfilename.c_str());
			outfile << "#VRML V2.0 utf8\n\n";
			outfile << "DEF " << body->name << "\n";
			outfile << "Transform { \n \t children [ \n \t\t Shape { \n";
			//appearance
			outfile << "\t\t\t appearance Appearance{ \n";
			outfile << "\t\t\t\t material Material { \n";
			outfile << "\t\t\t\t\t ambientIntensity 1.000000 \n";
			outfile << "\t\t\t\t\t diffuseColor 0.800000 0.800000 0.500000 \n";
			outfile << "\t\t\t\t\t specularColor 0.800000 0.850000 0.800000 \n";
			outfile << "\t\t\t\t\t shininess 0.300000 \n";
			outfile << "\t\t\t\t } \n";
			outfile << "\t\t\t } \n";

			//geometry
			outfile << "\t\t\t geometry IndexedFaceSet { \n";
			outfile << "\t\t\t\t coord Coordinate { \n";
			outfile << "\t\t\t\t\t point [ \n";
			for (size_t i = 0; i < points.size(); i++) {
				MyPoint3D p(points[i]);
				outfile << "\t\t\t\t\t" << p.data[0] << "\t" << p.data[1]
						<< "\t" << p.data[2] << ",\n";
			}
			outfile << "\t\t\t\t\t ] \n";
			outfile << "\t\t\t\t } \n";
			outfile << "\t\t\t\t coordIndex [ \n";
			for (size_t i = 0; i < indexes.size(); i++) {
				TriFaceIndex fi(indexes[i]);
				outfile << "\t\t\t\t\t";
				for (int j = 0; j < 3; j++) {
					outfile << fi.data[j] << "\t";
				}
				outfile << "-1,\n";
			}
			outfile << "\t\t\t\t ] \n";
			outfile
					<< "\t\t\t\t ccw TRUE \n	\t\t\t\t solid FALSE \n \t\t\t\t convex TRUE \n";
			outfile << "\t\t\t } \n";
			outfile << "\t\t } \n \t ] \n }";

			outfile.close();
		}
		//collision file
		if (!body->col_filename.empty()) {
			vector<MyPoint3D> points;
			vector<TriFaceIndex> indexes;
			ifstream infile;
			infile.open(body->col_filename.c_str());
			int npoints, nindexes;
			infile >> npoints;
			MyPoint3D pt;
			for (int i = 0; i < npoints; i++) {
				double p0, p1, p2;
				infile >> p0;
				infile >> p1;
				infile >> p2;
				pt.Set(p0, p1, p2);
				points.push_back(pt);
			}
			infile >> nindexes;
			TriFaceIndex ti;
			for (int i = 0; i < nindexes; i++) {
				int i0, i1, i2;
				infile >> i0;
				infile >> i1;
				infile >> i2;
				ti.Set(i0, i1, i2);
				indexes.push_back(ti);
			}
			infile.close();
			ofstream outfile;

			const char* filename;
			filename = GetFileName(body->col_filename.c_str());
			string newfilename(URDF_DIR);
			newfilename.append(filename);
			char fnamechar[100];
			strcpy(fnamechar, newfilename.c_str());
			ChangeFileExtension(fnamechar, "wrl");
			newfilename.assign(fnamechar);

			const char* filename2;
			filename2 = GetFileName(newfilename.c_str());
			strcpy(fnamechar, filename2);
			StripExtension(fnamechar);
			cmdfile << "meshconv.exe -c stl " << filename2 << " -o "
					<< fnamechar << endl;

			outfile.open(newfilename.c_str());
			outfile << "#VRML V2.0 utf8\n\n";
			outfile << "DEF " << body->name << "\n";
			outfile << "Transform { \n \t children [ \n \t\t Shape { \n";
			//appearance
			outfile << "\t\t\t appearance Appearance{ \n";
			outfile << "\t\t\t\t material Material { \n";
			outfile << "\t\t\t\t\t ambientIntensity 1.000000 \n";
			outfile << "\t\t\t\t\t diffuseColor 0.800000 0.800000 0.500000 \n";
			outfile << "\t\t\t\t\t specularColor 0.800000 0.850000 0.800000 \n";
			outfile << "\t\t\t\t\t shininess 0.300000 \n";
			outfile << "\t\t\t\t } \n";
			outfile << "\t\t\t } \n";

			//geometry
			outfile << "\t\t\t geometry IndexedFaceSet { \n";
			outfile << "\t\t\t\t coord Coordinate { \n";
			outfile << "\t\t\t\t\t point [ \n";
			for (int i = 0; i < points.size(); i++) {
				MyPoint3D p(points[i]);
				outfile << "\t\t\t\t\t" << p.data[0] << "\t" << p.data[1]
						<< "\t" << p.data[2] << ",\n";
			}
			outfile << "\t\t\t\t\t ] \n";
			outfile << "\t\t\t\t } \n";
			outfile << "\t\t\t\t coordIndex [ \n";
			for (int i = 0; i < indexes.size(); i++) {
				TriFaceIndex fi(indexes[i]);
				outfile << "\t\t\t\t\t";
				for (int j = 0; j < 3; j++) {
					outfile << fi.data[j] << "\t";
				}
				outfile << "-1,\n";
			}
			outfile << "\t\t\t\t ] \n";
			outfile
					<< "\t\t\t\t ccw TRUE \n	\t\t\t\t solid FALSE \n \t\t\t\t convex TRUE \n";
			outfile << "\t\t\t } \n";
			outfile << "\t\t } \n \t ] \n }";

			outfile.close();
		}
	}
	cmdfile.close();
}

bool OrXmlRobot::GetRobot(Robot& robot) {
	GetAllCleanBodyJoints();

	if (children[0].size() <= 1) {
		cout << "Fixed Base robot\n" << flush;
		if (!GetFixedBaseRobot(robot)) {
			return false;
		}
	} else {
		cout << "Floating Base robot\n" << flush;
		if (!GetFloatingBaseRobot(robot)) {
			return false;
		}
	}

	return true;
}

bool OrXmlRobot::ConvertToURDF() {
	GetAllCleanBodyJoints();
	this->WriteMesh2Wrl();

	this->getDiffJointNames();

	this->recomputeTworld();

	this->writeToURDF();
	return true;
}

void OrXmlRobot::recomputeTworld() {
	vector<int> needprocess;
	needprocess.push_back(linkroot);
	while (needprocess.size() > 0) {
		int bodyindex = needprocess.back();
		needprocess.pop_back();

		OrXmlBody* body = xmlBodys[bodyindex];

		if (bodyindex != linkroot) {
			OrXmlBody* parentbody = xmlBodys[parents[bodyindex]];
			body->Tworld->mul(*parentbody->Tworld, *body->Tparent);
		}
//		cout<<body->name<<":\n"<<"Tparent:"<<*body->Tparent<<"\nTworld:"<<*body->Tworld<<endl<<flush;getchar();

		for (size_t i = 0; i < children[bodyindex].size(); i++) {
			needprocess.push_back(children[bodyindex][i]);
		}
	}
}

void OrXmlRobot::getDiffJointNames() {
	for (size_t i = 0; i < xmlJoints.size(); i++) {
		string strname = xmlJoints[i]->name;
		for (size_t j = 0; j < xmlJoints.size(); j++) {
			if (i == j)
				continue;
			if (0 == strcmp(strname.c_str(), xmlJoints[j]->name.c_str())) {
				stringstream ss;
				ss << i;
				string tmp;
				ss >> tmp;
				xmlJoints[i]->name.append(tmp);
				break;
			}
		}
	}
}

void OrXmlRobot::writeToURDF() {
	string filename = name;
	filename.append(".xacro");
	ofstream file;
	file.open(filename.c_str());
	file << "<?xml version=\"1.0\"?>" << endl;
	file << "<robot name=\"" << name
			<< "\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">" << endl;
	file
			<< "<xacro:property name=\"path\" value=\"file:///home/jingru/fuerte_workspace/sandbox/learning_urdf/dae/\" />"
			<< endl;
	file << "<xacro:property name=\"effort\" value=\"10.0\" />" << endl;
	file << "<xacro:property name=\"lower\" value=\"-3.14\" />" << endl;
	file << "<xacro:property name=\"upper\" value=\"3.14\" />" << endl;
	file << "<xacro:property name=\"velocity\" value=\"1.0\" />" << endl;
	//links
	size_t nLink = xmlBodys.size();
	for (size_t i = 0; i < nLink; i++) {
		OrXmlBody* body = xmlBodys[i];
		file << "<link name=\"" << body->name << "\">" << endl;

		if (body->mass
				&& (body->mass->total || body->mass->inertia || body->mass->com)) {
			file << "<inertial>" << endl;
			if (body->mass->total)
				file << "<mass value=\"" << *body->mass->total << "\" />"
						<< endl;
			if (body->mass->inertia) {
				Matrix3 I(*body->mass->inertia);
				file << "<inertia ixx=\"" << I(0, 0) << "\" ixy=\"" << I(0, 1)
						<< "\" ixz=\"" << I(0, 2) << "\" iyy=\"" << I(1, 1)
						<< "\" iyz=\"" << I(1, 2) << "\" izz=\"" << I(2, 2)
						<< "\" />" << endl;
			} else {
				file
						<< "<inertia ixx=\"1\" ixy=\"0\" ixz=\"0\" iyy=\"1\" iyz=\"0\" izz=\"1\" />"
						<< endl;
			}
			if (body->mass->com)
				file << "<origin xyz=\"" << body->mass->com->x << " "
						<< body->mass->com->y << " " << body->mass->com->z
						<< "\" />" << endl;
			file << "</inertial>" << endl;
		}

		Vector3 xyz, rpy;
		rotationMat2RPY(body->Tworld->R, rpy);
		xyz.set(body->Tworld->t);

		if (!body->vis_filename.empty()) {
			file << "<visual>" << endl;
//			file <<"<origin xyz=\""<<xyz.x<<" "<<xyz.y<<" "<<xyz.z<<"\" rpy=\""<<rpy.x<<" "<<rpy.y<<" "<<rpy.z<<"\" />"<<endl;
			file << "<geometry>" << endl;
			const char* filename;
			filename = GetFileName(body->vis_filename.c_str());
			string newfilename(filename);
			char fnamechar[100];
			strcpy(fnamechar, newfilename.c_str());
			ChangeFileExtension(fnamechar, "dae");
			newfilename.assign(fnamechar);
			file << "<mesh filename=\"${path}" << newfilename << "\"/>" << endl;
			file << "</geometry>" << endl;
			file << "<material name=\"random\">" << endl;
			file << "<color rgba=\"0.800000 0.800000 0.500000 1\"/>" << endl;
			file << "</material>" << endl;
			file << "</visual>" << endl;
		}
		if(body->col_filename.empty() && !body->vis_filename.empty()){
			body->col_filename = body->vis_filename;
		}
		if (!body->col_filename.empty()) {
			const char* filename;
			filename = GetFileName(body->col_filename.c_str());
			string newfilename(filename);
			char fnamechar[100];
			strcpy(fnamechar, newfilename.c_str());
			ChangeFileExtension(fnamechar, "dae");
			newfilename.assign(fnamechar);
			file << "<collision>" << endl;
//			file <<"<origin xyz=\""<<xyz.x<<" "<<xyz.y<<" "<<xyz.z<<"\" rpy=\""<<rpy.x<<" "<<rpy.y<<" "<<rpy.z<<"\" />"<<endl;
			file << "<geometry>" << endl;
			file << "<mesh filename=\"${path}" << newfilename << "\"/>" << endl;
			file << "</geometry>" << endl;
			file << "</collision>" << endl;
		}

		file << "</link>" << endl;
	}
	//joints
	size_t nJoint = this->xmlJoints.size();
	for (size_t i = 0; i < nJoint; i++) {
		OrXmlJoint* joint = xmlJoints[i];
		string typestr;
		if (!joint->enabled)
			typestr = "fixed";
		else if (0 == strcmp(joint->type.c_str(), "hinge")) {
			typestr = "revolute";
		}
		file << "<joint name=\"" << joint->name << "\" type=\"" << typestr
				<< "\">" << endl;

		int b1 = getLinkIndex(joint->bodys[0]);
		int b2 = getLinkIndex(joint->bodys[1]);
		assert(b1 != -1);
		assert(b2 != -1);
		string parentstr, childstr;
		int parentint, childint;
		if (parents[b1] == b2) {
			parentstr = joint->bodys[1];
			childstr = joint->bodys[0];
			parentint = b2;
			childint = b1;
		} else if (parents[b2] == b1) {
			parentstr = joint->bodys[0];
			childstr = joint->bodys[1];
			parentint = b1;
			childint = b2;
		} else {
			cout << "Invalid joint " << xmlJoints[i]->name << ";"
					<< xmlJoints[i]->bodys[0] << ";" << xmlJoints[i]->bodys[1]
					<< endl;
			getchar();
			return;
		}
		Vector3 xyz, rpy;
		rotationMat2RPY(xmlBodys[childint]->Tparent->R, rpy);
		xyz.set(xmlBodys[childint]->Tparent->t);

		file << "<origin xyz=\"" << xyz.x << " " << xyz.y << " " << xyz.z
				<< "\" rpy=\"" << rpy.x << " " << rpy.y << " " << rpy.z
				<< "\" />" << endl;
		file << "<parent link=\"" << parentstr << "\" />" << endl;
		file << "<child link=\"" << childstr << "\" />" << endl;

		if (joint->axis)
			file << "<axis xyz=\"" << joint->axis->x << " " << joint->axis->y
					<< " " << joint->axis->z << "\" />" << endl;
		file << "<limit ";
		if (joint->cleanMaxTorque.size() == 1)
			file << "effort=\"" << joint->cleanMaxTorque[0] << "\" ";
		else
			file << "effort=\"${effort}\" ";
		if (joint->cleanLimits.size() == 2)
			file << "lower=\"" << joint->cleanLimits[0] << "\" upper=\""
					<< joint->cleanLimits[1] << "\" ";
		else
			file << "lower=\"${lower}\" upper=\"${upper}\" ";
		if (joint->cleanMaxVel.size() == 1)
			file << "velocity=\"" << joint->cleanMaxVel[0] << "\" ";
		else
			file << "velocity=\"${velocity}\" ";
		file << "/>" << endl;
		file << "</joint>" << endl;
	}

	file << "</robot>" << endl;
	file.close();
}

void OrXmlRobot::WriteHubo(Robot& robot, bool isjaemi) {
	string filename;
	if (LOADSimpleGEOM){
		if(isjaemi)
			filename = ROBOT_DIR + "jaemihubo_col.rob";
		else
			filename = ROBOT_DIR + "HuboPlus_col.rob";
	}else{
		if(isjaemi)
			filename = ROBOT_DIR + "jaemihubo_vis.rob";
		else
			filename = ROBOT_DIR + "HuboPlus_vis.rob";
	}

	ofstream file;
	file.open(filename.c_str());
	if (!file.is_open()) {
		cout << filename << " cannot be opened!\n" << flush;
		getchar();
		file.close();
		return;
	}
	cout<<"write "<<filename<<endl<<flush;
	size_t nLinks = robot.links.size();
	assert(linkgroups.back().back()+1 == nLinks);

	file << "links\t";
	for (size_t i = 0; i < linkgroups.size(); i++) {
		for (size_t j = 0; j < linkgroups[i].size(); j++)
			file << "\t\"" << robot.linkNames[linkgroups[i][j]] << "\" ";
		if (i < linkgroups.size() - 1)
			file << "\\" << endl;
	}
	file << endl << endl;

	file << "parents\t";
	for (size_t i = 0; i < robot.parents.size(); i++) {
		file << robot.parents[i] << " ";
	}
	file << endl << endl;

	file << "axis\t";
	for (size_t i = 0; i < nLinks; i++)
		file << robot.links[i].w[0] << " " << robot.links[i].w[1] << " "
				<< robot.links[i].w[2] << "\t";
	file << endl << endl;

	file << "jointtype\t";
	for (size_t i = 0; i < nLinks; i++) {
		if (robot.links[i].type == RobotLink3D::Prismatic)
			file << "p ";
		else if (robot.links[i].type == RobotLink3D::Revolute)
			file << "r ";
	}
	file << endl << endl;

	file << "Tparent\t";
	for (size_t i = 0; i < nLinks; i++) {
		RigidTransform T(robot.links[i].T0_Parent);
		file << T.R(0, 0) << " " << T.R(0, 1) << " " << T.R(0, 2) << "\t";
		file << T.R(1, 0) << " " << T.R(1, 1) << " " << T.R(1, 2) << "\t";
		file << T.R(2, 0) << " " << T.R(2, 1) << " " << T.R(2, 2) << "\t";
		file << T.t[0] << " " << T.t[1] << " " << T.t[2];
		if (i < nLinks - 1)
			file << " \\" << endl;
	}
	file << endl << endl;

	file << "qmin\t";
	for (size_t i = 0; i < linkgroups.size(); i++) {
		for (size_t j = 0; j < linkgroups[i].size(); j++)
			file << robot.qMin[linkgroups[i][j]] << " ";
		if (i < linkgroups.size() - 1)
			file << "\\" << endl << "\t";
	}
	file << endl << endl;

	file << "qmax\t";
	for (size_t i = 0; i < linkgroups.size(); i++) {
		for (size_t j = 0; j < linkgroups[i].size(); j++)
			file << robot.qMax[linkgroups[i][j]] << " ";
		if (i < linkgroups.size() - 1)
			file << "\\" << endl << "\t";
	}
	file << endl << endl;

	file << "geometry\t";
	for (size_t i = 0; i < nLinks; i++) {
		if (i < 5)
			file << "\"\" ";
		else {
			if (false == LOADSimpleGEOM) {
				if (xmlBodys[i - 5]->vis_filename.empty())
					file << "\"\" ";
				else
					file << "\"" << xmlBodys[i - 5]->vis_filename << "\" ";
			} else {
				if (xmlBodys[i - 5]->col_filename.empty()) {
					if (xmlBodys[i - 5]->vis_filename.empty())
						file << "\"\" ";
					else
						file << "\"" << xmlBodys[i - 5]->vis_filename << "\" ";
				} else
					file << "\"" << xmlBodys[i - 5]->col_filename << "\" ";
			}
		}
	}
	file << endl << endl;

	file << "mass\t";
	for (size_t i = 0; i < nLinks; i++) {
		file << robot.links[i].mass << " ";
	}
	file << endl << endl;

	file << "com\t";
	for (size_t i = 0; i < linkgroups.size(); i++) {
		for (size_t j = 0; j < linkgroups[i].size(); j++){
			file << robot.links[linkgroups[i][j]].com[0] << " " << robot.links[linkgroups[i][j]].com[1] << " "<< robot.links[linkgroups[i][j]].com[2] << "\t";
		}
		if (i < linkgroups.size() - 1)
			file << "\\" << endl << "\t";
	}
	file << endl << endl;

	file << "inertia\t";
	for (size_t i = 0; i < linkgroups.size(); i++) {
		for (size_t j = 0; j < linkgroups[i].size(); j++){
			Matrix3 inertia(robot.links[linkgroups[i][j]].inertia);
			file << inertia(0, 0) << " " << inertia(0, 1) << " " << inertia(0, 2)
					<< " ";
			file << inertia(1, 0) << " " << inertia(1, 1) << " " << inertia(1, 2)
					<< " ";
			file << inertia(2, 0) << " " << inertia(2, 1) << " " << inertia(2, 2)
					<< "\t";
		}
		if (i < linkgroups.size() - 1)
			file << "\\" << endl << "\t";
	}
//	for (size_t i = 0; i < nLinks; i++) {
//		Matrix3 inertia(robot.links[i].inertia);
//		file << inertia(0, 0) << " " << inertia(0, 1) << " " << inertia(0, 2)
//				<< " ";
//		file << inertia(1, 0) << " " << inertia(1, 1) << " " << inertia(1, 2)
//				<< " ";
//		file << inertia(2, 0) << " " << inertia(2, 1) << " " << inertia(2, 2)
//				<< "\t";
//	}
	file << endl << endl;

	file << "torquemax\t";
	for (size_t i = 0; i < linkgroups.size(); i++) {
		for (size_t j = 0; j < linkgroups[i].size(); j++)
			file << robot.torqueMax[linkgroups[i][j]] << " ";
		if (i < linkgroups.size() - 1)
			file << "\\" << endl << "\t";
	}
	file << endl << endl;

	file << "velmax\t";
	for (size_t i = 0; i < linkgroups.size(); i++) {
		for (size_t j = 0; j < linkgroups[i].size(); j++)
			file << robot.velMax[linkgroups[i][j]] << " ";
		if (i < linkgroups.size() - 1)
			file << "\\" << endl << "\t";
	}
	file << endl << endl;

	file << "accmax\t";
	for (size_t i = 0; i < linkgroups.size(); i++) {
		for (size_t j = 0; j < linkgroups[i].size(); j++)
			file << robot.accMax[linkgroups[i][j]] << " ";
		if (i < linkgroups.size() - 1)
			file << "\\" << endl << "\t";
	}
	file << endl << endl;

	size_t nJoints = robot.joints.size();
	for (size_t i = 0; i < nJoints; i++) {
		file << "joint ";
		if (robot.joints[i].type == RobotModelJoint::Floating)
			file << "floating " << robot.joints[i].linkIndex << " "
					<< robot.joints[i].baseIndex << endl;
		else if (robot.joints[i].type == RobotModelJoint::Normal)
			file << "normal " << robot.joints[i].linkIndex << endl;
		else if (robot.joints[i].type == RobotModelJoint::Weld)
			file << "weld " << robot.joints[i].linkIndex << endl;

	}
	file << endl << endl;

	size_t nDrivers = robot.drivers.size();
	for (size_t i = 0; i < nDrivers; i++) {
		if (robot.drivers[i].type == RobotModelDriver::Normal) {
			file << "driver normal " << robot.drivers[i].linkIndices[0] << endl;
		}
	}
	file << endl << endl;

	file << "servoP\t";
	for (size_t i = 0; i < drivergroups.size(); i++) {
		for (size_t j = 0; j < drivergroups[i].size(); j++)
			if (robot.drivers[drivergroups[i][j]].type
					== RobotModelDriver::Normal) {
				file << robot.drivers[drivergroups[i][j]].servoP << " ";
			}
		if (i < drivergroups.size() - 1)
			file << "\\" << endl << "\t";
	}
	file << endl;
	file << "servoI\t";
	for (size_t i = 0; i < drivergroups.size(); i++) {
		for (size_t j = 0; j < drivergroups[i].size(); j++)
			if (robot.drivers[drivergroups[i][j]].type
					== RobotModelDriver::Normal) {
				file << robot.drivers[drivergroups[i][j]].servoI << " ";
			}
		if (i < drivergroups.size() - 1)
			file << "\\" << endl << "\t";
	}
	file << endl;
	file << "servoD\t";
	for (size_t i = 0; i < drivergroups.size(); i++) {
		for (size_t j = 0; j < drivergroups[i].size(); j++)
			if (robot.drivers[drivergroups[i][j]].type
					== RobotModelDriver::Normal) {
				file << robot.drivers[drivergroups[i][j]].servoD << " ";
			}
		if (i < drivergroups.size() - 1)
			file << "\\" << endl << "\t";
	}
	file << endl << endl;

	file << "dryFriction\t";
	for (size_t i = 0; i < drivergroups.size(); i++) {
		for (size_t j = 0; j < drivergroups[i].size(); j++)
			if (robot.drivers[drivergroups[i][j]].type
					== RobotModelDriver::Normal) {
				file << robot.drivers[drivergroups[i][j]].dryFriction << " ";
			}
		if (i < drivergroups.size() - 1)
			file << "\\" << endl << "\t";
	}
	file << endl;


	file.close();
}

void OrXmlRobot::Write2Rob(Robot& robot) {
	string filename;
	if (LOADSimpleGEOM)
		filename = ROBOT_DIR + name + "_col.rob";
	else
		filename = ROBOT_DIR + name + "_vis.rob";

	cout << "Writing .rob file to " << filename << endl;
	vector<string> geomFiles(robot.links.size());
	if(robot.joints[0].type == RobotModelJoint::Floating){
		cout<<"write floating base robot"<<endl;
		for (size_t i = 0; i < robot.links.size(); i++) {
			if (i < 5)
				continue;
			if (false == LOADSimpleGEOM) {
				geomFiles[i] = xmlBodys[i - 5]->vis_filename;
			} else {
				if (xmlBodys[i - 5]->col_filename.empty()) {
					geomFiles[i] = xmlBodys[i - 5]->vis_filename;
				} else
					geomFiles[i] = xmlBodys[i - 5]->col_filename;
			}
		}
	}else{
		cout<<"write fixed base robot"<<endl;
		for(size_t i = 0; i < robot.links.size(); i++){
			if(false == LOADSimpleGEOM)
				geomFiles[i] = xmlBodys[i]->vis_filename;
			else
				geomFiles[i] = xmlBodys[i]->col_filename;
		}
	}

	robot.SetGeomFiles(geomFiles);
	bool res = robot.Save(filename.c_str());
	assert(res==true);
}

bool OrXmlRobot::GetFloatingBaseRobot(Robot& robot) {
	size_t nBody = xmlBodys.size();

	assert(parents.size() == nBody);
	assert(children.size() == nBody);
	assert(parents[0] == -1);
	assert(children[0].size() >= 1);

	robot.parents.resize(nBody + 5);
	robot.linkNames.resize(nBody + 5);
	robot.links.resize(nBody + 5);
	robot.geometry.resize(nBody + 5);

	robot.q.resize(nBody + 5);
	robot.dq.resize(nBody + 5);
	robot.qMin.resize(nBody + 5);
	robot.qMax.resize(nBody + 5);
	robot.accMax.resize(nBody + 5);
	robot.velMin.resize(nBody + 5);
	robot.velMax.resize(nBody + 5);
	robot.torqueMax.resize(nBody + 5);
	robot.powerMax.resize(nBody + 5);

	//base: trunk
	for (int i = 0; i < 6; i++) {
		robot.parents[i] = i - 1;
		stringstream ss;
		ss << i;
		string tmp;
		ss >> tmp;
		robot.linkNames[i] = xmlBodys[0]->name + tmp;
		robot.q[i] = 0;
		robot.dq[i] = 0;
		robot.accMax[i] = Inf;
		robot.qMax[i] = Inf;
		robot.qMin[i] = -Inf;
		robot.velMax[i] = Inf;
		robot.velMin[i] = -Inf;
		robot.torqueMax[i] = 0;
		robot.powerMax[i] = Inf;
	}
	for (int i = 0; i < 3; i++)
		robot.links[i].type = RobotLink3D::Prismatic;
	for (int i = 3; i < 6; i++)
		robot.links[i].type = RobotLink3D::Revolute;
	robot.links[0].w.set(1, 0, 0);
	robot.links[1].w.set(0, 1, 0);
	robot.links[2].w.set(0, 0, 1);
	robot.links[3].w.set(0, 0, 1);
	robot.links[4].w.set(0, 1, 0);
	robot.links[5].w.set(1, 0, 0);
	for (int i = 0; i < 5; i++) {
		robot.links[i].com.setZero();
		robot.links[i].mass = 0;
		robot.links[i].inertia.setZero();
		robot.links[i].T0_Parent.setIdentity();
	}
	if (xmlBodys[0]->mass && xmlBodys[0]->mass->com)
		robot.links[5].com.set(*(xmlBodys[0]->mass->com));
	else
		robot.links[5].com.setZero();
	if (xmlBodys[0]->mass && xmlBodys[0]->mass->total)
		robot.links[5].mass = (*(xmlBodys[0]->mass->total));
	else
		robot.links[5].mass = defaultmass;
	if (xmlBodys[0]->mass && xmlBodys[0]->mass->inertia)
		robot.links[5].inertia.set(*(xmlBodys[0]->mass->inertia));
	else
		robot.links[5].inertia.setIdentity();

	robot.links[0].T0_Parent.set(*xmlBodys[0]->Tparent);

	robot.links[5].T0_Parent.setIdentity();
	if (LOADSimpleGEOM && !(xmlBodys[0]->col_filename.empty()))
		robot.LoadGeometry(5, xmlBodys[0]->col_filename.c_str());
	else
		robot.LoadGeometry(5, xmlBodys[0]->vis_filename.c_str());

	//other links
	for (size_t i = 1; i < nBody; i++) {
		robot.parents[i + 5] = parents[i] + 5;
		if (LOADSimpleGEOM && !(xmlBodys[i]->col_filename.empty()))
			robot.LoadGeometry(i + 5, xmlBodys[i]->col_filename.c_str());
		else if (!(xmlBodys[i]->vis_filename.empty())) {
			robot.LoadGeometry(i + 5, xmlBodys[i]->vis_filename.c_str());
		}
		robot.linkNames[i + 5] = xmlBodys[i]->name;

		robot.links[i + 5].type = RobotLink3D::Revolute;
		robot.links[i + 5].T0_Parent.set(*xmlBodys[i]->Tparent);
		robot.links[i + 5].w.set(0, 0, 1);

		if(!xmlBodys[i]->mass && xmlBodys[i]->xmlGeoms.size() > 0){
			OrXmlMass* ormass = new OrXmlMass();
			ormass->GetUnitSphereMass();
			robot.links[i+5].com.set(*ormass->com);
			robot.links[i+5].inertia.set(*ormass->inertia);
			robot.links[i+5].mass = (*ormass->total);
			delete ormass;
		}else{
			if (xmlBodys[i]->mass && xmlBodys[i]->mass->com)
				robot.links[i + 5].com.set(*xmlBodys[i]->mass->com);
			else
				robot.links[i + 5].com.setZero();

			if (xmlBodys[i]->mass && xmlBodys[i]->mass->inertia)
				robot.links[i + 5].inertia.set(*xmlBodys[i]->mass->inertia);
			else{
				robot.links[i + 5].inertia.setZero();
//				cout<<robot.linkNames[i+5]<<";"<<robot.links[i+5].inertia<<endl;
			}

			if (xmlBodys[i]->mass && xmlBodys[i]->mass->total)
				robot.links[i + 5].mass = *xmlBodys[i]->mass->total;
			else
				robot.links[i+5].mass = 0;
		}
		robot.q[i + 5] = 0;
		robot.dq[i + 5] = 0;
		robot.accMax[i + 5] = Inf;
		robot.qMax[i + 5] = 0;
		robot.qMin[i + 5] = 0;
		robot.velMax[i + 5] = Inf;
		robot.velMin[i + 5] = -Inf;
		robot.torqueMax[i + 5] = Inf;
		robot.powerMax[i + 5] = Inf;
	}

	robot.joints.resize(1);
	robot.joints[0].type = RobotModelJoint::Floating;
	robot.joints[0].linkIndex = 5;
	robot.joints[0].baseIndex = -1;

	for (size_t i = 0; i < xmlJoints.size(); i++) {
		RobotModelJoint robjoint;
		robjoint.type = RobotModelJoint::Normal;
		if (xmlJoints[i]->enabled == false) {
			robjoint.type = RobotModelJoint::Weld;
		}

		int linkI = xmlJoints[i]->linkI;
		robjoint.linkIndex = linkI;
		if(robjoint.type != RobotModelJoint::Weld){
			if(robot.links[linkI].mass == 0){
				robot.links[linkI].mass = 0.1;
			}
			if(robot.links[linkI].inertia.isZero()){
//				robot.links[linkI].inertia.setIdentity();
				robot.links[linkI].inertia(0,0) = 1e-9;
				robot.links[linkI].inertia(1,1) = 1e-9;
				robot.links[linkI].inertia(2,2) = 1e-9;
			}
		}
		if (xmlJoints[i]->axis) {
			robot.links[linkI].w.set(*xmlJoints[i]->axis);
		}

		if (xmlJoints[i]->cleanLimits.size() == 2) {
			if (xmlJoints[i]->cleanLimits[0] == xmlJoints[i]->cleanLimits[1])
				robjoint.type = RobotModelJoint::Weld;
			else {
				robot.qMin[linkI] = xmlJoints[i]->cleanLimits[0];
				robot.qMax[linkI] = xmlJoints[i]->cleanLimits[1];
			}
		} else {
			robot.qMin[linkI] = -Inf;
			robot.qMax[linkI] = Inf;
		}
		if (xmlJoints[i]->cleanMaxVel.size() == 1) {
			robot.velMin[linkI] = -(xmlJoints[i]->cleanMaxVel[0]);
			robot.velMax[linkI] = xmlJoints[i]->cleanMaxVel[0];
		} else {
			robot.velMin[linkI] = -Inf;
			robot.velMax[linkI] = Inf;
		}
		if (xmlJoints[i]->cleanMaxAcc.size() == 1) {
			robot.accMax[linkI] = xmlJoints[i]->cleanMaxAcc[0];
		} else {
			robot.accMax[linkI] = -Inf;
		}
		if (xmlJoints[i]->cleanMaxTorque.size() == 1) {
			robot.torqueMax[linkI] = xmlJoints[i]->cleanMaxTorque[0];
		} else {
			robot.torqueMax[linkI] = Inf;
		}
		robot.joints.push_back(robjoint);
	}

	size_t nJoint = robot.joints.size();
	robot.driverNames.resize(0);
	robot.drivers.resize(0);
	for (size_t i = 0; i < nJoint; i++) {
		if (robot.joints[i].type == RobotModelJoint::Normal) {
			int linkI = robot.joints[i].linkIndex;
			robot.driverNames.push_back(robot.linkNames[linkI]);
			RobotModelDriver d;
			d.type = RobotModelDriver::Normal;
			d.linkIndices.push_back(linkI);
			d.qmin = robot.qMin(linkI);
			d.qmax = robot.qMax(linkI);
			d.vmin = robot.velMin(linkI);
			d.vmax = robot.velMax(linkI);
			d.tmin = -robot.torqueMax(linkI);
			d.tmax = robot.torqueMax(linkI);
			d.amin = -robot.accMax(linkI);
			d.amax = robot.accMax(linkI);
			d.servoP = 100;
			d.servoI = 0;
			d.servoD = 1;
			d.dryFriction = 0;
			robot.drivers.push_back(d);
		}
	}

//	if (0 == strcmp(name.c_str(), "jaemiHubo")) {
//		this->setJaemihuboProperty(robot);
//		this->WriteHubo(robot, true);
//	} else if(0 == strcmp(name.c_str(), "huboplus") || 0 == strcmp(name.c_str(), "huboPlus")){
//		this->setHuboplusProperty(robot);
//		this->WriteHubo(robot, false);
//	}
	robot.selfCollisions.resize(nBody + 5, nBody + 5, NULL);
	robot.envCollisions.resize(nBody + 5, NULL);

	robot.InitCollisions();
	robot.InitAllSelfCollisions();

//	robot.UpdateFrames();
//	ofstream outfile;
//	outfile.open("RobotSim_GlobalCOM.txt");
//	for(size_t i = 0; i < robot.links.size(); i++){
//		Vector3 v;
//		robot.links[i].T_World.mul(robot.links[i].com, v);
//		outfile <<"link "<<i<<" "<<robot.linkNames[i]<<": "<<v<<endl;
////		outfile <<"link "<<i<<" "<<robot.linkNames[i]<<": "<<robot.links[i].com<<endl;
//	}
//	outfile.close();

//	robot.UpdateFrames();
//	ofstream outfile;
//	outfile.open("RobotSim_GlobalTransform.txt");
//	for(size_t i = 0; i < robot.links.size(); i++){
//		outfile <<"link "<<i<<" "<<robot.linkNames[i]<<":\n"<<robot.links[i].T_World<<endl;
//	}
//	outfile.close();

//	for(size_t i = 0; i < robot.links.size(); i++){
//		if(0==strcmp(robot.linkNames[i].c_str(), "Body_LSY")||0==strcmp(robot.linkNames[i].c_str(), "Body_RSY")){
//			cout<<robot.linkNames[i]<<":"<<robot.links[i].inertia<<endl;
//		}
//	}
	return true;
}

void OrXmlRobot::setJaemihuboProperty(Robot& robot) {
	if (robot.qMin.size() == 71 && robot.drivers.size() == 59) {
		initJaemihuboGroups();
//		cout << "set qmin and qmax for fingers" << flush;
		int rthumbGroup = 4;
//		cout << linkgroups[rthumbGroup].size() << endl << flush;
		assert(linkgroups[rthumbGroup].size() == 3);
		robot.qMin[linkgroups[rthumbGroup][0]] = -2;
		robot.qMin[linkgroups[rthumbGroup][1]] = -1;
		robot.qMin[linkgroups[rthumbGroup][2]] = -1;
		robot.qMax[linkgroups[rthumbGroup][0]] = 0;
		robot.qMax[linkgroups[rthumbGroup][1]] = 0;
		robot.qMax[linkgroups[rthumbGroup][2]] = 0;
		int lthumbGroup = 12;
		assert(linkgroups[lthumbGroup].size() == 3);
		robot.qMin[linkgroups[lthumbGroup][0]] = -2;
		robot.qMin[linkgroups[lthumbGroup][1]] = -1;
		robot.qMin[linkgroups[lthumbGroup][2]] = -1;
		robot.qMax[linkgroups[lthumbGroup][0]] = 0;
		robot.qMax[linkgroups[lthumbGroup][1]] = 0;
		robot.qMax[linkgroups[lthumbGroup][2]] = 0;
		for (int i = rthumbGroup + 1; i < rthumbGroup + 5; i++) {
			robot.qMin[linkgroups[i][0]] = 0;
			robot.qMin[linkgroups[i][1]] = 0;
			robot.qMin[linkgroups[i][2]] = 0;
			robot.qMax[linkgroups[i][0]] = 2;
			robot.qMax[linkgroups[i][1]] = 1;
			robot.qMax[linkgroups[i][2]] = 1;
		}

		for (int i = lthumbGroup + 1; i < lthumbGroup + 5; i++) {
			robot.qMin[linkgroups[i][0]] = 0;
			robot.qMin[linkgroups[i][1]] = 0;
			robot.qMin[linkgroups[i][2]] = 0;
			robot.qMax[linkgroups[i][0]] = 2;
			robot.qMax[linkgroups[i][1]] = 1;
			robot.qMax[linkgroups[i][2]] = 1;
		}

//		cout << "set torques, velmax, accmax" << flush;
		for (size_t i = 0; i < linkgroups.size(); i++) {
			for (size_t j = 0; j < linkgroups[i].size(); j++) {
				robot.torqueMax[linkgroups[i][j]] = defaultTorqueMax[i];
				robot.velMax[linkgroups[i][j]] = defaultVelMax[i];
				robot.accMax[linkgroups[i][j]] = defaultAccMax[i];
			}
		}
		assert(linkgroups[3].size() == 3);
		robot.torqueMax[linkgroups[3][2]] = Math::Inf;
		assert(linkgroups[11].size() == 3);
		robot.torqueMax[linkgroups[11][2]] = Math::Inf;
		assert(linkgroups[18].size() == 2);
		robot.torqueMax[linkgroups[18][1]] = Math::Inf;
		robot.velMax[linkgroups[3][2]] = 0;
		robot.velMax[linkgroups[11][2]] = 0;
		robot.velMax[linkgroups[18][1]] = 0;
		robot.accMax[linkgroups[3][2]] = 0;
		robot.accMax[linkgroups[11][2]] = 0;
		robot.accMax[linkgroups[18][1]] = 0;

//		cout << " set servo PID dryFriction\n" << flush;
		for (size_t i = 0; i < drivergroups.size(); i++) {
			for (size_t j = 0; j < drivergroups[i].size(); j++) {
				robot.drivers[drivergroups[i][j]].servoP = defaultSP[i];
				robot.drivers[drivergroups[i][j]].servoI = defaultSI[i];
				robot.drivers[drivergroups[i][j]].servoD = defaultSD[i];
				robot.drivers[drivergroups[i][j]].dryFriction = defaultDF[i];
			}
		}
		robot.drivers[drivergroups[16][4]].servoP = 50;
		robot.drivers[drivergroups[16][5]].servoP = 50;
		robot.drivers[drivergroups[17][4]].servoP = 50;
		robot.drivers[drivergroups[17][5]].servoP = 50;
		robot.drivers[drivergroups[16][4]].servoI = 20;
		robot.drivers[drivergroups[16][5]].servoI = 20;
		robot.drivers[drivergroups[17][4]].servoI = 20;
		robot.drivers[drivergroups[17][5]].servoI = 20;
		robot.drivers[drivergroups[16][4]].servoD = 4;
		robot.drivers[drivergroups[16][5]].servoD = 4;
		robot.drivers[drivergroups[17][4]].servoD = 4;
		robot.drivers[drivergroups[17][5]].servoD = 4;
	}
}

void OrXmlRobot::setHuboplusProperty(Robot& robot) {
	if (robot.qMin.size() == 63 && robot.drivers.size() == 57) {
		initHuboplusGroups();
//		cout << "set qmin and qmax for fingers" << flush;
		int rthumbGroup = 3;
//		cout << linkgroups[rthumbGroup].size() << endl << flush;
		assert(linkgroups[rthumbGroup].size() == 3);
		robot.qMin[linkgroups[rthumbGroup][0]] = -2;
		robot.qMin[linkgroups[rthumbGroup][1]] = -1;
		robot.qMin[linkgroups[rthumbGroup][2]] = -1;
		robot.qMax[linkgroups[rthumbGroup][0]] = 0;
		robot.qMax[linkgroups[rthumbGroup][1]] = 0;
		robot.qMax[linkgroups[rthumbGroup][2]] = 0;
		int lthumbGroup = 9;
		assert(linkgroups[lthumbGroup].size() == 3);
		robot.qMin[linkgroups[lthumbGroup][0]] = -2;
		robot.qMin[linkgroups[lthumbGroup][1]] = -1;
		robot.qMin[linkgroups[lthumbGroup][2]] = -1;
		robot.qMax[linkgroups[lthumbGroup][0]] = 0;
		robot.qMax[linkgroups[lthumbGroup][1]] = 0;
		robot.qMax[linkgroups[lthumbGroup][2]] = 0;
		for (int i = rthumbGroup + 1; i < rthumbGroup + 5; i++) {
			robot.qMax[linkgroups[i][0]] = 0;
			robot.qMax[linkgroups[i][1]] = 0;
			robot.qMax[linkgroups[i][2]] = 0;
			robot.qMin[linkgroups[i][0]] = -2;
			robot.qMin[linkgroups[i][1]] = -1;
			robot.qMin[linkgroups[i][2]] = -1;
		}

		for (int i = lthumbGroup + 1; i < lthumbGroup + 5; i++) {
			robot.qMax[linkgroups[i][0]] = 0;
			robot.qMax[linkgroups[i][1]] = 0;
			robot.qMax[linkgroups[i][2]] = 0;
			robot.qMin[linkgroups[i][0]] = -2;
			robot.qMin[linkgroups[i][1]] = -1;
			robot.qMin[linkgroups[i][2]] = -1;
		}

//		cout << "set torques, velmax, accmax" << flush;
		for (size_t i = 0; i < linkgroups.size(); i++) {
			for (size_t j = 0; j < linkgroups[i].size(); j++) {
				robot.torqueMax[linkgroups[i][j]] = defaultTorqueMax[i];
				robot.velMax[linkgroups[i][j]] = defaultVelMax[i];
				robot.accMax[linkgroups[i][j]] = defaultAccMax[i];
			}
		}
		assert(linkgroups[2].size() == 6);
		robot.torqueMax[linkgroups[2][3]] = 20;
		robot.torqueMax[linkgroups[2][4]] = 10;
		robot.torqueMax[linkgroups[2][5]] = 10;
		assert(linkgroups[8].size() == 6);
		robot.torqueMax[linkgroups[8][3]] = 20;
		robot.torqueMax[linkgroups[8][4]] = 10;
		robot.torqueMax[linkgroups[8][5]] = 10;

//		cout << " set servo PID dryFriction\n" << flush;
		for (size_t i = 0; i < drivergroups.size(); i++) {
			for (size_t j = 0; j < drivergroups[i].size(); j++) {
				robot.drivers[drivergroups[i][j]].servoP = defaultSP[i];
				robot.drivers[drivergroups[i][j]].servoI = defaultSI[i];
				robot.drivers[drivergroups[i][j]].servoD = defaultSD[i];
				robot.drivers[drivergroups[i][j]].dryFriction = defaultDF[i];
			}
		}
		assert(drivergroups[1].size() == 6);
		robot.drivers[drivergroups[1][3]].servoP = 50;
		robot.drivers[drivergroups[1][4]].servoP = 50;
		robot.drivers[drivergroups[1][5]].servoP = 50;
		assert(drivergroups[7].size() == 6);
		robot.drivers[drivergroups[7][3]].servoP = 50;
		robot.drivers[drivergroups[7][4]].servoP = 50;
		robot.drivers[drivergroups[7][5]].servoP = 50;
		robot.drivers[drivergroups[1][3]].servoI = 20;
		robot.drivers[drivergroups[1][4]].servoI = 20;
		robot.drivers[drivergroups[1][5]].servoI = 20;
		robot.drivers[drivergroups[7][3]].servoI = 20;
		robot.drivers[drivergroups[7][4]].servoI = 20;
		robot.drivers[drivergroups[7][5]].servoI = 20;
		assert(drivergroups[14].size() == 6);
		robot.drivers[drivergroups[14][4]].servoI = 20;
		robot.drivers[drivergroups[14][5]].servoI = 20;
		assert(drivergroups[15].size() == 6);
		robot.drivers[drivergroups[15][4]].servoI = 20;
		robot.drivers[drivergroups[15][5]].servoI = 20;
		robot.drivers[drivergroups[14][4]].servoD = 10;
		robot.drivers[drivergroups[14][5]].servoD = 10;
		robot.drivers[drivergroups[15][4]].servoD = 10;
		robot.drivers[drivergroups[15][5]].servoD = 10;
		robot.drivers[drivergroups[1][3]].dryFriction = 0.5;
		robot.drivers[drivergroups[1][4]].dryFriction = 0.5;
		robot.drivers[drivergroups[1][5]].dryFriction = 0.5;
		robot.drivers[drivergroups[7][3]].dryFriction = 0.5;
		robot.drivers[drivergroups[7][4]].dryFriction = 0.5;
		robot.drivers[drivergroups[7][5]].dryFriction = 0.5;
	}
}

std::ostream& operator <<(std::ostream& out, const OrXmlEnvironment& env) {
	out << "**************Start Environment*******************" << endl;
	size_t nRob = env.xmlRobots.size();
	size_t nKin = env.xmlKinbodys.size();
	if (nRob > 0) {
		out << "-num of robot:" << nRob << endl;
		for (size_t i = 0; i < nRob; i++) {
			out << *(env.xmlRobots[i]) << endl;
		}
	}
	if (nKin > 0) {
		out << "-num of KinBody:" << nKin << endl;
		for (size_t i = 0; i < nKin; i++) {
			out << *(env.xmlKinbodys[i]) << endl;
		}
	}
	out << "**************End Environment*******************" << endl;
	return out;
}
std::ostream& operator <<(std::ostream& out, const OrXmlKinbody& kin) {
	out << "--KinBody name:" << kin.name << endl;
	if (kin.transformation)
		out << *kin.transformation << endl;
	size_t nbody = kin.xmlBodys.size();
	if (nbody > 0) {
		out << "--number of Body:" << nbody << endl;
		for (size_t i = 0; i < nbody; i++) {
			out << *(kin.xmlBodys[i]) << endl;
		}
	}
	size_t nkinbody = kin.xmlKinbodys.size();
	if (nkinbody > 0) {
		out << "--number of KinBody:" << nkinbody << endl;
		for (size_t i = 0; i < nkinbody; i++) {
			out << *(kin.xmlKinbodys[i]) << endl;
		}
	}
	size_t nJoint = kin.xmlJoints.size();
	if (nJoint > 0) {
		out << "--number of Joints:" << nJoint << endl;
		for (size_t i = 0; i < nJoint; i++) {
			out << *(kin.xmlJoints[i]) << endl;
		}
	}
	size_t nAdjacent = kin.xmlAdjacents.size();
	if (nAdjacent > 0) {
		out << "--number of Adjacent:" << nAdjacent << endl;
		for (size_t i = 0; i < nAdjacent; i++) {
			out << *(kin.xmlAdjacents[i]) << endl;
		}
	}
	return out;
}

std::ostream& operator <<(std::ostream& out, const OrXmlBody& body) {
	out << "---body name:" << body.name << endl;
	out << "---body type:" << body.type << endl;
	if (!body.offsetfrom.empty())
		out << "---body offsetfrom:" << body.offsetfrom << endl;
	if (body.mass)
		out << *body.mass << endl;
	size_t nGeom = body.xmlGeoms.size();
	out << "---num of Geom in body " << body.name << ":" << nGeom << endl;
	for (size_t i = 0; i < nGeom; i++) {
		if (body.xmlGeoms[i])
			out << *(body.xmlGeoms[i]) << endl;
	}
	if (body.transformation)
		out << *body.transformation << endl;
	return out;
}
std::ostream& operator <<(std::ostream& out, const OrXmlRobot& robot) {
	out << "---robot name:" << robot.name << endl;
	out << "---robot type:" << robot.type << endl;
	out << "---robot file:" << robot.file << endl;
	out << "---robot prefix:" << robot.prefix << endl;
	size_t nKinbody = robot.xmlKinbodys.size();
	out << "---num of Kinbody in robot " << robot.name << ":" << nKinbody
			<< endl;
	for (size_t i = 0; i < nKinbody; i++) {
		if (robot.xmlKinbodys[i])
			out << *(robot.xmlKinbodys[i]) << endl;
	}
	size_t nRobot = robot.xmlRobots.size();
	out << "---num of Robots in robot " << robot.name << ":" << nRobot << endl;
	for (size_t i = 0; i < nRobot; i++) {
		if (robot.xmlRobots[i])
			out << *(robot.xmlRobots[i]) << endl;
	}
	if (robot.jointvalues.size() > 0)
		out << "----joint values:" << endl;
	for (size_t i = 0; i < robot.jointvalues.size(); i++)
		out << robot.jointvalues[i] << " ";
	out << endl;
	if (robot.transformation)
		out << *robot.transformation << endl;
	return out;
}

} // namespace Klampt