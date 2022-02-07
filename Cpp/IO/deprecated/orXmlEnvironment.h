#ifndef XML_ENVIRONMENT_H
#define XML_ENVIRONMENT_H

//#include <string>
//#include <tinyxml.h>
//#include <vector>
//#include <math3d/primitives.h>
#include <Klampt/Modeling/World.h>
#include "orXmlTags.h"

namespace Klampt {
  using namespace std;
  using namespace Math3D;

class OrXmlBody;
string ToLowercase(string str);

class OrXmlKinbody {
public:
	OrXmlKinbody(TiXmlElement* element);
	~OrXmlKinbody();
	bool GetContent();
	bool GetObjectOrTerrain(WorldModel& world);
	bool GetAllCleanBodyJoints();
	void GetAllBodyGeom();
//	bool getParentChildRelation();

	void applyRelativeTransformation(int from, int to);
	void setPrefixName();
	bool getRealContent();
	bool appyTran2Tparent(OrXmlTransformation* tran);

	bool RemoveSameBody();

	void combineTwoBody(OrXmlBody* b1,OrXmlBody* b2);

	bool isGetAllBodys;
	bool hasSetPrefix;
	bool isGetAllCleanBody;

	TiXmlElement* e;
	TiXmlDocument* doc;

	string prefix;
	string type;
	string file;
	string name;
	OrXmlMass* mass;
	OrXmlTransformation* transformation;
	string modelsdir;
	string offsetfrom;

	string kin_dir_str;

	//if isGetAllBodys==true, the following 4 vector stores all info of other kinbodys
	vector<double> jointvalues;
	vector<OrXmlJoint*> xmlJoints;
	vector<OrXmlAdjacent*> xmlAdjacents;
	vector<OrXmlBody*> xmlBodys;

	vector<int> parents;
	vector< vector<int> > children;

	vector<OrXmlKinbody*> xmlKinbodys;
};

class OrXmlRobot {
public:
	OrXmlRobot(TiXmlElement* element);
	~OrXmlRobot();
	bool GetContent();
	bool getRealContent();
	bool GetAllCleanBodyJoints();
	bool GetRobot(Robot& robot);
	bool ConvertToURDF();
	bool GetFixedBaseRobot(Robot& robot);
	bool GetFloatingBaseRobot(Robot& robot);
	bool GetParentChildRelation();
	void GetTransformAnchorAxis();
	void ProcessJointAnchorInitial();
	void WriteMesh2Wrl();
	bool ReSetBodyJointOrder();
	void setPrefixName();
	bool RemoveSameBody();

	void combineTwoBody(OrXmlBody* b1,OrXmlBody* b2);

	void setJaemihuboProperty(Robot& robot);
	void WriteHubo(Robot& robot, bool isjaemi);
	void setHuboplusProperty(Robot& robot);
	void Write2Rob(Robot& robot);

	void recomputeTworld();
	void writeToURDF();
	void getDiffJointNames();
	void setNewRelativeTran(int tmpchild, const Vector3& an);
	void rewriteTriMesh(int tmpchild, const Vector3& an, bool visible);
	int getLinkIndex(const string& linkname);
	void applyRelativeTransformation(int from, int to);
	bool applyTran2Tparent();

	int linkroot;

	string name;
	string file;
	string prefix;
	string type;
	OrXmlTransformation* transformation;
//	vector<double> jointvalues;
	vector<OrXmlKinbody*> xmlKinbodys;
	vector<OrXmlRobot*> xmlRobots;

	bool hasSetPrefix;
	bool allclean;

	TiXmlDocument* doc;
	TiXmlElement* e;

	int nDOF;

	//all info from Robots and Kinbodys
	vector<double> jointvalues;
	vector<OrXmlJoint*> xmlJoints;
	vector<OrXmlAdjacent*> xmlAdjacents;
	vector<OrXmlBody*> xmlBodys;

	vector<int> parents;
	vector< vector<int> > children;
};

class OrXmlBody {
public:
	OrXmlBody(TiXmlElement* element);
	~OrXmlBody();
	bool GetContent();
	bool GetFinalGeom();

	void GetMimicGeomMass();

	bool applyTran2Tparent(OrXmlTransformation* tran);
	void addMesh(const vector<MyPoint3D>& pts, const vector<TriFaceIndex>& tis, bool visible);
	void output2Tri();
	void output2Wrl();

	string name;
	string type;
	string offsetfrom;
	OrXmlTransformation* transformation;
	vector<OrXmlGeom*> xmlGeoms;
	OrXmlMass* mass;

	string vis_filename;
	string col_filename;
	TiXmlElement* e;

	RigidTransform* Tworld;
	RigidTransform* Tparent;

	vector<MyPoint3D> vispoints;
	vector<TriFaceIndex> visindexes;

	vector<MyPoint3D> colpoints;
	vector<TriFaceIndex> colindexes;
};

class OrXmlEnvironment {
public:
	OrXmlEnvironment();
	~OrXmlEnvironment();
	bool Load(const string& fn);
	bool GetWorld(WorldModel& world);
	bool Convert2URDF();
	bool Convert2Rob();
	bool getWorld();
	void clear();

	TiXmlElement* GetElement(const string& name);
	TiXmlElement* GetElement(const string& name, int index);
	TiXmlElement* GetRobot(int index) {
		return GetElement("robot", index);
	}
	TiXmlElement* GetRigidObject(int index) {
		return GetElement("rigidObject", index);
	}
	TiXmlElement* GetTerrain(int index) {
		return GetElement("terrain", index);
	}

	TiXmlDocument doc;

	vector<OrXmlRobot*> xmlRobots;
	vector<OrXmlKinbody*> xmlKinbodys;
};

std::ostream& operator <<(std::ostream&, const OrXmlEnvironment&);
std::ostream& operator <<(std::ostream&, const OrXmlKinbody&);
std::ostream& operator <<(std::ostream&, const OrXmlBody&);
std::ostream& operator <<(std::ostream&, const OrXmlRobot&);

} // namespace Klampt

#endif

