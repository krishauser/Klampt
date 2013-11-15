/*
 * URDFConverter.h
 *
 *  Created on: Mar 10, 2013
 *      Author: yajia
 */

#ifndef URDFCONVERTER_H_
#define URDFCONVERTER_H_

#include <vector>
#include <math3d/primitives.h>
#include "Modeling/Robot.h"
#include "PrimitiveShape.h"
#include "urdf_link.h"
using namespace std;
using namespace Math3D;

class URDFLinkNode {
public:
	URDFLinkNode(boost::shared_ptr<urdf::Link>& link, int index, int index_parent);
	void GetTransformations();
	void GetGeometryProperty(bool useVisGeom=false);
	void GetJoint();
	boost::shared_ptr<urdf::Link> link;
	int index;
	int index_parent;
	RigidTransform T_link_to_inertia;
	RigidTransform T_link_to_inertia_inverse;
	RigidTransform T_link_to_visgeom;
	RigidTransform T_link_to_colgeom;
	RigidTransform T_parent;
	Vector3 axis;
	bool geomPrimitive;
	string geomName;
	Matrix4 geomScale;
	urdf::Joint* joint;
};

class URDFConverter {
public:
	static int GetLinkIndexfromName(string name, const vector<string> linknames);
	static RobotJoint::Type jointType_URDF2ROB(int );
	static void DFSLinkTree( URDFLinkNode& root, vector<URDFLinkNode>& linkNodes);
	static void setJointforNodes(vector< boost::shared_ptr<urdf::Joint> >& joints, vector<URDFLinkNode>& linkNodes);
	static Math3D::Matrix3 convertInertial( urdf::Inertial& I);
	static void QuatToRotationMat(const Vector4& aa, Matrix3& mat);
	static void processTParentTransformations(vector<URDFLinkNode>& linkNodes);
	static void ConvertWrltoTri(string filename);
//	static void ScalePrimitiveGeom(string infilename, string outfilename);

	//The location of primitive_mesh must be provided.
	static string primitive_mesh_path;
	//Set this to true if visualization geometry should be used
	static bool useVisGeom;
	//Set this to true if the geometry Y-Z plane should be flipped
	static bool flipYZ; 
};

#endif /* URDFCONVERTER_H_ */
