/*
 * URDFConverter.h
 *
 *  Created on: Mar 10, 2013
 *      Author: yajia
 */

#ifndef URDFCONVERTER_H_
#define URDFCONVERTER_H_

#include <vector>
#include <KrisLibrary/math3d/primitives.h>
#include "Modeling/Robot.h"
#include "urdf_link.h"
using namespace std;
using namespace Math3D;

class URDFLinkNode {
public:
	URDFLinkNode(std::shared_ptr<urdf::Link>& link, int index, int index_parent);
	void GetTransformations();
	void GetGeometryProperty(bool useVisGeom=false);
	void GetJoint();
	std::shared_ptr<urdf::Link> link;
	int index;
	int index_parent;
	RigidTransform T_link_to_inertia;
	RigidTransform T_link_to_inertia_inverse;
	RigidTransform T_link_to_visgeom;
	RigidTransform T_link_to_colgeom;
	RigidTransform T_parent;
	Vector3 axis;
	string geomName;   //if empty, the geometry is loaded in geomData
	Geometry::AnyGeometry3D geomData;
	Matrix4 geomScale;
	urdf::Joint* joint;
};

class URDFConverter {
public:
	static int GetLinkIndexfromName(string name, const vector<string> linknames);
	static RobotJoint::Type jointType_URDF2ROB(int );
	static void DFSLinkTree( URDFLinkNode& root, vector<URDFLinkNode>& linkNodes);
	static void setJointforNodes(vector< std::shared_ptr<urdf::Joint> >& joints, vector<URDFLinkNode>& linkNodes);
	static Math3D::Matrix3 convertInertial( urdf::Inertial& I);
	static void processTParentTransformations(vector<URDFLinkNode>& linkNodes);

	//The location for package:// directives
	static string packageRootPath;
	//Set this to true if visualization geometry should be used
	static bool useVisGeom;
	//Set this to true if the geometry Y-Z plane should be flipped
	static bool flipYZ; 
};

#endif /* URDFCONVERTER_H_ */
