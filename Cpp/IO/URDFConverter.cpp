/*
 * URDFConverter.cpp
 *
 *  Created on: Mar 10, 2013
 *      Author: yajia
 */

#include "URDFConverter.h"
#include <fstream>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/geometry/AnyGeometry.h>
#include <KrisLibrary/math3d/geometry3d.h>
#include <string.h>
using namespace std;
using namespace Geometry;
using namespace Math3D;

string URDFConverter::packageRootPath("");
bool URDFConverter::useVisGeom = false;
bool URDFConverter::flipYZ = false;

int URDFConverter::GetLinkIndexfromName(string name, const vector<string> linknames){
	int link_index = -1;
	for (size_t i = 0; i < linknames.size(); i++) {
		if ( strcmp(name.c_str(), linknames[i].c_str()) == 0) {
			link_index = i;
			break;
		}
	}
	return link_index;
}

RobotJoint::Type URDFConverter::jointType_URDF2ROB(int type) {
	//ROB { Weld, Normal, Spin, Floating, FloatingPlanar, BallAndSocket, Closed };
	//URDF {UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED };

	RobotJoint::Type robtype = RobotJoint::Weld;
	if (type == urdf::Joint::REVOLUTE)
		robtype = RobotJoint::Normal;
	if (type == urdf::Joint::CONTINUOUS)
		robtype = RobotJoint::Spin;
	else if (type == urdf::Joint::FIXED)
		robtype = RobotJoint::Weld;
	else if (type == urdf::Joint::FLOATING)
		robtype = RobotJoint::Floating;
	else if (type == urdf::Joint::PLANAR)
		robtype = RobotJoint::FloatingPlanar;
	else if (type == urdf::Joint::PRISMATIC) {
		robtype = RobotJoint::Normal;
	}
	return robtype;
}

void URDFConverter::DFSLinkTree(URDFLinkNode& root, vector<URDFLinkNode>& linkNodes) {
	linkNodes.push_back( root);

	//Parent has index
	for (size_t i = 0; i < root.link->child_links.size(); i++) {
		URDFLinkNode child( root.link->child_links[i], linkNodes.size(), root.index);
		DFSLinkTree( child, linkNodes);
	}
}

void URDFConverter::setJointforNodes(vector< std::shared_ptr<urdf::Joint> >& joints, vector<URDFLinkNode>& linkNodes){
	for(size_t i = 0; i < linkNodes.size(); i++){
		string linkname = linkNodes[i].link->name;
		linkNodes[i].joint = 0;
		for (size_t j = 0; j < joints.size(); j++) {
			std::shared_ptr<urdf::Joint> joint = joints[j];
			//find link index of the joint
			string joint_name = joint->child_link_name;
			if(0 == strcmp(joint_name.c_str(), linkname.c_str())){
				linkNodes[i].joint = joint.get();
				break;
			}
		}
	}
}

void URDFConverter::QuatToRotationMat(const Vector4& aa, Matrix3& mat){
	double y,z,w,x;
	x = aa.x;
	y = aa.y;
	z = aa.z;
	w = aa.w;

	mat(0,0) = 1 - 2*y*y - 2*z*z;
	mat(1,0) = 2*x*y + 2*z*w;
	mat(2,0) = 2*x*z - 2*w*y;
	mat(0,1) = 2*y*x - 2*z*w;
	mat(1,1) = 1 - 2*x*x - 2*z*z;
	mat(2,1) = 2*z*y + 2*x*w;
	mat(0,2) = 2*x*z + 2*w*y;
	mat(1,2) = 2*z*y - 2*x*w;
	mat(2,2) = 1 - 2*x*x - 2*y*y;
}

URDFLinkNode::URDFLinkNode(std::shared_ptr<urdf::Link>& link, int index, int index_parent) {
	this->link = link;
	this->index = index;
	this->index_parent = index_parent;
	this->T_link_to_inertia.setIdentity();
	this->T_link_to_inertia_inverse.setIdentity();
	this->T_link_to_visgeom.setIdentity();
	this->T_link_to_colgeom.setIdentity();
	this->T_parent.setIdentity();
	this->joint = NULL;
	this->axis.set(0,0,1);

	this->GetTransformations();
	return;
}

void URDFLinkNode::GetGeometryProperty(bool useVisGeom){
	if(!this->link){
		cout<<"link is NULL!"<<endl;
		return;
	}
	geomScale.setIdentity();
	geomPrimitive = false;
	std::shared_ptr<urdf::Geometry> geom;
	if(useVisGeom && this->link->visual) geom = this->link->visual->geometry;
	else if(!useVisGeom && this->link->collision) geom = this->link->collision->geometry;
	if(geom){
	  if(geom->type == urdf::Geometry::BOX){
		  std::shared_ptr<urdf::Box> box = std::static_pointer_cast<urdf::Box>(geom);
		  geomPrimitive = true;
		  AABB3D b;
		  b.bmin.set(-box->dim.x*0.5,-box->dim.y*0.5,-box->dim.z*0.5);
		  b.bmax = -b.bmin;
		  GeometricPrimitive3D prim(b);
		  AnyGeometry3D geom(prim);
		  stringstream ss;
		  geom.Save(ss);
		  geomData = ss.str();

		}else if(geom->type == urdf::Geometry::CYLINDER){
		  	std::shared_ptr<urdf::Cylinder> cylinder = std::static_pointer_cast<urdf::Cylinder>(geom);
		  	geomPrimitive = true;
			Cylinder3D c;
			c.center.setZero();
			c.axis.set(0,0,1);
			c.radius = cylinder->radius;
			c.height = cylinder->length;
			GeometricPrimitive3D prim(c);
			AnyGeometry3D geom(prim);
		  	stringstream ss;
		  	geom.Save(ss);
		  	geomData = ss.str();
		}else if(geom->type == urdf::Geometry::SPHERE){
			std::shared_ptr<urdf::Sphere> sphere = std::static_pointer_cast<urdf::Sphere>(geom);
			geomPrimitive = true;
			Sphere3D s;
			s.center.setZero();
			s.radius = sphere->radius;
			GeometricPrimitive3D prim(s);
			AnyGeometry3D geom(prim);
		  	stringstream ss;
		  	geom.Save(ss);
		  	geomData = ss.str();
		}
	  else if(geom->type == urdf::Geometry::MESH){
			std::shared_ptr<urdf::Mesh> mesh = std::static_pointer_cast<urdf::Mesh>(geom);
			geomName = mesh->filename.c_str();
			if(geomName.compare(0,10,"package://")==0) {
			  //take off the first 10 characters
			  geomName = URDFConverter::packageRootPath + "/" + geomName.substr(10,string::npos);
			}
			geomScale(0,0) = mesh->scale.x;
			geomScale(1,1) = mesh->scale.y;
			geomScale(2,2) = mesh->scale.z;
			if(URDFConverter::flipYZ) {
			  Matrix4 Ryz; 
			  Ryz.setZero();
			  Ryz(0,0) = 1;
			  Ryz(1,2) = -1;
			  Ryz(2,1) = 1;
			  Ryz(3,3) = 1;
			  geomScale = geomScale * Ryz;
			}
		}
	  else {
	    cout<<"Unknown URDF geometry type "<<(int)geom->type<<endl;
	  }
	}else{
		geomName = "";
		geomScale.setIdentity();
	}
}

void URDFLinkNode::GetTransformations(){
	if(!this->link){
		cout<<"link is NULL!"<<endl;
		return;
	}
	urdf::Vector3 pos;
	urdf::Rotation rotation;
	Vector4 quat;

	if(this->link->inertial){
		pos.set(this->link->inertial->origin.position);
		rotation.set(this->link->inertial->origin.rotation);
		quat.set(rotation.x, rotation.y, rotation.z, rotation.w);
		URDFConverter::QuatToRotationMat(quat, T_link_to_inertia.R);
		this->T_link_to_inertia.t.set(pos.x, pos.y, pos.z);
		this->T_link_to_inertia_inverse.setInverse(this->T_link_to_inertia);
	}

	if(this->link->collision){
		pos.set(this->link->collision->origin.position);
		rotation.set(this->link->collision->origin.rotation);
		quat.set(rotation.x, rotation.y, rotation.z, rotation.w);
		URDFConverter::QuatToRotationMat(quat, T_link_to_colgeom.R);
		this->T_link_to_colgeom.t.set(pos.x, pos.y, pos.z);
	}
	if(this->link->visual){
		pos.set(this->link->visual->origin.position);
		rotation.set(this->link->visual->origin.rotation);
		quat.set(rotation.x, rotation.y, rotation.z, rotation.w);
		URDFConverter::QuatToRotationMat(quat, T_link_to_visgeom.R);
		this->T_link_to_visgeom.t.set(pos.x, pos.y, pos.z);
	}
}

Math3D::Matrix3 URDFConverter::convertInertial(urdf::Inertial& I) {
	Math3D::Matrix3 m;
	m(0, 0) = I.ixx;
	m(0, 1) = m(1, 0) = I.ixy;
	m(0, 2) = m(2, 0) = I.ixz;
	m(1, 1) = I.iyy;
	m(1, 2) = m(2, 1) = I.iyz;
	m(2, 2) = I.izz;
	return m;
}

//void URDFConverter::ScalePrimitiveGeom(string infilename, string outfilename, const Vector3& geomScale){
//	vector<MyPoint3D> points;
//	vector<TriFaceIndex> indexes;
//	loadWrl(infilename, points, indexes);
//
//	Matrix4 mscale;
//	mscale.setIdentity();
//	mscale(0, 0) = geomScale[0];
//	mscale(1, 1) = geomScale[1];
//	mscale(2, 2) = geomScale[2];
//
//	for(int i = 0; i < points.size(); i++)
//}

void URDFConverter::processTParentTransformations(vector<URDFLinkNode>& linkNodes){
	for(int i = 0; i < linkNodes.size(); i++){
	  linkNodes[i].GetGeometryProperty(useVisGeom);
		RigidTransform T0, T1, T2;
		T0.setIdentity();
		Vector3 tmpaxis;
		if(linkNodes[i].joint){
			urdf::Pose pose = linkNodes[i].joint->parent_to_joint_origin_transform;
			T0.t.set(pose.position.x, pose.position.y, pose.position.z);
			Vector4 quat(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
			QuatToRotationMat(quat, T0.R);

			tmpaxis.set( linkNodes[i].joint->axis.x, linkNodes[i].joint->axis.y, linkNodes[i].joint->axis.z);
			if(tmpaxis.norm() > 0){
				linkNodes[i].axis.set( linkNodes[i].joint->axis.x, linkNodes[i].joint->axis.y, linkNodes[i].joint->axis.z);
			}

		}
		linkNodes[i].T_parent.set(T0);

		RigidTransform G0, G1;
		G1.setIdentity();
		if(useVisGeom) {
		  G0.mul(linkNodes[i].T_link_to_inertia_inverse, linkNodes[i].T_link_to_visgeom);
		  G1.mul(linkNodes[i].T_link_to_visgeom, linkNodes[i].geomScale);
		}
		else {
		  G0.mul(linkNodes[i].T_link_to_inertia_inverse, linkNodes[i].T_link_to_colgeom);
		  G1.mul(linkNodes[i].T_link_to_colgeom, linkNodes[i].geomScale);
		}
		linkNodes[i].geomScale.set(G1);
	}
}

