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
 #include <KrisLibrary/math3d/rotation.h>
#include <string.h>
using namespace std;
using namespace Geometry;
using namespace Math3D;

string URDFConverter::packageRootPath("");
bool URDFConverter::useVisGeom = false;
bool URDFConverter::flipYZ = false;

void PoseToTransform(const urdf::Pose& pose,RigidTransform& T)
{
	QuaternionRotation quat;
	quat.set(Quaternion(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z));
	quat.getMatrix(T.R);
	T.t.set(pose.position.x, pose.position.y, pose.position.z);
}


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
	else if (type == urdf::Joint::CONTINUOUS)
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

bool extractGeomData(std::shared_ptr<urdf::Geometry> geom,AnyGeometry3D& out)
{
	if(geom->type == urdf::Geometry::BOX){
		  std::shared_ptr<urdf::Box> box = std::static_pointer_cast<urdf::Box>(geom);
		  AABB3D b;
		  b.bmin.set(-box->dim.x*0.5,-box->dim.y*0.5,-box->dim.z*0.5);
		  b.bmax = -b.bmin;
		  GeometricPrimitive3D prim(b);
		  out = AnyGeometry3D(prim);
		  return true;
	}
	else if(geom->type == urdf::Geometry::CYLINDER){
	  	std::shared_ptr<urdf::Cylinder> cylinder = std::static_pointer_cast<urdf::Cylinder>(geom);
		Cylinder3D c;
		c.center.set(0,0,-cylinder->length*0.5);
		c.axis.set(0,0,1);
		c.radius = cylinder->radius;
		c.height = cylinder->length;
		GeometricPrimitive3D prim(c);
		out = AnyGeometry3D(prim);
		return true;
	}else if(geom->type == urdf::Geometry::SPHERE){
		std::shared_ptr<urdf::Sphere> sphere = std::static_pointer_cast<urdf::Sphere>(geom);
		Sphere3D s;
		s.center.setZero();
		s.radius = sphere->radius;
		GeometricPrimitive3D prim(s);
		out = AnyGeometry3D(prim);
		return true;
	}
	else if(geom->type == urdf::Geometry::MESH) {
		std::shared_ptr<urdf::Mesh> mesh = std::static_pointer_cast<urdf::Mesh>(geom);
		std::string geomName = mesh->filename;
		if(geomName.compare(0,10,"package://")==0) {
		  //take off the first 10 characters
		  geomName = URDFConverter::packageRootPath + "/" + geomName.substr(10,string::npos);
		}
		if(!out.Load(geomName.c_str())) {
			cout<<"Could not load mesh from file "<<geomName<<endl;
			return false;
		}
		Matrix4 geomScale;
		geomScale.setIdentity();
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
		out.Transform(geomScale);
		return true;
	}
	else {
		cout<<"Unknown URDF geometry type "<<(int)geom->type<<endl;
	}
	return false;
}

void URDFLinkNode::GetGeometryProperty(bool useVisGeom){
	if(!this->link){
		cout<<"link is NULL!"<<endl;
		return;
	}
	geomScale.setIdentity();
	geomName="";
	std::vector<std::shared_ptr<urdf::Geometry> > geoms;
	std::vector<urdf::Pose> geomPoses;
	if(useVisGeom && this->link->visual) {
		geoms = this->link->getVisualGeoms("default");
		geomPoses = this->link->getVisualPoses("default");
	}
	else if(!useVisGeom && this->link->collision) {
		geoms = this->link->getCollisionGeoms("default");
		geomPoses = this->link->getCollisionPoses("default");
	}
	if(!geoms.empty()){
		if(geoms.size() == 1 || true) {
			auto geom = geoms[0];
			if(geom->type == urdf::Geometry::MESH) {
				std::shared_ptr<urdf::Mesh> mesh = std::static_pointer_cast<urdf::Mesh>(geom);
				geomName = mesh->filename;
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
			else
				extractGeomData(geom,geomData);
		}
		else {
			if(useVisGeom)
				cout<<"URDFConverter: Warning, multiple vis geometries?"<<endl;
			vector<AnyGeometry3D> subitems(geoms.size());
			RigidTransform T0,T0inv,Ti;
			PoseToTransform(geomPoses[0],T0);
			T0inv.setInverse(T0);
			for(size_t i=0;i<subitems.size();i++) {
				if(!extractGeomData(geoms[i],subitems[i])) {
					cout<<"URDFConverter: Warning, unable to convert geometry item "<<i<<endl;
				}
				else {
					assert(subitems[i].AsPrimitive().type != GeometricPrimitive3D::Empty);
					if(i > 0) {
						PoseToTransform(geomPoses[i],Ti);
						subitems[i].Transform(T0inv*Ti);
					}
				}
			}
			geomData = AnyGeometry3D(subitems);
			for(size_t i=0;i<subitems.size();i++)
				assert(subitems[i].AsPrimitive().type != GeometricPrimitive3D::Empty);
			for(const auto& i:geomData.AsGroup()) {
				assert(i.AsPrimitive().type != GeometricPrimitive3D::Empty);
			}
		}
	}
}

void URDFLinkNode::GetTransformations(){
	if(!this->link){
		cout<<"link is NULL!"<<endl;
		return;
	}
	
	if(this->link->inertial){
		PoseToTransform(this->link->inertial->origin,this->T_link_to_inertia);
		this->T_link_to_inertia_inverse.setInverse(this->T_link_to_inertia);
	}

	if(this->link->collision){
		PoseToTransform(this->link->collision->origin, T_link_to_colgeom);
	}
	if(this->link->visual){
		PoseToTransform(this->link->visual->origin, T_link_to_visgeom);
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
	for(size_t i = 0; i < linkNodes.size(); i++){
	  linkNodes[i].GetGeometryProperty(useVisGeom);
		RigidTransform T0;
		T0.setIdentity();
		Vector3 tmpaxis;
		if(linkNodes[i].joint){
			PoseToTransform(linkNodes[i].joint->parent_to_joint_origin_transform,T0);

			tmpaxis.set( linkNodes[i].joint->axis.x, linkNodes[i].joint->axis.y, linkNodes[i].joint->axis.z);
			if(tmpaxis.norm() > 0){
				linkNodes[i].axis = tmpaxis;
			}

		}
		linkNodes[i].T_parent.set(T0);

		RigidTransform G1;
		if(useVisGeom) {
		  G1.mul(linkNodes[i].T_link_to_visgeom, linkNodes[i].geomScale);
		}
		else {
		  G1.mul(linkNodes[i].T_link_to_colgeom, linkNodes[i].geomScale);
		}
		linkNodes[i].geomScale.set(G1);
	}
}

