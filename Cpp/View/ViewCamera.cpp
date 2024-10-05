#include "ViewCamera.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLError.h>
#include <stdio.h>
using namespace Math;
using namespace Math3D;
using namespace GLDraw;
using namespace Klampt;

ViewCamera::ViewCamera()
:drawIcon(true),drawIconWireframe(false),iconSize(0.05),
 drawFrustum(true),drawCoords(true),coordsLen(0.1)
{
  iconColor.set(0.3f,0.3f,0.3f);
  frustumColor.set(1,1,0,0.5);
}

void ViewCamera::DrawGL(const Camera::Viewport& v) const
{
	if(v.ori != Camera::CameraConventions::OpenGL) {
		fprintf(stderr,"ViewCamera::DrawGL: Warning, only OpenGL orientation is supported for now\n");
		return;
	}
	Real aspectRatio = Real(v.w)/Real(v.h);
	glPushMatrix();
	glMultMatrix(Matrix4(v.pose));
	//note that +z is *backward* in the camera view
	if(drawIcon) {
		Real scale = 2.0*v.fx/v.w;
		Real pyr_h = iconSize*scale;
		Real pyr_base = -pyr_h + iconSize*0.25;
		if(drawIconWireframe) {
			glDisable(GL_LIGHTING);
			iconColor.setCurrentGL();
			glTranslatef(0,0,-iconSize*0.25);
			drawWireBox(iconSize,iconSize/aspectRatio,iconSize*0.5);
			glTranslatef(0,0,iconSize*0.25);
			glTranslatef(0,0,-pyr_h);
			drawWirePyramid(iconSize,iconSize/aspectRatio,pyr_h);
			glTranslatef(0,0,pyr_h);
		}
		else {
			glEnable(GL_LIGHTING);
			glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,iconColor.rgba);
			glTranslatef(0,0,-iconSize*0.25);
			drawBox(iconSize,iconSize/aspectRatio,iconSize*0.5);
			glTranslatef(0,0,iconSize*0.25);
			glTranslatef(0,0,-pyr_h);
			drawPyramid(iconSize,iconSize/aspectRatio,pyr_h);
			glTranslatef(0,0,pyr_h);
		}
	}
	if(drawFrustum) {
		AABB2D bb2d = v.getViewRectangle(1.0,false);
		Real xmin=bb2d.bmin.x,xmax=bb2d.bmax.x,ymin=bb2d.bmin.y,ymax=bb2d.bmax.y;
		glDisable(GL_LIGHTING);
		if(frustumColor.rgba[3] != 1.0) {
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		}
		frustumColor.setCurrentGL();
		glBegin(GL_LINES);
		//near plane
		glVertex3f(v.n*xmax,v.n*ymax,-v.n);
		glVertex3f(v.n*xmax,v.n*ymin,-v.n);
		glVertex3f(v.n*xmax,v.n*ymin,-v.n);
		glVertex3f(v.n*xmin,v.n*ymin,-v.n);
		glVertex3f(v.n*xmin,v.n*ymin,-v.n);
		glVertex3f(v.n*xmin,v.n*ymax,-v.n);
		glVertex3f(v.n*xmin,v.n*ymax,-v.n);
		glVertex3f(v.n*xmax,v.n*ymax,-v.n);
		//far plane
		glVertex3f(v.f*xmax,v.f*ymax,-v.f);
		glVertex3f(v.f*xmax,v.f*ymin,-v.f);
		glVertex3f(v.f*xmax,v.f*ymin,-v.f);
		glVertex3f(v.f*xmin,v.f*ymin,-v.f);
		glVertex3f(v.f*xmin,v.f*ymin,-v.f);
		glVertex3f(v.f*xmin,v.f*ymax,-v.f);
		glVertex3f(v.f*xmin,v.f*ymax,-v.f);
		glVertex3f(v.f*xmax,v.f*ymax,-v.f);
		//connections
		glVertex3f(v.n*xmax,v.n*ymax,-v.n);
		glVertex3f(v.f*xmax,v.f*ymax,-v.f);
		glVertex3f(v.n*xmax,v.n*ymin,-v.n);
		glVertex3f(v.f*xmax,v.f*ymin,-v.f);
		glVertex3f(v.n*xmin,v.n*ymin,-v.n);
		glVertex3f(v.f*xmin,v.f*ymin,-v.f);
		glVertex3f(v.n*xmin,v.n*ymax,-v.n);
		glVertex3f(v.f*xmin,v.f*ymax,-v.f);
		glEnd();
		if(frustumColor.rgba[3] != 1.0) {
			glDisable(GL_BLEND);
		}
	}
	if(drawCoords) {
		glPushMatrix();
		Matrix3 flipYZ;
		flipYZ.setZero();
		flipYZ(0,0) = 1;
		flipYZ(1,1) = -1;
		flipYZ(2,2) = -1;
		glMultMatrix(Matrix4(flipYZ));
		GLDraw::drawCoords(coordsLen);
		glPopMatrix();
	}
	glPopMatrix();
}