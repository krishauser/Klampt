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
	//Real fov = 2.0*Atan(0.5/v.scale);
	Real aspectRatio = Real(v.w)/Real(v.h);
	glPushMatrix();
	glMultMatrix(Matrix4(v.xform));
	//note that +z is *backward* in the camera view
	if(drawIcon) {
		Real pyr_h = iconSize*v.scale;
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
		Real xmin = Real(v.x - v.w*0.5)/(Real(v.w)*0.5);
		Real xmax = Real(v.x + v.w*0.5)/(Real(v.w)*0.5);
		Real ymax = -Real(v.y - v.h*0.5)/(Real(v.h)*0.5);
		Real ymin = -Real(v.y + v.h*0.5)/(Real(v.h)*0.5);
		Real xscale = 0.5/v.scale;
		Real yscale = xscale/aspectRatio;
		xmin *= xscale;
		xmax *= xscale;
		ymin *= yscale;
		ymax *= yscale;
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