#ifndef VIEW_CAMERA_H
#define VIEW_CAMERA_H

#include <KrisLibrary/GLdraw/GLColor.h>
#include <KrisLibrary/camera/viewport.h>
#include <KrisLibrary/math3d/primitives.h>

namespace Klampt {

/** @brief Draws a camera in the OpenGL world.
 *
 * If drawIcon = true (default), draws a little icon for the camera
 *   showing the viewing pyramid.
 * If drawFrustum = true (default), draws the camera frustum boundaries
 *   in wireframe.
 */
class ViewCamera
{
public:
	ViewCamera();
	void DrawGL(const Camera::Viewport& v) const;

	bool drawIcon,drawIconWireframe;
	double iconSize;
	bool drawFrustum;
    bool drawCoords;
    double coordsLen;
    GLDraw::GLColor iconColor,frustumColor;
};

} //namespace Klampt

#endif