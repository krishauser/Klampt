#ifndef VIEW_WRENCH_H
#define VIEW_WRENCH_H

#include <KrisLibrary/GLdraw/GLColor.h>
#include <KrisLibrary/math3d/primitives.h>

namespace Klampt {

class ViewWrench
{
public:
	ViewWrench();
	void DrawGL(const Math3D::Vector3& center,const Math3D::Vector3& f,const Math3D::Vector3& m) const;
	Math::Real fscale,mscale;
    GLDraw::GLColor forceColor,momentColor,centerColor;
};

} // namespace Klampt

#endif