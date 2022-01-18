#ifndef VIEW_HOLD_H
#define VIEW_HOLD_H

#include <KrisLibrary/GLdraw/GLColor.h>
#include <Klampt/Contact/Hold.h>
#include "ViewRobot.h"

namespace Klampt {

/** \ingroup View 
 * \brief Displays a contact point using OpenGL.
 *
 * Set the coneColor alpha to 0 to hide the friction cone.
 */
struct ViewContact
{
  ViewContact();
  void Draw();
  void Draw(const ContactPoint& _p) { p=&_p; Draw(); }

  const ContactPoint* p;
  GLDraw::GLColor pointColor, normalColor, coneColor;
  Real normalScale;
};

/** \ingroup View 
 * \brief Displays a hold using OpenGL.
 *
 * Modify the internal viewContact structure
 * to change how the contact points are displayed.
 */
struct ViewHold
{
  ViewHold();
  void Draw();
  void DrawContacts();
  void DrawOutline();
  void DrawLabel(const char* label);
  void Draw(const Hold& _h) { h=&_h; Draw(); }
  void DrawContacts(const Hold& _h) { h=&_h; DrawContacts(); }
  void DrawOutline(const Hold& _h) { h=&_h; DrawOutline(); }
  void DrawLabel(const Hold& _h,const char* label) { h=&_h; DrawLabel(label); }

  //draws the link of the hold as it should be placed according to the IK constraint
  void DrawConstraint(const Hold& _h,ViewRobot& robotviewer);
  void DrawConstraint(const Hold& _h,ViewRobot& robotviewer,const Matrix3& refMatrix);

  const Hold* h;
  ViewContact viewContact;
  bool drawContacts;
  GLDraw::GLColor outlineColor;
};

} //namespace Klampt

#endif
