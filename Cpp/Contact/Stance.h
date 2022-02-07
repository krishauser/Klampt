#ifndef STANCE_H
#define STANCE_H

#include "Hold.h"
#include <KrisLibrary/math3d/Plane3D.h>
#include <KrisLibrary/math/matrix.h>
#include <map>
#include <vector>

namespace Klampt {

using namespace std;

/** @ingroup Contact
 *  @brief A collection of holds.
 *
 * Represented by a map of links to holds.  Removals are performed
 * by link index.
 */
struct Stance : public map<int,Hold>
{
  typedef map<int,Hold>::iterator iterator;
  typedef map<int,Hold>::const_iterator const_iterator;

  void insert(const Hold& h) { operator[](h.link)=h; }
  bool contains(int link) const { return find(link)!=end(); }
  bool remove(int link);

  ///returns true if the stance is valid (i.e. links map to appropriate holds)
  bool isValid() const;
};

//returns true if the stance is valid
bool CopyStance(const vector<int>& indices,const vector<Hold>& holds,Stance& s);

/** @ingroup Contact
 * @brief If b = a union {h}, returns h.
 *
 * Unexpected results may occur if the precondition b = a union {h}
 * is not fulfilled.
 */
const Hold& DifferingHold(const Stance& a,const Stance& b);

/** @ingroup Contact
 * @brief Returns true if a and b share the same links
 */
bool IsSimilarStance(const Stance& a,const Stance& b);

/** @ingroup Contact
 * @brief Returns true if a is a subset of b
 */
bool IsSubset(const Stance& a,const Stance& b);

/** @ingroup Contact
 * @brief Returns the difference d = a-b 
 */
void GetDifference(const Stance& a,const Stance& b,vector<Hold>& d);

istream& operator >> (istream& in,Stance& stance);
ostream& operator << (ostream& out,const Stance& stance);

int NumContactPoints(const Stance& s);
void GetContactPoints(const Stance& s,vector<ContactPoint>& cps); 
const ContactPoint& GetContactPoint(const Stance& s,int i);
ContactPoint& GetContactPoint(Stance& s,int i);
Vector3 GetCentroid(const Stance& s,Real L=Zero);
Real GetStdDev(const Stance& s);
Real GetStdDev(const Stance& s,const Vector3& c);  //provide the centroid
void GetPlaneFit(const Stance& s,Plane3D& p);
void GetPlaneFit(const Stance& s,const Vector3& c,Plane3D& p);  //provides the centroid

///@ingroup Contact
///converts a stance to a contact formation
void ToContactFormation(const Stance& stance,ContactFormation& contacts);

///@ingroup Contact
///extracts the IK goals from the stance
void ToIKProblem(const Stance& stance,vector<IKGoal>& constraints);

} //namespace Klampt

#endif
