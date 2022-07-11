#ifndef MULTI_PATH_H
#define MULTI_PATH_H

#include "DynamicPath.h"
#include <Klampt/Contact/Hold.h>
#include <Klampt/Contact/Stance.h>
#include <KrisLibrary/planning/GeneralizedBezierCurve.h>
#include <KrisLibrary/utils/PropertyMap.h>
#include <KrisLibrary/math/vector.h>

class TiXmlElement;

namespace Klampt {
  using namespace std;

/** @ingroup Modeling
 * @brief A very general multi-section path container.
 *
 * Path sections can be time-parameterized, have velocities, and can also
 * have section-specific constraints.
 *
 * A non-time-parameterized path with velocities is assumed to be just a
 * linear interpolation in config x velocity space (q,v).
 *
 * Section-specific constraints are either ikGoals, holds or both. 
 * To save space a multipath may also define holds as indexes into the
 * holdSet data structure.
 *
 * Load/Save to XML files is supported.
 */
class MultiPath
{
 public:
  bool Load(const string& fn);
  bool Save(const string& fn) const;
  bool Load(TiXmlElement* in);
  bool Save(TiXmlElement* out) const;
  size_t NumSections() const { return sections.size(); }
  ///Tests whether the path has a valid set of milestones and constraints
  bool IsValid() const;
  ///Tests whether the milestone, times, and velocities are continuous
  ///within the tolerance tol
  bool IsContinuous(Real tol=0.0) const;
  ///Returns true if the specified section has timing information
  bool HasTiming(int section=0) const;
  ///Returns true if the specified section has velocity  information
  bool HasVelocity(int section=0) const;
  ///Returns true if the specified section has any constraints (ik or holds)
  bool HasConstraints(int section=0) const;
  ///Returns true if the specified section has hold information
  bool HasContacts(int section=0) const;
  ///Returns the set of untimed milestones along the given section
  void GetMilestones(vector<Vector>& milestones,int section=0) const;
  ///Returns the set of timed milestones along the given section.  If the
  ///section is untimed, the times are set uniformly in the range [0,1].
  void GetTimedMilestones(vector<Real>& times,vector<Vector>& milestones,int section=0) const;
  ///Computes a DynamicPath from the given section.  path.velMax and 
  ///path.accMax need to be set.
  bool GetDynamicPath(ParabolicRamp::DynamicPath& path,int section=0) const;
  ///Sets the given section to an untimed path.  Dynamic resizing is supported
  ///if section=sections.size()
  void SetMilestones(const vector<Vector>& milestones,int section=0);
  ///Sets the given section to a timed path.  Note: timing is absolute.
  ///Dynamic resizing is supported if section=sections.size().
  void SetTimedMilestones(const vector<Real>& times,const vector<Vector>& milestones,int section=0);
  ///Sets the given section to a DynamicPath (untimed).
  ///Dynamic resizing is supported if section=sections.size().
  void SetDynamicPath(const ParabolicRamp::DynamicPath& path,int section=0);
  ///Returns the start time of the path.  If untimed, the start time is 0.
  Real StartTime() const;
  ///Returns the end time of the path.  If untimed, the end time is 1.
  Real EndTime() const;
  ///Returns the duration of the path.  If untimed, the duration is 1.
  Real Duration() const;
  ///Sets the duration of the path by scaling.  If untimed, milestone times
  ///are assigned.  If untimed and uniformSectionTimes = false, section
  ///durations are assigned proportional to the number of section milestones.
  ///Otherwise, section durations are uniform
  void SetDuration(Real duration,bool uniformSectionTimes=true);
  ///Sets the timing of the path by smoothly accelerating and decelerating
  ///to a stop between sections.  Existing timing is overwritten.  See
  ///@SetDuration for a description of uniformSectionTimes.
  void SetSmoothTiming(Real duration,bool uniformSectionTimes=true);
  ///Concatenates another multipath.  If this is a timed path, the suffix
  ///path must be timed as well.  In such a case, if relative = true then
  ///timing of the suffix path is shifted forward by time this->Duration()
  ///before concatenating it.
  void Concat(const MultiPath& suffix,bool relative=true);
  ///Retrieves all of the IK goals from the problem, including holds
  void GetIKProblem(vector<IKGoal>& goals,int section=0) const;
  ///Sets the section to just an IK constrained section (holds are deleted).
  ///Dynamic resizing is supported if section=sections.size().
  void SetIKProblem(const vector<IKGoal>& goals,int section=0);
  ///Retrieves the section's stance, including other ikGoal constraints.
  void GetStance(Stance& stance,int section=0) const;
  ///Sets the section to a stance section (ikGoals are deleted).
  ///Dynamic resizing is supported if section=sections.size().
  void SetStance(const Stance& stance,int section=0);
  ///Sets a named global hold
  void SetHold(const string& str,const Hold& h);
  ///Retreives a named global hold
  bool GetHold(const string& str,Hold& h) const;
  ///Returns the section corresponding to the current time.  If untimed, the
  ///time range is [0,1].  Does not support mixed timed and untimed paths.
  int TimeToSection(Real time) const;

  enum InterpPolicy { InterpLinear, InterpCubic };
  ///Generates a hermite interpolator with "natural" tangents if velocities
  ///are not present.  Returns the section index
  int Evaluate(Real time,GeneralizedCubicBezierCurve& curve,Real& duration,Real& u,InterpPolicy policy=InterpCubic) const;
  int Evaluate(Real time,Vector& q,InterpPolicy policy=InterpCubic) const;
  int Evaluate(Real time,Vector& q,Vector& v,InterpPolicy policy=InterpCubic) const;

  struct PathSection 
  {
    PropertyMap settings;

    //section-specific constraints
    vector<IKGoal> ikGoals;
    vector<Hold> holds;
    vector<int> holdIndices;  //indexes into the holdSet array
    vector<string> holdNames;  //indexes into the holdSet array

    vector<Real> times;
    vector<Vector> milestones;
    vector<Vector> velocities;
  };

  //typically store things like name, robot, scenario, etc
  PropertyMap settings;
  vector<PathSection> sections;
  vector<Hold> holdSet;
  vector<string> holdSetNames;
};

ostream& operator << (ostream& out,const MultiPath& path);

} //namespace Klampt

#endif
