#ifndef MOTIONPLANNING_H
#define MOTIONPLANNING_H

// Forward declaration of C-type PyObject
struct _object;
typedef _object PyObject;

/// Sets the random seed used by the motion planner
void setRandomSeed(int seed);

/** @brief Sets the planner type.
 *
 * Valid values are
 * - "prm": Probabilistic roadmap
 * - "rrt": Rapidly-exploring Random Trees
 * - "sbl": The SBL (single-query, bidirectional, lazy) planner
 */
void setPlanType(const char* type);

/** @brief Sets a setting for the planner.
 *
 * Valid values are
 * - "knn": k value for the k-nearest neighbor connection strategy (only for
 *   PRM)
 * - "connectionThreshold": a milestone connection threshold
 * - "perturbationRadius": (only for RRT and SBL)
 * - "bidirectional": 1 if bidirectional planning is requested (only for RRT)
 * - "grid": 1 if a point selection grid should be used (only for SBL)
 * - "gridResolution": resolution for the grid, if the grid should be used
 * - "randomizeFrequency": a grid randomization frequency (only for SBL)
 */
void setPlanSetting(const char* setting,double value);

///Performs cleanup of all created spaces and planners
void destroy();

/** @brief A raw interface for a configuration space.  The CSpace interface
 * in cspace.py is easier to use.
 */
class CSpaceInterface
{
 public:
  CSpaceInterface();
  CSpaceInterface(const CSpaceInterface&);  //copy constructor
  ~CSpaceInterface();
  void destroy();
  void setFeasibility(PyObject* pyFeas);
  void setVisibility(PyObject* pyVisible);
  void setVisibilityEpsilon(double eps);
  void setSampler(PyObject* pySamp);
  void setNeighborhoodSampler(PyObject* pySamp);
  void setDistance(PyObject* pyDist);
  void setInterpolate(PyObject* pyInterp);

  int index;
};

/** @brief An interface for a motion planner.  The MotionPlanner interface
 * in cspace.py is somewhat easier to use.
 *
 * On construction, uses the planner type specified by setPlanType
 * and the settings currently specified by calls to setPlanSetting.
 *
 * Point-to-point planning is enabled using the setEndpoints method.
 * This is mandatory for RRT and SBL planners.  The start and end
 * milestones are given by indices 0 and 1, respectively
 *
 * PRM can be used in either point-to-point or multi-query mode.  In 
 * multi-query mode, you may call addMilestone(q) to add a new milestone.
 * addMilestone() returns the index of that milestone, which can be used
 * in later calls to getPath().
 * 
 * To plan, call planMore(iters) until getPath(0,1) returns non-NULL.
 * The return value is a list of configurations.
 *
 * To get a roadmap dump, call dump(fn).  This saves to a
 * Trivial Graph Format (TGF) format.
 */
class PlannerInterface
{
 public:
  PlannerInterface(const CSpaceInterface& cspace);
  ~PlannerInterface();
  void destroy();
  bool setEndpoints(PyObject* start,PyObject* goal);
  int addMilestone(PyObject* milestone);
  void planMore(int iterations);
  PyObject* getPathEndpoints();
  PyObject* getPath(int milestone1,int milestone2);
  double getData(const char* setting);
  void dump(const char* fn);

  int index;
};

#endif
