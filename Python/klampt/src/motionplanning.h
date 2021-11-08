#ifndef MOTIONPLANNING_H
#define MOTIONPLANNING_H

#include <string>
#include <vector>

// Forward declaration of C-type PyObject
struct _object;
typedef _object PyObject;

/// Sets the random seed used by the motion planner
void set_random_seed(int seed);

/// Loads planner values from a JSON string
void set_plan_json_string(const char* string);
/// Saves planner values to a JSON string
std::string get_plan_json_string();

/** @brief Sets the planner type.
 *
 * Valid values are
 * - prm: the Probabilistic Roadmap algorithm
 * - rrt: the Rapidly Exploring Random Trees algorithm
 * - sbl: the Single-Query Bidirectional Lazy planner
 * - sblprt: the probabilistic roadmap of trees (PRT) algorithm with SBL as the inter-root planner.
 * - rrt*: the RRT* algorithm for optimal motion planning 
 * - prm*: the PRM* algorithm for optimal motion planning
 * - lazyprm*: the Lazy-PRM* algorithm for optimal motion planning
 * - lazyrrg*: the Lazy-RRG* algorithm for optimal motion planning
 * - fmm: the fast marching method algorithm for resolution-complete optimal motion planning
 * - fmm*: an anytime fast marching method algorithm for optimal motion planning
 */
void set_plan_type(const char* type);

void set_plan_setting(const char* setting,double value);
/** @brief Sets a numeric or string-valued setting for the planner.
 *
 * Valid numeric values are:
 * - "knn": k value for the k-nearest neighbor connection strategy (only for
 *   PRM)
 * - "connectionThreshold": a milestone connection threshold
 * - "perturbationRadius": (for RRT and SBL)
 * - "bidirectional": 1 if bidirectional planning is requested (for RRT)
 * - "grid": 1 if a point selection grid should be used (for SBL)
 * - "gridResolution": resolution for the grid, if the grid should be used 
 *   (for SBL with grid, FMM, FMM*)
 * - "suboptimalityFactor": allowable suboptimality (for RRT*, lazy PRM*,
 *   lazy RRG*)
 * - "randomizeFrequency": a grid randomization frequency (for SBL)
 * - "shortcut": nonzero if you wish to perform shortcutting after a first
 *   plan is found.
 * - "restart": nonzero if you wish to restart the planner to get better
 *   paths with the remaining time.
 * 
 * Valid string values are:
 * - "pointLocation": a string designating a point location data structure.
 *   "kdtree" is supported, optionally followed by a weight vector (for
 *   PRM, RRT*, PRM*, LazyPRM*, LazyRRG*)
 * - "restartTermCond": used if the "restart" setting is true.  This is a
 *   JSON string defining the termination condition.
 *
 *   The default value is "{foundSolution:1;maxIters:1000}", which indicates
 *   that the planner will restart if it has found a solution, or 1000
 *   iterations have passed.
 *
 *   To restart after a certain amount of time has elasped, use
 *   "{timeLimit:X}".  If you are using an optimizing planner, e.g.,
 *   shortcutting, you should set foundSolution:0.
 */
void set_plan_setting(const char* setting,const char* value);

///Performs cleanup of all created spaces and planners
void destroy();

/** @brief A raw interface for a configuration space.  
 * 
 * ..note::
 * 
 *     The native Python CSpace interface class in cspace.py is easier to use.
 *
 * You can either set a single feasibility test function using setFeasibility()
 * or add several feasibility tests, all of which need to be satisfied, using
 * addFeasibilityTest().  In the latter case, planners may be able to provide
 * debugging statistics, solve Minimum Constraint Removal problems, run faster
 * by eliminating constraint tests, etc.
 *
 * Either setVisibility() or setVisibilityEpsilon() must be called to define
 * a visibility checker between two (feasible) configurations.  In the 
 * latter case, the path will be discretized at the resolution sent to
 * setVisibilityEpsilon.  If you have special single-constraint visibility
 * tests, you can call that using addVisibilityTest (for example, for convex
 * constraints you can set it to the lambda function that returns true
 * regardless of its arguments).
 *
 * Supported properties include "euclidean" (boolean), "metric" (string),
 * "geodesic" (boolean), "minimum" (vector), and "maximum" (vector). 
 * These may be used by planners to make planning faster or more accurate.
 * For a complete list see KrisLibrary/planning/CSpace.h.
 */
class CSpaceInterface
{
 public:
  CSpaceInterface();
  CSpaceInterface(const CSpaceInterface&);  //copy constructor
  ~CSpaceInterface();
  void destroy();
  void setFeasibility(PyObject* pyFeas);
  void addFeasibilityTest(const char* name,PyObject* pyFeas);
  void setVisibility(PyObject* pyVisible);
  void addVisibilityTest(const char* name,PyObject* pyVisible);
  void setVisibilityEpsilon(double eps);
  void setSampler(PyObject* pySamp);
  void setNeighborhoodSampler(PyObject* pySamp);
  void setDistance(PyObject* pyDist);
  void setInterpolate(PyObject* pyInterp);
  void setProperty(const char* key,const char* value);
  const char* getProperty(const char* key);

  ///Queries whether a given configuration is feasible 
  bool isFeasible(PyObject* q);
  ///Queries whether two configurations are visible
  bool isVisible(PyObject* a,PyObject* b);
  ///Queries whether a given configuration is feasible with respect to a given constraint
  bool testFeasibility(const char* name,PyObject* q);
  ///Queries whether two configurations are visible with respect to a given constraint
  bool testVisibility(const char* name,PyObject* a,PyObject* b);
  ///Returns a list of all failed feasibility constraints
  PyObject* feasibilityFailures(PyObject* q);
  ///Returns a list of all failed visibility constraints
  PyObject* visibilityFailures(PyObject* a,PyObject* b);
  ///Samples a configuration
  PyObject* sample();
  ///Returns the distance between two configurations
  double distance(PyObject* a,PyObject* b);
  ///Interpolates between two configurations
  PyObject* interpolate(PyObject* a,PyObject* b,double u);

  ///optional: adaptive queries can be used to automatically minimize the total
  ///cost of testing feasibility / visibility using empirical estimates.  Off by default.
  bool adaptiveQueriesEnabled();
  ///Call this to enable adaptive queries.  (It has a small overhead.)
  void enableAdaptiveQueries(bool enabled=true);
  ///Call this to optimize the feasibility / visibility testing order.
  void optimizeQueryOrder();
  ///Marks that a certain feasibility test must be performed before another
  void setFeasibilityDependency(const char* name,const char* precedingTest);
  ///Resets the data for a certain feasibility test.  Default values give a data-gathering behavior
  void setFeasibilityPrior(const char* name,double costPrior=0.0,double feasibilityProbability=0.0,double evidenceStrength=1.0);
  ///Marks that a certain feasibility test must be performed before another
  void setVisibilityDependency(const char* name,const char* precedingTest);
  ///Resets the data for a certain visibility test.  Default values give a data-gathering behavior
  void setVisibilityPrior(const char* name,double costPrior=0.0,double visibilityProbability=0.0,double evidenceStrength=1.0);
  ///Retrieves the empirical average cost of a given feasibility test 
  double feasibilityCost(const char* name);
  ///Retrieves the empirical average success rate of a given feasibility test
  double feasibilityProbability(const char* name);
  ///Retrieves the empirical average cost of a given visibility test
  double visibilityCost(const char* name);
  ///Retrieves the empirical average success rate of a given visibility test
  double visibilityProbability(const char* name);
  ///Retrieves the current order of feasibility tests
  PyObject* feasibilityQueryOrder();
  ///Retrieves the current order of visibility tests
  PyObject* visibilityQueryOrder();

  ///Returns constraint testing statistics.
  ///If adaptive queries are enabled, this returns the stats on each constraint
  PyObject* getStats();

  int index;
};

/** @brief An interface for a kinematic motion planner.  The :class:`MotionPlan`
 * interface in cspace.py is somewhat easier to use.
 *
 * On construction, uses the planner type specified by setPlanType
 * and the settings currently specified by calls to setPlanSetting.
 *
 * Point-to-point planning is enabled by sending two configurations to
 * the setEndpoints method.
 * This is mandatory for RRT and SBL-style planners.  The start and end
 * milestones are given by indices 0 and 1, respectively
 *
 * Point-to-set planning is enabled by sending a *goal test* as
 * the second argument to the setEndpoints method.  It is possible also
 * to send a special goal sampler by providing a *pair of functions* as the
 * second argument consisting of the two functions (goaltest,goalsample).
 * The first in this pair  tests whether a configuration is a goal, and
 * the second returns a sampled configuration in a superset of the goal.
 * Ideally the goal sampler generates as many goals as possible.
 * 
 * To plan, call planMore(iters) until getPath(0,1) returns non-NULL.
 * The return value is a list of configurations.
 *
 * Some planners can be used multi-query mode (such as PRM).  In 
 * multi-query mode, you may call addMilestone(q) to add a new milestone.
 * addMilestone() returns the index of that milestone, which can be used
 * in later calls to getPath().
 *
 * In point-to-set mode, getSolutionPath will return the optimal path to 
 * any goal milestone.
 *
 * All planners work with the standard path-length objective function.
 * Some planners can work with other cost functions, and you can use
 * setCostFunction to set the edge / terminal costs. Usually, the results
 * will only be optimal on the computed graph, and the graph is not
 * specifically computed to optimize that cost.
 *
 * To get a roadmap (V,E), call getRoadmap().  V is a list of configurations
 * (each configuration is a Python list) and E is a list of edges (each edge is
 * a pair (i,j) indexing into V).
 * 
 * To dump the roadmap to disk, call dump(fn).  This saves to a
 * Trivial Graph Format (TGF) format.
 */
class PlannerInterface
{
 public:
  PlannerInterface(const CSpaceInterface& cspace);
  ~PlannerInterface();
  void destroy();
  bool setEndpoints(PyObject* start,PyObject* goal);
  bool setEndpointSet(PyObject* start,PyObject* goal,PyObject* goalSample=NULL);
  void setCostFunction(PyObject* edgeCost=NULL,PyObject* terminalCost=NULL);
  int addMilestone(PyObject* milestone);
  int getClosestMilestone(PyObject* config);
  PyObject* getMilestone(int);
  void planMore(int iterations);
  PyObject* getSolutionPath();
  PyObject* getPath(int milestone1,int milestone2);
  PyObject* getPath(int milestone1,const std::vector<int>& goalMilestones);
  double getData(const char* setting);
  PyObject* getStats();
  PyObject* getRoadmap();
  void dump(const char* fn);

  int index;
  int spaceIndex;
};

#endif
