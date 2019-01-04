#ifndef NUMERICAL_CSPACE_CONSTRAINT_H
#define NUMERICAL_CSPACE_CONSTRAINT_H

#include <KrisLibrary/math/InequalityConstraint.h>
#include <KrisLibrary/robotics/Stability.h>
#include <KrisLibrary/robotics/TorqueSolver.h>
#include "DistanceQuery.h"
#include <Klampt/Modeling/Robot.h>
#include <KrisLibrary/utils/ArrayMapping.h>

/** @ingroup Continuous
 * @file NumericalConstraint.h
 * @brief Formulates the robot constraints as mathematical inequalities
 * D(q) >= 0
 */

/** @ingroup Continuous  @brief Joint limit inequality */
struct JointLimitConstraint : public LimitConstraint
{
  JointLimitConstraint(const Robot& _robot)
    :LimitConstraint(_robot.qMin,_robot.qMax),robot(_robot)
  {}
  virtual string Label() const;
  virtual string Label(int i) const;

  const Robot& robot;
};

/** @ingroup Continuous  @brief Support polygon inequality */
struct SuppPolyConstraint : public InequalityConstraint
{
  SuppPolyConstraint(Robot& robot, SupportPolygon& sp);
  virtual string Label() const;
  virtual int NumDimensions() const;
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x,Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  //saves by considering constraints dependent
  virtual Real Margin(const Vector& x,int& minConstraint);
  virtual bool Satisfies_i(const Vector& x,int i,Real d=Zero);

  Robot& robot;
  SupportPolygon& sp;
  LinearConstraint cmInequality;
  Matrix A;
  Vector b;

  //temp
  Vector vcom;
  DirtyData<Matrix> Jcom;
  DirtyData<Matrix> Hcomx,Hcomy,Hcomz;
};

/** @ingroup Continuous  @brief Environment collision inequality */
struct CollisionConstraint : public InequalityConstraint
{
  CollisionConstraint(Robot& robot, Geometry::AnyCollisionGeometry3D& geom);
  virtual string Label() const;
  virtual string Label(int i) const;
  virtual int NumDimensions() const {
//	  return this->activeDofs.Size();
//	  cout << "calling NumDimensions" << endl;
//	  getchar();
	  return robot.links.size();
  }
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x, Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  //virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  virtual bool Satisfies_i(const Vector& x,int i,Real d=Zero);

  Robot& robot;
  Geometry::AnyCollisionGeometry3D& geometry;
  vector<DistanceQuery> query;
  ArrayMapping activeDofs;
};

/** @ingroup Continuous  @brief Self collision inequality */
struct SelfCollisionConstraint : public InequalityConstraint
{
  SelfCollisionConstraint(Robot& robot);
  virtual string Label() const;
  virtual string Label(int i) const;
  virtual int NumDimensions() const { return (int)collisionPairs.size(); }
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x, Vector& v);
  virtual Real Eval_i(const Vector& x,int i);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  //virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi);

  virtual bool Satisfies_i(const Vector& x,int i,Real d=Zero);

  Robot& robot;
  vector<pair<int,int> > collisionPairs;
  vector<DistanceQuery> query;
};

/** @ingroup Continuous  @brief Torque limit inequality */
struct TorqueLimitConstraint : public InequalityConstraint
{
  TorqueLimitConstraint(TorqueSolver& solver);
  virtual string Label() const;
  virtual int NumDimensions() const;
  virtual void PreEval(const Vector& x);
  virtual void Eval(const Vector& x,Vector& v);

  TorqueSolver& solver;
};

#endif
