/*********************************************************************
* Software Ligcense Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: John Hsu */

#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include <sstream>
#include "urdf_joint.h"
#include <boost/lexical_cast.hpp>
#include <tinyxml.h>
#include "urdf_parser.h"
#include <stdio.h>
#include <iostream>

namespace urdf{

bool parsePose(Pose &pose, TiXmlElement* xml);

bool parseJointDynamics(JointDynamics &jd, TiXmlElement* config)
{
  jd.clear();

  // Get joint damping
  const char* damping_str = config->Attribute("damping");
  if (damping_str == NULL){
    LOG4CXX_INFO(KrisLibrary::logger(),"joint dynamics: no damping, defaults to 0"<<"\n");
    jd.damping = 0;
  }
  else
  {
    try
    {
      jd.damping = boost::lexical_cast<double>(damping_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"damping value ("<< damping_str<<") is not a float: "<<  e.what()<< "\n");
      return false;
    }
  }

  // Get joint friction
  const char* friction_str = config->Attribute("friction");
  if (friction_str == NULL){
    LOG4CXX_INFO(KrisLibrary::logger(),"joint dynamics: no friction, defaults to 0"<<"\n");
    jd.friction = 0;
  }
  else
  {
    try
    {
      jd.friction = boost::lexical_cast<double>(friction_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"friction value ("<< friction_str<< ") is not a float: "<< e.what() << "\n");
      return false;
    }
  }

  if (damping_str == NULL && friction_str == NULL)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"joint dynamics element specified with no damping and no friction \n");
    return false;
  }
  else{
    //LOG4CXX_INFO(KrisLibrary::logger(),"joint dynamics: damping "<< jd.damping<<" and friction "<< jd.friction);
    return true;
  }
}

bool parseJointLimits(JointLimits &jl, TiXmlElement* config)
{
  jl.clear();

  // Get lower joint limit
  const char* lower_str = config->Attribute("lower");
  if (lower_str == NULL){
    LOG4CXX_INFO(KrisLibrary::logger(),"joint limit: no lower, defaults to 0");
    jl.lower = 0;
  }
  else
  {
    try
    {
      jl.lower = boost::lexical_cast<double>(lower_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"lower value ("<< lower_str<<") is not a float: "<< e.what());
      return false;
    }
  }

  // Get upper joint limit
  const char* upper_str = config->Attribute("upper");
  if (upper_str == NULL){
    LOG4CXX_INFO(KrisLibrary::logger(),"joint limit: no upper, , defaults to 0");
    jl.upper = 0;
  }
  else
  {
    try
    {
      jl.upper = boost::lexical_cast<double>(upper_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"upper value ("<<upper_str<<") is not a float: "<< e.what());
      return false;
    }
  }

  // Get joint effort limit
  const char* effort_str = config->Attribute("effort");
  if (effort_str == NULL){
    LOG4CXX_INFO(KrisLibrary::logger(),"joint limit: no effort");
    return false;
  }
  else
  {
    try
    {
      jl.effort = boost::lexical_cast<double>(effort_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"effort value ("<<effort_str<<") is not a float: "<< e.what());
      return false;
    }
  }

  // Get joint velocity limit
  const char* velocity_str = config->Attribute("velocity");
  if (velocity_str == NULL){
    LOG4CXX_INFO(KrisLibrary::logger(),"joint limit: no velocity \n");
    return false;
  }
  else
  {
    try
    {
      jl.velocity = boost::lexical_cast<double>(velocity_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"velocity value ("<<velocity_str<<") is not a float: "<< e.what());
      return false;
    }
  }

  return true;
}

bool parseJointSafety(JointSafety &js, TiXmlElement* config)
{
  js.clear();

  // Get soft_lower_limit joint limit
  const char* soft_lower_limit_str = config->Attribute("soft_lower_limit");
  if (soft_lower_limit_str == NULL)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"joint safety: no soft_lower_limit, using default value \n");
    js.soft_lower_limit = 0;
  }
  else
  {
    try
    {
      js.soft_lower_limit = boost::lexical_cast<double>(soft_lower_limit_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"soft_lower_limit value ("<<soft_lower_limit_str<<") is not a float: "<< e.what());
      return false;
    }
  }

  // Get soft_upper_limit joint limit
  const char* soft_upper_limit_str = config->Attribute("soft_upper_limit");
  if (soft_upper_limit_str == NULL)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"joint safety: no soft_upper_limit, using default value \n");
    js.soft_upper_limit = 0;
  }
  else
  {
    try
    {
      js.soft_upper_limit = boost::lexical_cast<double>(soft_upper_limit_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"soft_upper_limit value ("<<soft_upper_limit_str<<") is not a float: "<< e.what());
      return false;
    }
  }

  // Get k_position_ safety "position" gain - not exactly position gain
  const char* k_position_str = config->Attribute("k_position");
  if (k_position_str == NULL)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"joint safety: no k_position, using default value \n");
    js.k_position = 0;
  }
  else
  {
    try
    {
      js.k_position = boost::lexical_cast<double>(k_position_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"k_position value ("<<k_position_str<<") is not a float: "<< e.what());
      return false;
    }
  }
  // Get k_velocity_ safety velocity gain
  const char* k_velocity_str = config->Attribute("k_velocity");
  if (k_velocity_str == NULL)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"joint safety: no k_velocity \n");
    return false;
  }
  else
  {
    try
    {
      js.k_velocity = boost::lexical_cast<double>(k_velocity_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"k_velocity value ("<<k_velocity_str<<") is not a float: "<< e.what());
      return false;
    }
  }

  return true;
}

bool parseJointCalibration(JointCalibration &jc, TiXmlElement* config)
{
  jc.clear();

  // Get rising edge position
  const char* rising_position_str = config->Attribute("rising");
  if (rising_position_str == NULL)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"joint calibration: no rising, using default value \n");
    jc.rising.reset();
  }
  else
  {
    try
    {
      jc.rising.reset(new double(boost::lexical_cast<double>(rising_position_str)));
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"risingvalue ("<<rising_position_str<<") is not a float: "<< e.what());
      return false;
    }
  }

  // Get falling edge position
  const char* falling_position_str = config->Attribute("falling");
  if (falling_position_str == NULL)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"joint calibration: no falling, using default value \n");
    jc.falling.reset();
  }
  else
  {
    try
    {
      jc.falling.reset(new double(boost::lexical_cast<double>(falling_position_str)));
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"fallingvalue ("<<falling_position_str<<") is not a float: "<< e.what());
      return false;
    }
  }

  return true;
}

bool parseJointMimic(JointMimic &jm, TiXmlElement* config)
{
  jm.clear();

  // Get name of joint to mimic
  const char* joint_name_str = config->Attribute("joint");

  if (joint_name_str == NULL)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"joint mimic: no mimic joint specified \n");
    return false;
  }
  else
    jm.joint_name = joint_name_str;
  
  // Get mimic multiplier
  const char* multiplier_str = config->Attribute("multiplier");

  if (multiplier_str == NULL)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"joint mimic: no multiplier, using default value of 1 \n");
    jm.multiplier = 1;    
  }
  else
  {
    try
    {
      jm.multiplier = boost::lexical_cast<double>(multiplier_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"multiplier value ("<<multiplier_str<<") is not a float: "<< e.what());
      return false;
    }
  }

  
  // Get mimic offset
  const char* offset_str = config->Attribute("offset");
  if (offset_str == NULL)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"joint mimic: no offset, using default value of 0 \n");
    jm.offset = 0;
  }
  else
  {
    try
    {
      jm.offset = boost::lexical_cast<double>(offset_str);
    }
    catch (boost::bad_lexical_cast &e)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"offset value ("<<offset_str<<") is not a float: "<< e.what());
      return false;
    }
  }

  return true;
}

bool parseJoint(Joint &joint, TiXmlElement* config)
{
  joint.clear();

  // Get Joint Name
  const char *name = config->Attribute("name");
  if (!name)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"unnamed joint found \n");
    return false;
  }
  joint.name = name;

  // Get transform from Parent Link to Joint Frame
  TiXmlElement *origin_xml = config->FirstChildElement("origin");
  if (!origin_xml)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"Joint ["<<joint.name.c_str()<<"] missing origin tag under parent describing transform from Parent Link to Joint Frame, (using Identity transform). \n");
    joint.parent_to_joint_origin_transform.clear();
  }
  else
  {
    if (!parsePose(joint.parent_to_joint_origin_transform, origin_xml))
    {
      joint.parent_to_joint_origin_transform.clear();
      LOG4CXX_INFO(KrisLibrary::logger(),"Malformed parent origin element for joint ["<< joint.name.c_str());
      return false;
    }
  }

  // Get Parent Link
  TiXmlElement *parent_xml = config->FirstChildElement("parent");
  if (parent_xml)
  {
    const char *pname = parent_xml->Attribute("link");
    if (!pname)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"no parent link name specified for Joint link ["<< joint.name.c_str());
    }
    else
    {
      joint.parent_link_name = std::string(pname);
    }
  }

  // Get Child Link
  TiXmlElement *child_xml = config->FirstChildElement("child");
  if (child_xml)
  {
    const char *pname = child_xml->Attribute("link");
    if (!pname)
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"no child link name specified for Joint link ["<< joint.name.c_str());
    }
    else
    {
      joint.child_link_name = std::string(pname);
    }
  }

  // Get Joint type
  const char* type_char = config->Attribute("type");
  if (!type_char)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"joint ["<<joint.name.c_str()<<"] has not type, check to see if it's a reference. \n");    
    return false;
  }
  
  std::string type_str = type_char;
  if (type_str == "planar")
    joint.type = Joint::PLANAR;
  else if (type_str == "floating")
    joint.type = Joint::FLOATING;
  else if (type_str == "revolute")
    joint.type = Joint::REVOLUTE;
  else if (type_str == "continuous")
    joint.type = Joint::CONTINUOUS;
  else if (type_str == "prismatic")
    joint.type = Joint::PRISMATIC;
  else if (type_str == "fixed")
    joint.type = Joint::FIXED;
  else
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"Joint ["<< joint.name.c_str()<<"] has no known type ["<< type_str.c_str());
    return false;
  }

  // Get Joint Axis
  if (joint.type != Joint::FLOATING && joint.type != Joint::FIXED)
  {
    // axis
    TiXmlElement *axis_xml = config->FirstChildElement("axis");
    if (!axis_xml){
      LOG4CXX_INFO(KrisLibrary::logger(),"no axis elemement for Joint link ["<< joint.name.c_str()<<", defaulting to (1,0,0) axis \n");
      joint.axis = Vector3(1.0, 0.0, 0.0);
    }
    else{
      if (axis_xml->Attribute("xyz")){
        try {
          joint.axis.init(axis_xml->Attribute("xyz"));
        }
        catch (ParseError &e) {
          joint.axis.clear();
          LOG4CXX_INFO(KrisLibrary::logger(),"Malformed axis element for joint ["<< joint.name.c_str()<<"]: "<< e.what());
          return false;
        }
      }
    }
  }

  // Get limit
  TiXmlElement *limit_xml = config->FirstChildElement("limit");
  if (limit_xml)
  {
    joint.limits.reset(new JointLimits());
    if (!parseJointLimits(*joint.limits, limit_xml))
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"Could not parse limit element for joint ["<< joint.name.c_str());
      joint.limits.reset();
      return false;
    }
  }
  else if (joint.type == Joint::REVOLUTE)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"Joint ["<< joint.name.c_str());
    return false;
  }
  else if (joint.type == Joint::PRISMATIC)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"Joint ["<< joint.name.c_str()); 
    return false;
  }

  // Get safety
  TiXmlElement *safety_xml = config->FirstChildElement("safety_controller");
  if (safety_xml)
  {
    joint.safety.reset(new JointSafety());
    if (!parseJointSafety(*joint.safety, safety_xml))
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"Could not parse safety element for joint ["<< joint.name.c_str());
      joint.safety.reset();
      return false;
    }
  }

  // Get calibration
  TiXmlElement *calibration_xml = config->FirstChildElement("calibration");
  if (calibration_xml)
  {
    joint.calibration.reset(new JointCalibration());
    if (!parseJointCalibration(*joint.calibration, calibration_xml))
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"Could not parse calibration element for joint  ["<< joint.name.c_str());
      joint.calibration.reset();
      return false;
    }
  }

  // Get Joint Mimic
  TiXmlElement *mimic_xml = config->FirstChildElement("mimic");
  if (mimic_xml)
  {
    joint.mimic.reset(new JointMimic());
    if (!parseJointMimic(*joint.mimic, mimic_xml))
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"Could not parse mimic element for joint  ["<< joint.name.c_str());
      joint.mimic.reset();
      return false;
    }
  }

  // Get Dynamics
  TiXmlElement *prop_xml = config->FirstChildElement("dynamics");
  if (prop_xml)
  {
    joint.dynamics.reset(new JointDynamics());
    if (!parseJointDynamics(*joint.dynamics, prop_xml))
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"Could not parse joint_dynamics element for joint ["<< joint.name.c_str());
      joint.dynamics.reset();
      return false;
    }
  }

  return true;
}


/* exports */
bool exportPose(Pose &pose, TiXmlElement* xml);

bool exportJointDynamics(JointDynamics &jd, TiXmlElement* xml)
{
  TiXmlElement *dynamics_xml = new TiXmlElement("dynamics");
  dynamics_xml->SetAttribute("damping", urdf_export_helpers::values2str(jd.damping) );
  dynamics_xml->SetAttribute("friction", urdf_export_helpers::values2str(jd.friction) );
  xml->LinkEndChild(dynamics_xml);
  return true;
}

bool exportJointLimits(JointLimits &jl, TiXmlElement* xml)
{
  TiXmlElement *limit_xml = new TiXmlElement("limit");
  limit_xml->SetAttribute("effort", urdf_export_helpers::values2str(jl.effort) );
  limit_xml->SetAttribute("velocity", urdf_export_helpers::values2str(jl.velocity) );
  limit_xml->SetAttribute("lower", urdf_export_helpers::values2str(jl.lower) );
  limit_xml->SetAttribute("upper", urdf_export_helpers::values2str(jl.upper) );
  xml->LinkEndChild(limit_xml);
  return true;
}

bool exportJointSafety(JointSafety &js, TiXmlElement* xml)
{
  TiXmlElement *safety_xml = new TiXmlElement("safety_controller");
  safety_xml->SetAttribute("k_position", urdf_export_helpers::values2str(js.k_position) );
  safety_xml->SetAttribute("k_velocity", urdf_export_helpers::values2str(js.k_velocity) );
  safety_xml->SetAttribute("soft_lower_limit", urdf_export_helpers::values2str(js.soft_lower_limit) );
  safety_xml->SetAttribute("soft_upper_limit", urdf_export_helpers::values2str(js.soft_upper_limit) );
  xml->LinkEndChild(safety_xml);
  return true;
}

bool exportJointCalibration(JointCalibration &jc, TiXmlElement* xml)
{
  if (jc.falling || jc.rising)
  {
    TiXmlElement *calibration_xml = new TiXmlElement("calibration");
    if (jc.falling)
      calibration_xml->SetAttribute("falling", urdf_export_helpers::values2str(*jc.falling) );
    if (jc.rising)
      calibration_xml->SetAttribute("rising", urdf_export_helpers::values2str(*jc.rising) );
    //calibration_xml->SetAttribute("reference_position", urdf_export_helpers::values2str(jc.reference_position) );
    xml->LinkEndChild(calibration_xml);
  }
  return true;
}

bool exportJointMimic(JointMimic &jm, TiXmlElement* xml)
{
  if (!jm.joint_name.empty())
  {
    TiXmlElement *mimic_xml = new TiXmlElement("mimic");
    mimic_xml->SetAttribute("offset", urdf_export_helpers::values2str(jm.offset) );
    mimic_xml->SetAttribute("multiplier", urdf_export_helpers::values2str(jm.multiplier) );
    mimic_xml->SetAttribute("joint", jm.joint_name );
    xml->LinkEndChild(mimic_xml);
  }
  return true;
}

bool exportJoint(Joint &joint, TiXmlElement* xml)
{
  TiXmlElement * joint_xml = new TiXmlElement("joint");
  joint_xml->SetAttribute("name", joint.name);
  if (joint.type == urdf::Joint::PLANAR)
    joint_xml->SetAttribute("type", "planar");
  else if (joint.type == urdf::Joint::FLOATING)
    joint_xml->SetAttribute("type", "floating");
  else if (joint.type == urdf::Joint::REVOLUTE)
    joint_xml->SetAttribute("type", "revolute");
  else if (joint.type == urdf::Joint::CONTINUOUS)
    joint_xml->SetAttribute("type", "continuous");
  else if (joint.type == urdf::Joint::PRISMATIC)
    joint_xml->SetAttribute("type", "prismatic");
  else if (joint.type == urdf::Joint::FIXED)
    joint_xml->SetAttribute("type", "fixed");
  else
    LOG4CXX_ERROR(KrisLibrary::logger(),"ERROR:  Joint ["<<joint.name.c_str()<<"] type ["<< joint.type);

  // origin
  exportPose(joint.parent_to_joint_origin_transform, joint_xml);

  // axis
  TiXmlElement * axis_xml = new TiXmlElement("axis");
  axis_xml->SetAttribute("xyz", urdf_export_helpers::values2str(joint.axis));
  joint_xml->LinkEndChild(axis_xml);

  // parent 
  TiXmlElement * parent_xml = new TiXmlElement("parent");
  parent_xml->SetAttribute("link", joint.parent_link_name);
  joint_xml->LinkEndChild(parent_xml);

  // child
  TiXmlElement * child_xml = new TiXmlElement("child");
  child_xml->SetAttribute("link", joint.child_link_name);
  joint_xml->LinkEndChild(child_xml);

  if (joint.dynamics)
    exportJointDynamics(*(joint.dynamics), joint_xml);
  if (joint.limits)
    exportJointLimits(*(joint.limits), joint_xml);
  if (joint.safety)
    exportJointSafety(*(joint.safety), joint_xml);
  if (joint.calibration)
    exportJointCalibration(*(joint.calibration), joint_xml);
  if (joint.mimic)
    exportJointMimic(*(joint.mimic), joint_xml);

  xml->LinkEndChild(joint_xml);
  return true;
}



}
