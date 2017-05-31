/*********************************************************************
* Software License Agreement (BSD License)
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

/* Author: Wim Meeussen */
#include <log4cxx/logger.h>
#include <KrisLibrary/Logger.h>
#include <boost/algorithm/string.hpp>
#include <vector>
#include "urdf_parser.h"

//const bool debug = true;
const bool debug = false;

namespace urdf{

bool parseMaterial(Material &material, TiXmlElement *config);
bool parseLink(Link &link, TiXmlElement *config);
bool parseJoint(Joint &joint, TiXmlElement *config);

// Added by achq on 2012/10/13 *******************//

  /**
    * @function isObjectURDF
    */
  bool isObjectURDF( const std::string &_xml_string ) {  
    TiXmlDocument xml_doc;
    xml_doc.Parse( _xml_string.c_str() );

    TiXmlElement *test_xml = xml_doc.FirstChildElement("object");
    if ( !test_xml ) { return false; }
    else { return true; }
  }

  /**
    * @function isRobotURDF
    */
  bool isRobotURDF( const std::string &_xml_string ) {
    TiXmlDocument xml_doc;
    xml_doc.Parse( _xml_string.c_str() );

    TiXmlElement *test_xml = xml_doc.FirstChildElement("robot");
    if ( !test_xml ) { return false; }
    else { return true; }
  }

// *************************************************//

boost::shared_ptr<ModelInterface>  parseURDF(const std::string &xml_string)
{

  boost::shared_ptr<ModelInterface> model(new ModelInterface);
  model->clear();

  TiXmlDocument xml_doc;
  xml_doc.LoadFile(xml_string.c_str());
  //xml_doc.Parse(xml_string.c_str());

  TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
  if (!robot_xml)
  { 
    // Added the object if by achq for DART - GRIP (2012/10/13
    // all the rest is the same as original
    robot_xml = xml_doc.FirstChildElement("object");
    if( !robot_xml ) {
      if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"Could find neither a robot nor an object element in the xml file \n" );
      model.reset();
      return model;
    }
    else {
      if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"Found an object file in the xml file!  \n");
    }
  }
  else {
    if(debug) LOG4CXX_INFO(KrisLibrary::logger()," Found a robot object in urdf file! \n");
  }

  // Get robot name
  const char *name = robot_xml->Attribute("name");
  if (!name)
  {
    if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"No name given for the robot. \n");
    model.reset();
    return model;
  }
  model->name_ = std::string(name);

  // Get all Material elements
  for (TiXmlElement* material_xml = robot_xml->FirstChildElement("material"); material_xml; material_xml = material_xml->NextSiblingElement("material"))
  {
    boost::shared_ptr<Material> material;
    material.reset(new Material);

    try {
      parseMaterial(*material, material_xml);
      if (model->getMaterial(material->name))
      {
        if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"material '"<<material->name.c_str() <<"' is not unique. \n" );
        material.reset();
        model.reset();
        return model;
      }
      else
      {
        model->materials_.insert(make_pair(material->name,material));
        if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"successfully added a new material '"<< material->name.c_str() <<"' \n");
      }
    }
    catch (ParseError &e) {
      if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"material xml is not initialized correctly \n");
      material.reset();
      model.reset();
      return model;
    }
  }

  // Get all Link elements
  for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
  {
    boost::shared_ptr<Link> link;
    link.reset(new Link);

    try {
      parseLink(*link, link_xml);
      if (model->getLink(link->name))
      {
        if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"link '"<< link->name.c_str() <<"' is not unique. \n");
        model.reset();
        return model;
      }
      else
      {
        // set link visual material
        if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"setting link '" <<link->name.c_str() << "' material \n" );
        if (link->visual)
        {
          if (!link->visual->material_name.empty())
          {
            if (model->getMaterial(link->visual->material_name))
            {
              if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"setting link '"<<link->name.c_str() << "' material to '"<< link->visual->material_name.c_str() <<"' \n");
              link->visual->material = model->getMaterial( link->visual->material_name.c_str() );
            }
            else
            {
              if (link->visual->material)
              {
                if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"link '"<<link->name.c_str() <<"' material '"<< link->visual->material_name.c_str() <<"' defined in Visual. \n");
                model->materials_.insert(make_pair(link->visual->material->name,link->visual->material));
              }
              else
              {
                if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"link '"<< link->name.c_str()<<"' material '"<<link->visual->material_name.c_str() <<"' undefined. \n");
                model.reset();
                return model;
              }
            }
          }
        }

        model->links_.insert(make_pair(link->name,link));
        if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"successfully added a new link '"<< link->name.c_str() <<"' \n");
      }
    }
    catch (ParseError &e) {
      if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"link xml is not initialized correctly \n");
      model.reset();
      return model;
    }
  }
  if (model->links_.empty()){
    if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"No link elements found in urdf file \n");
    model.reset();
    return model;
  }

  // Get all Joint elements
  for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    boost::shared_ptr<Joint> joint;
    joint.reset(new Joint);

    if (parseJoint(*joint, joint_xml))
    {
      if (model->getJoint(joint->name))
      {
        if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"joint '"<< joint->name.c_str() <<"' is not unique. \n");
        model.reset();
        return model;
      }
      else
      {
        model->joints_.insert(make_pair(joint->name,joint));
        if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"successfully added a new joint '"<< joint->name.c_str()<<"' \n");
      }
    }
    else
    {
      if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"joint xml is not initialized correctly \n");
      model.reset();
      return model;
    }
  }


  // every link has children links and joints, but no parents, so we create a
  // local convenience data structure for keeping child->parent relations
  std::map<std::string, std::string> parent_link_tree;
  parent_link_tree.clear();

  // building tree: name mapping
  try 
  {
    model->initTree(parent_link_tree);
  }
  catch(ParseError &e)
  {
    if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"Failed to build tree: "<< e.what()<<" \n");
    model.reset();
    return model;
  }

  // find the root link
  try
  {
    model->initRoot(parent_link_tree);
  }
  catch(ParseError &e)
  {
    if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"Failed to find root link: "<< e.what() <<"%s \n");
    model.reset();
    return model;
  }
  
  return model;
}

bool exportMaterial(Material &material, TiXmlElement *config);
bool exportLink(Link &link, TiXmlElement *config);
bool exportJoint(Joint &joint, TiXmlElement *config);
TiXmlDocument*  exportURDF(boost::shared_ptr<ModelInterface> &model)
{
  TiXmlDocument *doc = new TiXmlDocument();

  TiXmlElement *robot = new TiXmlElement("robot");
  robot->SetAttribute("name", model->name_);
  doc->LinkEndChild(robot);

  for (std::map<std::string, boost::shared_ptr<Link> >::const_iterator l=model->links_.begin(); l!=model->links_.end(); l++)  
    exportLink(*(l->second), robot);

  for (std::map<std::string, boost::shared_ptr<Joint> >::const_iterator j=model->joints_.begin(); j!=model->joints_.end(); j++)  
  {
    if(debug) LOG4CXX_INFO(KrisLibrary::logger(),"exporting joint ["<<j->second->name.c_str()<<"]\n");
    exportJoint(*(j->second), robot);
  }

  return doc;
}


}

