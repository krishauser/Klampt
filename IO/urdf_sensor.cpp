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

/* Author: John Hsu */


#include "urdf_sensor.h"
#include <fstream>
#include <sstream>
#include <KrisLibrary/utils/AnyValue.h>
#include <algorithm>
#include <tinyxml.h>

namespace urdf{

bool parsePose(Pose &pose, TiXmlElement* xml);

bool parseCamera(Camera &camera, TiXmlElement* config)
{
  camera.clear();
  camera.type = VisualSensor::CAMERA;

  TiXmlElement *image = config->FirstChildElement("image");
  if (image)
  {
    const char* width_char = image->Attribute("width");
    if (width_char)
    {
      if(!LexicalCast(width_char,camera.width)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Camera image width ["<< width_char<<"] is not a valid int");
        return false;
      }
    }
    else
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"Camera sensor needs an image width attribute ");
      return false;
    }

    const char* height_char = image->Attribute("height");
    if (height_char)
    {
      if(!LexicalCast(height_char,camera.height)) 
      {
	       LOG4CXX_INFO(KrisLibrary::logger(),"Camera image height ["<< height_char<<"] is not a valid int");
        return false;
      }
    }
    else
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"Camera sensor needs an image height attribute ");
      return false;
    }

    const char* format_char = image->Attribute("format");
    if (format_char)
      camera.format = std::string(format_char);
    else
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"Camera sensor needs an image format attribute ");
      return false;
    }    

    const char* hfov_char = image->Attribute("hfov");
    if (hfov_char)
    {
      if(!LexicalCast(hfov_char,camera.hfov)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Camera image hfov ["<< hfov_char<<"] is not a valid float");
        return false;
      }
    }
    else
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"Camera sensor needs an image hfov attribute ");
      return false;
    }

    const char* near_char = image->Attribute("near");
    if (near_char)
    {
      if(!LexicalCast(near_char,camera.near)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Camera image near ["<< near_char<<"] is not a valid float: ");
        return false;
      }
    }
    else
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"Camera sensor needs an image near attribute ");
      return false;
    }

    const char* far_char = image->Attribute("far");
    if (far_char)
    {
      if(!LexicalCast(far_char,camera.far)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Camera image far ["<< far_char<<"] is not a valid float");
        return false;
      }
    }
    else
    {
      LOG4CXX_INFO(KrisLibrary::logger(),"Camera sensor needs an image far attribute ");
      return false;
    }
    
  }
  else
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"Camera sensor has no <image> element ");
    return false;
  }
  return true;
}

bool parseRay(Ray &ray, TiXmlElement* config)
{
  ray.clear();
  ray.type = VisualSensor::RAY;

  TiXmlElement *horizontal = config->FirstChildElement("horizontal");
  if (horizontal)
  {
    const char* samples_char = horizontal->Attribute("samples");
    if (samples_char)
    {
      if(!LexicalCast(samples_char,ray.horizontal_samples)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Ray horizontal samples ["<< samples_char<<"] is not a valid float");
        return false;
      }
    }

    const char* resolution_char = horizontal->Attribute("resolution");
    if (resolution_char)
    {
      if(!LexicalCast(resolution_char,ray.horizontal_resolution)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Ray horizontal resolution ["<< resolution_char<<"] is not a valid float");
        return false;
      }
    }   
    
    const char* min_angle_char = horizontal->Attribute("min_angle");
    if (min_angle_char)
    {
      if(!LexicalCast(min_angle_char,ray.horizontal_min_angle)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Ray horizontal min_angle ["<< min_angle_char<<"] is not a valid float");
        return false;
      }
    }

    const char* max_angle_char = horizontal->Attribute("max_angle");
    if (max_angle_char)
    {
      if(!LexicalCast(max_angle_char,ray.horizontal_max_angle)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Ray horizontal max_angle ["<< max_angle_char<<"] is not a valid float");
        return false;
      }
    }
  }
  
  TiXmlElement *vertical = config->FirstChildElement("vertical");
  if (vertical)
  {
    const char* samples_char = vertical->Attribute("samples");
    if (samples_char)
    {
      if(!LexicalCast(samples_char,ray.vertical_samples)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Ray vertical samples ["<< samples_char<<"] is not a valid float");
        return false;
      }
    }

    const char* resolution_char = vertical->Attribute("resolution");
    if (resolution_char)
    {
      if(!LexicalCast(resolution_char,ray.vertical_resolution)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Ray vertical resolution ["<< resolution_char<<"] is not a valid float");
        return false;
      }
    }   
    
    const char* min_angle_char = vertical->Attribute("min_angle");
    if (min_angle_char)
    {
      if(!LexicalCast(min_angle_char,ray.vertical_min_angle)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Ray vertical min_angle ["<< min_angle_char<<"] is not a valid float");
        return false;
      }
    }

    const char* max_angle_char = vertical->Attribute("max_angle");
    if (max_angle_char)
    {
      if(!LexicalCast(max_angle_char,ray.vertical_max_angle)) 
      {
        LOG4CXX_INFO(KrisLibrary::logger(),"Ray vertical max_angle ["<< max_angle_char<<"] is not a valid float");
        return false;
      }
    }
  }
  return true;
}

std::shared_ptr<VisualSensor> parseVisualSensor(TiXmlElement *g)
{
  std::shared_ptr<VisualSensor> visual_sensor;

  // get sensor type
  TiXmlElement *sensor_xml;
  if (g->FirstChildElement("camera"))
  {
    Camera *camera = new Camera();
    visual_sensor.reset(camera);
    sensor_xml = g->FirstChildElement("camera");
    if (!parseCamera(*camera, sensor_xml))
      visual_sensor.reset();
  }
  else if (g->FirstChildElement("ray"))
  {
    Ray *ray = new Ray();
    visual_sensor.reset(ray);
    sensor_xml = g->FirstChildElement("ray");
    if (!parseRay(*ray, sensor_xml))
      visual_sensor.reset();
  }
  else
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"No known sensor types [camera|ray] defined in <sensor> block ");
  }
  return visual_sensor;
}


bool parseSensor(Sensor &sensor, TiXmlElement* config)
{
  sensor.clear();

  const char *name_char = config->Attribute("name");
  if (!name_char)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"No name given for the sensor. ");
    return false;
  }
  sensor.name = std::string(name_char);

  // parse parent_link_name
  const char *parent_link_name_char = config->Attribute("parent_link_name");
  if (!parent_link_name_char)
  {
    LOG4CXX_INFO(KrisLibrary::logger(),"No parent_link_name given for the sensor. ");
    return false;
  }
  sensor.parent_link_name = std::string(parent_link_name_char);

  // parse origin
  TiXmlElement *o = config->FirstChildElement("origin");
  if (o)
  {
    if (!parsePose(sensor.origin, o))
      return false;
  }

  // parse sensor
  sensor.sensor = parseVisualSensor(config);
  return true;
}


}


