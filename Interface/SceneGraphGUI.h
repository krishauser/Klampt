
/* 
class GenericSceneGraphBackendBase : public GenericBackendBase
{
  bool SendNodeAdd(const string& name,const string& parent);
  bool SendNodeDelete(const string& name);
  bool SendNodeAppearance(const string& name,const string& item,const AnyValue& value);
  bool SendNodeTransform(const string& name,const Math3D::Matrix4& mat);
  bool SendNodeColor(const string& name,const Math3D::Vector4& color);
  bool SendNodeGeometry(const string& name,const string& data);
  bool SendNodeGeometryFile(const string& name,const string& file);
};

class GenericSceneGraphGUIBase : public GenericGUIBase
{
 public:
  struct SceneGraphNode
  {
    string name;
    string parent;
    Geometry::AnyGeometry3D geometry;
    Math3D::Matrix4 T;    //current transform
    GLDraw::GeometryAppearance appearance;
  };

  GenericSceneGraphGUIBase();
  void DrawGL();
  void DrawScreenGL();
  virtual bool ProcessMessage(const AnyCollection& msg);
  virtual bool OnNodeAdd(const string& name,const string& parent);
  virtual bool OnNodeDelete(const string& name);
  virtual bool OnNodeAppearance(const string& name,const string& item,const string& value);
  virtual bool OnNodeTransform(const string& name,const Matrix3& mat);
  virtual bool OnNodeColor(const string& name,const GLColor& color);
  virtual bool OnNodeGeometry(const string& name,const string& data);
  virtual bool OnNodeGeometryFile(const string& name,const string& file);

  map<string,SceneGraphNode> sceneGraph;
  vector<ConsolePrintout>
};

*/



/** 
bool GenericSceneGraphBackendBase::SendNodeAdd(const string& name,const string& parent)
{
  AnyCollection msg;
  msg["type"] = string("node_add");
  msg["name"] = name;
  msg["parent"] = parent;
  return SendMessage(msg);
}

bool GenericSceneGraphBackendBase::SendNodeDelete(const string& name)
{
  AnyCollection msg;
  msg["type"] = string("node_delete");
  msg["name"] = name;
  return SendMessage(msg);
}

bool GenericSceneGraphBackendBase::SendNodeAppearance(const string& name,const string& item,const AnyValue& value)
{
  AnyCollection msg;
  msg["type"] = string("node_appearance");
  msg["item"] = item;
  msg["value"] = value;
  return SendMessage(msg);
}

bool GenericSceneGraphBackendBase::SendNodeTransform(const string& name,const Math3D::Matrix4& mat)
{
  AnyCollection msg;
  msg["type"] = string("node_transform");
  msg["name"] = name;
  msg["mat"].resize(16);
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      msg["mat"][i*4+j] = mat(i,j);
  return SendMessage(msg);
}

bool GenericSceneGraphBackendBase::SendNodeColor(const string& name,const Math3D::Vector4& color)
{
  AnyCollection msg;
  msg["type"] = string("node_color");
  msg["name"] = name;
  msg["color"].resize(4);
  msg["color"][0] = color.x;
  msg["color"][1] = color.y;
  msg["color"][2] = color.z;
  msg["color"][3] = color.w;
  return SendMessage(msg);
}

bool GenericSceneGraphBackendBase::SendNodeGeometry(const string& name,const string& data)
{
  AnyCollection msg;
  msg["type"] = string("mouse_click");
  msg["name"] = name;
  msg["data"] = data;
  return SendMessage(msg);
}

bool GenericSceneGraphBackendBase::SendNodeGeometryFile(const string& name,const string& file)
{
  AnyCollection msg;
  msg["type"] = string("mouse_click");
  msg["name"] = name;
  msg["file"] = file;
  return SendMessage(msg);
}
*/

/*
bool GenericSceneGraphGUIBase::ProcessMessage(const AnyCollection& msg)
{
  string type = msg.get("type",string("error"));
  if(type == "node_add") {
    return OnNodeAdd(msg["name"],msg["parent"]);
  }
  else if(type == "node_delete") {
    return OnNodeDelete(msg["name"]);
  }
  else if(type == "node_appearance") {
    return OnNodeAppearance(msg["name"],msg["item"],msg["value"]);
  }
  //virtual bool OnNodeTransform(const string& name,const Math3D::RigidTransform& mat) { return false; }
  //virtual bool OnNodeColor(const string& name,const Math3D::Vector4& color) { return false; }
  else if(type == "node_geometry") {
    return OnNodeGeometry(msg["name"],msg["data"]);
  }
  else if(type == "node_geometry_file") {
    return OnNodeGeometryFile(msg["name"],msg["file"]);
  }
  return GenericGUIBase::ProcessMessage(msg);
}
*/
