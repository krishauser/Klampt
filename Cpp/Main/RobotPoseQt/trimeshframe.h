#ifndef TRIMESHFRAME_H
#define TRIMESHFRAME_H

#include "Modeling/Resources.h"
#include "resourceframe.h"
#include <QGroupBox>

namespace Ui {
class TriMeshFrame;
}

class TriMeshFrame : public QGroupBox
{
  Q_OBJECT
    
public:
  explicit TriMeshFrame(ResourceFrame* frame,QWidget *parent = 0);
  virtual ~TriMeshFrame();
  void set(TriMeshResource* r);
  void set(ResourceBase* r) { set(dynamic_cast<TriMeshResource*>(r)); }
  TriMeshResource* resource;
  ResourceFrame* frame;

public slots:
  void Simplify();
 private:
  Ui::TriMeshFrame* ui;
};

#endif // FLOATARRAYFRAME_H
