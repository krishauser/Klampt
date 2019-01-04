#ifndef PATHFRAME_H
#define PATHFRAME_H

#include "Modeling/Resources.h"
#include "resourceframe.h"
#include <QGroupBox>

namespace Ui {
class PathFrame;
}

class PathFrame : public QGroupBox
{
  Q_OBJECT
    
public:
  explicit PathFrame(ResourceFrame* frame,QWidget *parent = 0);
  virtual ~PathFrame();
  void set(ResourceBase* r) {
    if(dynamic_cast<MultiPathResource*>(r) != NULL)
      set(dynamic_cast<MultiPathResource*>(r));
    else
      set(dynamic_cast<LinearPathResource*>(r));
  }
  void set(MultiPathResource* r);
  void set(LinearPathResource* r);
  ResourceFrame* frame;
  MultiPathResource* multipath;
  LinearPathResource* linearpath;

public slots:
  void Split();
  void Discretize();
  void Optimize();
  void SetDuration();
private:
  Ui::PathFrame* ui;
};

#endif // PATHFRAME_H
