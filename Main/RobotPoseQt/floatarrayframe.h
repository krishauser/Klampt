#ifndef FLOATARRAYFRAME_H
#define FLOATARRAYFRAME_H

#include "Modeling/Resources.h"
#include "resourceframe.h"
#include <QGroupBox>

namespace Ui {
class FloatArrayFrame;
}

class FloatArrayFrame : public QGroupBox
{
  Q_OBJECT
    
public:
  explicit FloatArrayFrame(ResourceFrame* frame,QWidget *parent = 0);
  virtual ~FloatArrayFrame();
  void set(FloatArrayResource* r);
  void set(ResourceBase* r) { set(dynamic_cast<FloatArrayResource*>(r)); }
  FloatArrayResource* resource;
  ResourceFrame* frame;

public slots:
  void IndexChanged(int);
  void ValueChanged(QString);
  void Insert();
  void Delete();
 private:
  Ui::FloatArrayFrame* ui;
};

#endif // FLOATARRAYFRAME_H
