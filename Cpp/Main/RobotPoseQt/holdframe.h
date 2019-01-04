#ifndef HOLDFRAME_H
#define HOLDFRAME_H

#include "Modeling/Resources.h"
#include "resourceframe.h"
#include <QGroupBox>

namespace Ui {
class HoldFrame;
}

class HoldFrame : public QGroupBox
{
  Q_OBJECT
    
public:
  explicit HoldFrame(ResourceFrame* frame,QWidget *parent = 0);
  virtual ~HoldFrame();
  void set(ResourceBase* r) { set(dynamic_cast<HoldResource*>(r)); }
  void set(HoldResource* r);
  HoldResource* resource;
  ResourceFrame* frame;

public slots:
  void Clean();
private:
  Ui::HoldFrame* ui;
};

#endif // HOLDFRAME_H
