#ifndef STANCEFRAME_H
#define STANCEFRAME_H

#include "Modeling/Resources.h"
#include "resourceframe.h"
#include <QGroupBox>

namespace Ui {
class StanceFrame;
}

class StanceFrame : public QGroupBox
{
  Q_OBJECT
    
public:
  explicit StanceFrame(ResourceFrame* frame,QWidget *parent = 0);
  void set(StanceResource* r);
  void set(ResourceBase* r) { set(dynamic_cast<StanceResource*>(r)); }
  virtual ~StanceFrame();
  StanceResource* resource;
  ResourceFrame* frame;

public slots:
  void CreateFlat();
  void CreateNearby();
  void Clean();
private:
  Ui::StanceFrame* ui;
};

#endif // STANCEFRAME_H
