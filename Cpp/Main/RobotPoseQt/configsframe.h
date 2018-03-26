#ifndef CONFIGSFRAME_H
#define CONFIGSFRAME_H

#include "Modeling/Resources.h"
#include "resourceframe.h"
#include <QGroupBox>

namespace Ui {
class ConfigsFrame;
}

class ConfigsFrame : public QGroupBox
{
  Q_OBJECT
    
public:
  explicit ConfigsFrame(ResourceFrame* frame,QWidget *parent = 0);
  virtual ~ConfigsFrame();
  void set(ConfigsResource* r);
  void set(ResourceBase* r) { set(dynamic_cast<ConfigsResource*>(r)); }
  ResourceFrame* frame;
  ConfigsResource* resource;

public slots:
  void Optimize();
private:
  Ui::ConfigsFrame* ui;
};

#endif // CONFIGSFRAME_H
