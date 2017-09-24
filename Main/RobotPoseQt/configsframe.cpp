#include "configsframe.h"
#include "ui_configsframe.h"
#include "qrobotposegui.h"
#include <iostream>
using namespace std;

ConfigsFrame::ConfigsFrame(ResourceFrame* _frame,QWidget *parent)
  :QGroupBox(parent),
   resource(NULL),
   frame(_frame),
   ui(new Ui::ConfigsFrame)
{
  ui->setupUi(this);
}

void ConfigsFrame::set(ConfigsResource* r)
{
  assert(r != NULL);
  resource = r;
  if(resource->configs.size() < 2) {
    ui->btn_optimize->setEnabled(false);
    ui->spin_numDivs->setEnabled(false);
  }
  else {
    ui->btn_optimize->setEnabled(true);
    ui->spin_numDivs->setEnabled(true);
  }
}

ConfigsFrame::~ConfigsFrame()
{
    delete ui;
}

void ConfigsFrame::Optimize()
{
  int numdivs = ui->spin_numDivs->value();
  frame->gui->SendCommand("optimize_path",numdivs);
}
