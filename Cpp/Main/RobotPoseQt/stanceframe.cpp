#include "stanceframe.h"
#include "ui_stanceframe.h"
#include <iostream>
using namespace std;

StanceFrame::StanceFrame(ResourceFrame* _frame,QWidget *parent)
  :QGroupBox(parent),
   resource(NULL),
   frame(_frame),
   ui(new Ui::StanceFrame)
{
  ui->setupUi(this);
}

StanceFrame::~StanceFrame()
{
    delete ui;
}

void StanceFrame::set(StanceResource* r)
{
  assert(r != NULL);
  resource = r;
}

void StanceFrame::CreateFlat()
{
  frame->gui->SendCommand("get_flat_contacts",ui->contactToleranceSpin->value());
  frame->onResourceEdit();
}

void StanceFrame::CreateNearby()
{
  frame->gui->SendCommand("get_nearby_contacts",ui->contactToleranceSpin->value());
  frame->onResourceEdit();
}

void StanceFrame::Clean()
{
  frame->gui->SendCommand("clean_contacts",ui->contactToleranceSpin->value(),ui->contactToleranceSpin->value()*5);
  frame->onResourceEdit();
}
