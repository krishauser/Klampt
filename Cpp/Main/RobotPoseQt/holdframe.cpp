#include "holdframe.h"
#include "ui_holdframe.h"
#include <iostream>
using namespace std;

HoldFrame::HoldFrame(ResourceFrame* _frame,QWidget *parent)
  :QGroupBox(parent),
   resource(NULL),
   frame(_frame),
   ui(new Ui::HoldFrame)
{
  ui->setupUi(this);
}

void HoldFrame::set(HoldResource* r)
{
  assert(r != NULL);
  resource = r;
}

HoldFrame::~HoldFrame()
{
    delete ui;
}

void HoldFrame::Clean()
{
  frame->gui->SendCommand("clean_contacts");
}
