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
  printf("TODO: CreateFlat\n");
  frame->onResourceEdit();
}

void StanceFrame::CreateNearby()
{
  printf("TODO: CreateNearby\n");
  frame->onResourceEdit();
}
