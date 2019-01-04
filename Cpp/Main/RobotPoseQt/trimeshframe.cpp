#include "trimeshframe.h"
#include "ui_trimeshframe.h"
#include <iostream>
using namespace std;

TriMeshFrame::TriMeshFrame(ResourceFrame* _frame,QWidget *parent)
  :QGroupBox(parent),
   resource(NULL),
   frame(_frame),
    ui(new Ui::TriMeshFrame)
{
  ui->setupUi(this);
}

void TriMeshFrame::set(TriMeshResource* r)
{
  assert(r != NULL);
  resource = r;

  stringstream ss;
  ss<<r->data.verts.size()<<" vertices, "<<r->data.tris.size()<<" triangles";
  ui->label->setText(ss.str().c_str());
}

TriMeshFrame::~TriMeshFrame()
{
    delete ui;
}


void TriMeshFrame::Simplify()
{
  if(resource->data.tris.empty()) return;
  float res = ui->spin_resolution->value();
  frame->gui->SendCommand("resample",res);
}
