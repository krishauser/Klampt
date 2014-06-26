#include "floatarrayframe.h"
#include "ui_floatarrayframe.h"
#include <iostream>
using namespace std;

FloatArrayFrame::FloatArrayFrame(ResourceFrame* _frame,QWidget *parent)
  :QGroupBox(parent),
   resource(NULL),
   frame(_frame),
    ui(new Ui::FloatArrayFrame)
{
  ui->setupUi(this);
}

void FloatArrayFrame::set(FloatArrayResource* r)
{
  assert(r != NULL);
  resource = r;
  if(resource->data.empty()) {
    ui->spin_index->setEnabled(false);
    ui->edit_value->setEnabled(false);
  }
  else {
    ui->spin_index->setEnabled(true);
    ui->edit_value->setEnabled(true);
    ui->spin_index->setMaximum(resource->data.size()-1);
  }
}

FloatArrayFrame::~FloatArrayFrame()
{
    delete ui;
}

void FloatArrayFrame::IndexChanged(int index)
{
  blockSignals(true);
  double v=resource->data[index];
  ui->edit_value->setText(QString::number(v));
  blockSignals(false);
}

void FloatArrayFrame::ValueChanged(QString val)
{
  bool ok;
  double v = val.toDouble(&ok);  
  if(ok) {
    resource->data[ui->spin_index->value()] = v;
    frame->onResourceEdit();
  }
}

void FloatArrayFrame::Insert()
{
  blockSignals(true);
  if(resource->data.size()==0) {
    ui->spin_index->setEnabled(true);
    ui->edit_value->setEnabled(true); 
    ui->spin_index->setValue(0);
    resource->data.push_back(0.0);
  }
  else {
    int index=ui->spin_index->value();
    assert(index >= 0 && index < (int)resource->data.size());
    resource->data.insert(resource->data.begin()+index,0.0);
  }
  ui->spin_index->setMaximum(resource->data.size()-1);
  int index=ui->spin_index->value();
  assert(index >= 0 && index < (int)resource->data.size());
  ui->edit_value->setText(QString::number(resource->data[index]));
  blockSignals(false);
  frame->onResourceEdit();
}

void FloatArrayFrame::Delete()
{
  if(resource->data.empty()) return;
  blockSignals(true);
  resource->data.erase(resource->data.begin()+ui->spin_index->value());
  if(resource->data.size()==0) {
    ui->spin_index->setEnabled(false);
    ui->edit_value->setEnabled(false); 
  }
  else {
    int index=ui->spin_index->value();
    if(index >= (int)resource->data.size()) {
      index = (int)resource->data.size()-1;
      ui->spin_index->setValue(index);
    }
    ui->spin_index->setMaximum(resource->data.size()-1);
    double v=resource->data[index];
    ui->edit_value->setText(QString::number(v));
  }
  blockSignals(false);
  frame->onResourceEdit();
}
