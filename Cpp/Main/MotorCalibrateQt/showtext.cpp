#include "showtext.h"
#include "ui_showtext.h"
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QDateTime>


ShowText::ShowText(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ShowText)
{
  ui->setupUi(this);
}

ShowText::~ShowText()
{
  delete ui;
}

void ShowText::SetText(QString str){
  ui->textEdit->setText(str);
}

void ShowText::copyText(){
  ui->textEdit->selectAll();
  ui->textEdit->copy();
}

void ShowText::appendText(){
  QFile robfile(rob);
  if(robfile.open(QFile::Append|QFile::Text)){
  QTextStream out(&robfile);
  out<<"\n#Added through MotorCalibrate on "<<QDateTime::currentDateTime().toString();
  out<<"\n"<<ui->textEdit->toPlainText();
  robfile.close();
  QMessageBox(QMessageBox::NoIcon,"Success","The Calibration data has been appended to "+rob).exec();
  }
  QMessageBox(QMessageBox::NoIcon,"Failure","Cannot open file for writing "+rob).exec();
}
