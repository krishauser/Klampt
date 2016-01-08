#include "dialog.h"
#include "ui_dialog.h"
#include "Main/urdftorob.h"
#include <KrisLibrary/utils/AnyCollection.h>

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::findIn()
{
    QString filename = f.getOpenFileName(0,"Open URDF",ui->line_in->text(),"*.urdf");
    if(!filename.isEmpty()){
        ui->line_in->setText(filename);
        if(ui->line_out->text().isEmpty()){
            filename=QFileInfo(filename).path() +
            #ifdef WIN32
            "\\"
            #else
            "/"
            #endif
            + QString(QFileInfo(filename).baseName());
            filename.append(".rob");
            ui->line_out->setText(filename);
        }
    }
}

void Dialog::findOut(){
    QString filename = f.getSaveFileName(0,"Save Rob",ui->line_out->text(),"*.rob");
    if(!filename.isEmpty())
        ui->line_out->setText(filename);
}

void Dialog::findRoot(){
    QString filename = f.getExistingDirectory(0,"Package Root Path","");
    if(!filename.isEmpty())
        ui->line_root->setText(filename);

}

void Dialog::findPrefix(){
    QString filename = f.getExistingDirectory(0,"Package Root Path","");
    if(!filename.isEmpty())
        ui->line_prefix->setText(filename);
}

void Dialog::execute(){
  if(!QFileInfo(ui->line_in->text()).isFile()){
      ui->lbl_status->setText("Invalid Input File");
      return;
  }
  if(!QFileInfo(ui->line_in->text()).isWritable()){
      ui->lbl_status->setText("Unwritable Output File");
      return;
  }
//  QDir dir=QFileInfo(ui->line_in->text()).absoluteDir();
  if(!QFileInfo(ui->line_out->text()).dir().exists()){
//  if(!dir.exists()){
  ui->lbl_status->setText("Directory Not Found");
      return;
  }
  AnyCollection settings;
  settings["useVisGeom"] = ui->chk_visual->isChecked();
  settings["flipYZ"] = ui->chk_flip->isChecked();
  settings["outputGeometryExtension"] = ui->line_ext->text().toStdString();
  settings["outputGeometryPrefix"] = ui->line_prefix->text().toStdString();
  settings["packageRootPath"] = ui->line_root->text().toStdString();
  if(URDFtoRob(settings,ui->line_in->text().toStdString(),ui->line_out->text().toStdString()))
      ui->lbl_status->setText("Success");
  else
      ui->lbl_status->setText("Failure");

}
