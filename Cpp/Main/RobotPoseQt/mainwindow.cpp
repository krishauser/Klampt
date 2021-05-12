#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QDebug"
#include <QTimer>
#include <cmath>
#include <collisionoutput.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->displaywidget->installEventFilter(this);
    ui->displaywidget->setFocusPolicy(Qt::WheelFocus);
    LinkMode();
    ui->radioButton->setChecked(1);

}

void MainWindow::Initialize(int argc,const char** argv)
{
  manager = make_shared<ResourceManager>();
  backend = make_shared<RobotPoseBackend>(&world,manager.get());
  if(!backend->LoadCommandLine(argc,argv)){
    exit(1);
  }

  gui=make_shared<QRobotPoseGUI>(ui->displaywidget,backend.get());
    backend->Start();

    gui->resource_frame=ui->frame_resources;

    ui->frame_resources->gui = gui.get();
    ui->frame_resources->SetManager(manager.get());

    //Receive info from the GUI
    connect(gui.get(),SIGNAL(UpdateDriverValue()),this,SLOT(UpdateDriverValue()));
    connect(gui.get(),SIGNAL(UpdateDriverParameters()),this,SLOT(UpdateDriverParameters()));

    connect(gui.get(),SIGNAL(UpdateLinkValue()),this,SLOT(UpdateLinkValue()));
    connect(gui.get(),SIGNAL(UpdateLinkParameters()),this,SLOT(UpdateLinkParameters()));

    //Send GUI events
    connect(ui->spn_driver,SIGNAL(valueChanged(double)),gui.get(),SLOT(SetDriverValue(double)));
    connect(ui->spn_link,SIGNAL(valueChanged(double)),gui.get(),SLOT(SetLinkValue(double)));

    if(!world.robots.empty()) {
      Robot* rob=world.robots[0].get();

      //fill GUI info
      for(int i=0;i<rob->linkNames.size();i++)
          ui->cb_link->addItem(QString::fromStdString(rob->linkNames[i]))
                  ;
      for(int i=0;i<rob->drivers.size();i++)
          ui->cb_driver->addItem(QString::fromStdString(rob->driverNames[i]));

      ui->spn_driver_index->setMaximum(rob->drivers.size() - 1);
      ui->spn_link_index->setMaximum(rob->links.size() - 1);
    }

    ui->displaywidget->installEventFilter(this);
    ui->displaywidget->setFocusPolicy(Qt::WheelFocus);    

    SetFree();
}

void MainWindow::SetGeometry(bool status){
  gui->SendButtonToggle("draw_geom",status);
}

void MainWindow::SetPoser(bool status){
  gui->SendButtonToggle("draw_poser",status);
}

void MainWindow::SetBboxes(bool status){
  gui->SendButtonToggle("draw_bbs",status);
}


void MainWindow::SetCOM(bool status){
  gui->SendButtonToggle("draw_com",status);
}

void MainWindow::SetFrame(bool status){
  gui->SendButtonToggle("draw_frame",status);
}

void MainWindow::SetSensors(bool status){
  gui->SendButtonToggle("draw_sensors",status);
}


void MainWindow::SetFree() {
  gui->SendCommand("pose_mode");
  ui->btn_free->setChecked(true);
  ui->btn_ik->setChecked(false);
  ui->lbl_ik->setVisible(false);
  ui->line_ik->setVisible(false);
  ui->btn_delete->setVisible(false);
  ui->btn_constrain->setVisible(false);
  ui->btn_constrain_point->setVisible(false);
  ui->btn_constrain_point->setChecked(false);
  ui->btn_constrain->setChecked(0);
  ui->btn_delete->setChecked(0);
  ui->displaywidget->setStatusTip("Pose Mode: Right-click to pose joints");
}

void MainWindow::SetIK(){
  gui->SendCommand("constrain_point_mode");
  ui->btn_free->setChecked(false);
  ui->btn_ik->setChecked(true);
  ui->lbl_ik->setVisible(true);
  ui->line_ik->setVisible(true);
  ui->btn_delete->setVisible(true);
  ui->btn_constrain->setVisible(true);
  ui->btn_constrain_point->setVisible(true);
  ui->btn_constrain_point->setChecked(true);
  ui->btn_constrain->setChecked(0);
  ui->btn_delete->setChecked(0);
  ui->displaywidget->setStatusTip("IK Mode: right drag to pose by IK, c=constrain, d=delete");
}

void MainWindow::IKConstrain(){
  gui->SendCommand("constrain_link_mode");
  blockSignals(true);
  ui->btn_constrain_point->setChecked(0);
  ui->btn_constrain->setChecked(1);
  ui->btn_delete->setChecked(0);
  blockSignals(false);
}

void MainWindow::IKConstrainPoint(){
  gui->SendCommand("constrain_point_mode");
  blockSignals(true);
  ui->btn_constrain_point->setChecked(1);
  ui->btn_constrain->setChecked(0);
  ui->btn_delete->setChecked(0);
  blockSignals(false);
}

void MainWindow::IKDelete(){
  gui->SendCommand("delete_constraint_mode");
  blockSignals(true);
  ui->btn_constrain_point->setChecked(0);
  ui->btn_constrain->setChecked(0);
  ui->btn_delete->setChecked(1);
  blockSignals(false);
}

void MainWindow::SetDriver(int index){
    gui->SetDriver(index);
}

//this information is shared via pointer, not message passing
void MainWindow::UpdateDriverParameters(){
    if(world.robots.empty()) return;
    if(gui->driver_index < 0) return;
    Robot* rob=world.robots[0].get();
    bool oldState = ui->spn_driver->blockSignals(true);
#define NUM(x) QString::number(x)
  RobotJointDriver dr=rob->drivers[gui->driver_index];
  QString driver_info=QString("V [%1 %2], T [%3,%4], PID %5,%6,%7").arg( \
        NUM(dr.vmin),NUM(dr.vmax),NUM(dr.tmin),NUM(dr.tmax),NUM(dr.servoP),NUM(dr.servoI),NUM(dr.servoD));
  ui->lbl_driver_info->setText(driver_info);
  Vector2 limits = rob->GetDriverLimits(gui->driver_index);
  ui->spn_driver->setMinimum(limits.x);
  ui->spn_driver->setMaximum(limits.y);
#undef NUM(x)
     if(IsInf(ui->spn_driver->minimum()) || IsInf(ui->spn_driver->minimum()))
         ui->sld_driver->setEnabled(false);
     else ui->sld_driver->setEnabled(true);
  ui->spn_driver->blockSignals((oldState));
}

void MainWindow::SetLink(int index){
    gui->SetLink(index);
}

void MainWindow::UpdateLinkValue(){
    if(world.robots.empty()) return;
    if(gui->link_index < 0) return;
    Robot* rob=world.robots[0].get();
    bool oldState = ui->spn_link->blockSignals(true);
    ui->spn_link->setValue(rob->q[gui->link_index]);
    UpdateLinkSlider(rob->q[gui->link_index]);
    ui->spn_link->blockSignals(oldState);
}

void MainWindow::UpdateDriverValue(){
    if(world.robots.empty()) return;
    if(gui->driver_index < 0) return;
    Robot* rob=world.robots[0].get();
    bool oldState = ui->spn_driver->blockSignals(true);
    ui->spn_driver->setValue(rob->GetDriverValue(gui->driver_index));
    UpdateDriverSlider(rob->GetDriverValue(gui->driver_index));
    ui->spn_driver->blockSignals((oldState));
}

void MainWindow::UpdateLinkSlider(double value){
    bool oldState=ui->sld_link->blockSignals(true);
    float range=ui->spn_link->maximum()-ui->spn_link->minimum();
    float relative=value-ui->spn_link->minimum();
    ui->sld_link->setValue(relative/range*1000);
    ui->sld_link->blockSignals(oldState);
}

void MainWindow::UpdateDriverSlider(double value){
    bool oldState=ui->sld_driver->blockSignals(true);
    float range=ui->spn_driver->maximum()-ui->spn_driver->minimum();
    float relative=value-ui->spn_driver->minimum();
    ui->sld_driver->setValue(relative/range*1000);
    ui->sld_driver->blockSignals(oldState);
}

//this information is shared via pointer, not message passing
void MainWindow::UpdateLinkParameters(){
    if(world.robots.empty()) return;
    if(gui->link_index < 0) return;
    Robot* rob=world.robots[0].get();
#define NUM(x) QString::number(x)
  QString link_info=QString("[%1 %2], T [%3,%4]").arg(NUM(rob->velMin(gui->link_index)),NUM(rob->velMax(gui->link_index)),NUM(-rob->torqueMax(gui->link_index)),NUM(rob->torqueMax(gui->link_index)));
  ui->lbl_link_info->setText(link_info);
  ui->spn_link->setMinimum(rob->qMin(gui->link_index));
  ui->spn_link->setMaximum(rob->qMax(gui->link_index));
  if(IsInf(ui->spn_link->minimum()) || IsInf(ui->spn_link->minimum()))
      ui->sld_link->setEnabled(false);
  else ui->sld_link->setEnabled(true);

#undef NUM(x)
}

void MainWindow::LinkMode(){
    ui->lbl_mode->setText("Link");
    ui->spn_driver->hide();
    ui->cb_driver->hide();
    ui->lbl_driver_info->hide();
    ui->spn_driver->hide();
    ui->spn_driver_index->hide();
    ui->sld_driver->hide();

    ui->spn_link->show();
    ui->cb_link->show();
    ui->lbl_link_info->show();
    ui->spn_link->show();
    ui->spn_link_index->show();
    ui->sld_link->show();

}

void MainWindow::DriverMode(){
    ui->lbl_mode->setText("Driver");

    ui->spn_link->hide();
    ui->cb_link->hide();
    ui->lbl_link_info->hide();
    ui->spn_link->hide();
    ui->spn_link_index->hide();
    ui->sld_link->hide();

    ui->spn_driver->show();
    ui->cb_driver->show();
    ui->lbl_driver_info->show();
    ui->spn_driver->show();
    ui->spn_driver_index->show();
    ui->sld_driver->show();
}

void MainWindow::SliderLinkAngle(int ticks){
    float inc=(ui->spn_link->maximum()-ui->spn_link->minimum())/1000;
    float value=ui->spn_link->minimum()+inc*ticks;
    ui->spn_link->setValue(value);
}

void MainWindow::SliderDriverAngle(int ticks){
    float inc=(ui->spn_driver->maximum()-ui->spn_driver->minimum())/1000;
    float value=ui->spn_driver->minimum()+inc*ticks;
    ui->spn_driver->setValue(value);
}

void MainWindow::PrintCollisions(){
    gui->SendButtonPress("request_self_collisions");
}

void MainWindow::LoadFile(){
    gui->LoadFilePrompt("last_resource_location");
}

void MainWindow::SaveFile() {
  gui->SaveFilePrompt("last_resource_location");
}

MainWindow::~MainWindow()
{
    delete ui;
}
