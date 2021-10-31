#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QDebug"
#include <QTimer>
#include <cmath>

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

bool MainWindow::Initialize(int _argc,const char** _argv)
{
    argc=_argc;
    argv=_argv;

    if(world.LoadRobot(argv[1]) < 0) {
        printf("RobotTest: error loading robot file %s, quitting\n",argv[1]);
        return false;
    }
    RobotModel* rob=world.robots[0].get();
    backend.reset(new RobotTestBackend(&world));
    printf("BACKEND LOADED\n");
    gui.reset(new QRobotTestGUI(ui->displaywidget,backend.get(),rob));
    gui->opened_file = argv[1];
    backend->Start();

    //Receive info from the GUI
    connect(gui.get(),SIGNAL(UpdateDriverValue()),this,SLOT(UpdateDriverValue()));
    connect(gui.get(),SIGNAL(UpdateDriverParameters()),this,SLOT(UpdateDriverParameters()));

    connect(gui.get(),SIGNAL(UpdateLinkValue()),this,SLOT(UpdateLinkValue()));
    connect(gui.get(),SIGNAL(UpdateLinkParameters()),this,SLOT(UpdateLinkParameters()));

    //Send GUI events
    connect(ui->spn_driver,SIGNAL(valueChanged(double)),gui.get(),SLOT(SetDriverValue(double)));
    connect(ui->spn_link,SIGNAL(valueChanged(double)),gui.get(),SLOT(SetLinkValue(double)));

    //fill GUI info
    for(int i=0;i<rob->linkNames.size();i++)
        ui->cb_link->addItem(QString::fromStdString(rob->linkNames[i]));
    for(int i=0;i<rob->drivers.size();i++)
        ui->cb_driver->addItem(QString::fromStdString(rob->driverNames[i]));

    ui->spn_driver_index->setMaximum(rob->drivers.size() - 1);
    ui->spn_link_index->setMaximum(rob->links.size() - 1);

    ui->displaywidget->installEventFilter(this);
    ui->displaywidget->setFocusPolicy(Qt::WheelFocus);    
  
    return true;
}

void MainWindow::SetGeometry(bool status){
  gui->SendButtonToggle("draw_geom",status);
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

void MainWindow::SetExpanded(bool status){
  gui->SendButtonToggle("draw_expanded",status);
}

void MainWindow::SetCollisions(bool status){
  gui->SendButtonToggle("draw_self_collision_tests",status);
}

void MainWindow::SetSensors(bool status){
  gui->SendButtonToggle("draw_sensors",status);
}


void MainWindow::SetROS(bool status) {
  gui->SendButtonToggle("output_ros",status);
}

void MainWindow::SetIK(bool status){
  if(status) gui->SendCommand("constrain_point_mode");
  else gui->SendCommand("pose_mode");
}

void MainWindow::SetDriver(int index){
    gui->SetDriver(index);
}

void MainWindow::UpdateDriverParameters(){
    bool oldState = ui->spn_driver->blockSignals(true);
#define NUM(x) QString::number(x)
  RobotModel* rob = world.robots[0].get();
  if(gui->driver_index >=0) {
    RobotModelDriver dr=rob->drivers[gui->driver_index];
    QString driver_info=QString("V [%1 %2], T [%3,%4], PID %5,%6,%7").arg( \
          NUM(dr.vmin),NUM(dr.vmax),NUM(dr.tmin),NUM(dr.tmax),NUM(dr.servoP),NUM(dr.servoI),NUM(dr.servoD));
    ui->lbl_driver_info->setText(driver_info);
    Vector2 limits = rob->GetDriverLimits(gui->driver_index);
    ui->spn_driver->setMinimum(limits.x);
    ui->spn_driver->setMaximum(limits.y);
  }
#undef NUM
     if(IsInf(ui->spn_driver->minimum()) || IsInf(ui->spn_driver->minimum()))
         ui->sld_driver->setEnabled(false);
     else ui->sld_driver->setEnabled(true);
  ui->spn_driver->blockSignals((oldState));
}

void MainWindow::SetLink(int index){
    gui->SetLink(index);
}

void MainWindow::UpdateLinkValue(){
    RobotModel* rob = world.robots[0].get();
    bool oldState = ui->spn_link->blockSignals(true);
    if(gui->link_index >= 0) {
      ui->spn_link->setValue(rob->q[gui->link_index]);
      UpdateLinkSlider(rob->q[gui->link_index]);
    }
    ui->spn_link->blockSignals(oldState);
}

void MainWindow::UpdateDriverValue(){
    RobotModel* rob = world.robots[0].get();
    bool oldState = ui->spn_driver->blockSignals(true);
    if(gui->driver_index >= 0) {
      ui->spn_driver->setValue(rob->GetDriverValue(gui->driver_index));
      UpdateDriverSlider(rob->GetDriverValue(gui->driver_index));
    }
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

void MainWindow::UpdateLinkParameters(){
#define NUM(x) QString::number(x)
  RobotModel* rob = world.robots[0].get();
  if(gui->link_index >= 0) {
    QString link_info=QString("V [%1 %2], A [%3,%4], T [%5,%6]").arg(
    NUM(rob->velMin(gui->link_index)),NUM(rob->velMax(gui->link_index)),
    NUM(-rob->accMax(gui->link_index)),NUM(rob->accMax(gui->link_index)),
    NUM(-rob->torqueMax(gui->link_index)),NUM(rob->torqueMax(gui->link_index)));
    ui->lbl_link_info->setText(link_info);
    ui->spn_link->setMinimum(rob->qMin(gui->link_index));
    ui->spn_link->setMaximum(rob->qMax(gui->link_index));
  }
  if(IsInf(ui->spn_link->minimum()) || IsInf(ui->spn_link->minimum()))
      ui->sld_link->setEnabled(false);
  else ui->sld_link->setEnabled(true);

#undef NUM
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

void MainWindow::PrintConfig()
{
    gui->SendButtonPress("print_pose");
}

void MainWindow::LoadFile(){
    gui->LoadFile();
}

void MainWindow::ReloadFile(){
    gui->ReloadFile();
}

MainWindow::~MainWindow()
{
    delete ui;
    gui = NULL;
    backend = NULL;
}
