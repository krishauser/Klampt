#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QDebug"
#include <QTimer>
#include <QWidgetList>
#include <QFileInfo>
#include <QInputDialog>

string toStdString(const QString& s);

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

bool  MainWindow::Initialize(int argc,const char** argv)
{
    backend = make_shared<SimTestBackend>(&world);
    if(!backend->LoadAndInitSim(argc,argv)) {
      printf("ERROR LOADING FROM COMMAND LINE");
      return false;
    }
    else
      printf("BACKEND LOADED\n");

    gui=make_shared<QSimTestGUI>(ui->displaywidget,backend.get());
    gui->ini=ini;
    ui->displaywidget->gui = gui.get();

    //set the system calls for encoding video
    ui->displaywidget->moviefile = toStdString(ini->value("video_record_file","klampt_record.mp4").toString());
    ui->displaywidget->SetVideoEncoding(toStdString(ini->value("video_encoding_command","ffmpeg -y -f image2 -i image%04d.ppm").toString()));

    ui->displaywidget->installEventFilter(this);
    ui->displaywidget->setFocusPolicy(Qt::WheelFocus);

    backend->Start();
    DoFreeMode();
    return true;
}

MainWindow::~MainWindow()
{
  if(backend) backend->Stop();
  if(ui) delete ui;
}

//gui stuff

void MainWindow::SetWrenches(bool status){
  gui->SendButtonToggle("draw_wrenches",status);
}

void MainWindow::SetPoser(bool status){
  gui->SendButtonToggle("draw_poser",status);
}

void MainWindow::SetPoseObjects(bool status){
  gui->SendButtonToggle("pose_objects",status);
}

void MainWindow::SetDesired(bool status){
  gui->SendButtonToggle("draw_desired",status);
}

void MainWindow::SetEstimated(bool status){
  gui->SendButtonToggle("draw_estimated",status);
}

void MainWindow::SetDrawTime(bool status){
  gui->SendButtonToggle("draw_time",status);
}

void MainWindow::SetBBoxes(bool status){
  gui->SendButtonToggle("draw_bbs",status);
}

void MainWindow::SetContacts(bool status){
  gui->SendButtonToggle("draw_contacts",status);
}

void MainWindow::SetExpanded(bool status){
  gui->SendButtonToggle("draw_expanded",status);
}

void MainWindow::SetPoseIK(bool status){
  gui->SendButtonToggle("pose_ik",status);
}

void MainWindow::SetSensorPlot(bool status){
  gui->SendButtonToggle("show_sensor",status);
}

void MainWindow::SetLogCheck(bool status){
  gui->SendButtonToggle("do_logging",status);
}

void MainWindow::SetSimulate(bool status){
  gui->SendButtonToggle("simulate",status);
}

void MainWindow::SetROS(bool status)
{
  gui->SendButtonToggle("output_ros",status);
}

void MainWindow::SendMilestone(){
    //gui->SendButtonPress("command_pose");
    gui->SendCommand("command_pose");
}


void MainWindow::SetRecord(bool status){
  //gui->SendCommand("record",status);
  if(status)
    ui->displaywidget->StartMovie();
  else
    ui->displaywidget->StopMovie();
}

void MainWindow::ChangeEncoderCommand(){
    QString preset = ini->value("video_encoding_command","ffmpeg -y -f image2 -i image%04d.ppm").toString();
    QString value = QInputDialog::getText(this,"Record Command","enter the system command for the encoder using input files *.ppm",
                          QLineEdit::Normal,preset);
    string tmp = toStdString(value);
    const char* cstr = tmp.c_str();
    ui->displaywidget->SetVideoEncoding(toStdString(value));
    ini->setValue("video_encoding_command",value);
}

void MainWindow::LogSimulation(bool status){
  string file;
  if(status) file = toStdString(ini->value("log_simulation_file","simtest_log.csv").toString());
  gui->SendCommand("log_sim",file);
}

void MainWindow::LogContactState(bool status){
  string file;
  if(status) file = toStdString(ini->value("log_contact_state_file","simtest_contact_log.csv").toString());
  gui->SendCommand("log_contact_state",file);
}

void MainWindow::LogContactWrenches(bool status){
  string file;
  if(status) file = toStdString(ini->value("log_contact_wrenches_file","simtest_wrenches_log.csv").toString());
  gui->SendCommand("log_contact_wrenches",file);
}

void MainWindow::LogSensedPath(bool status){
  string file;
  stringstream ss;
  if(status) file = toStdString(ini->value("log_sensed_path_file","simtest_sensed_path_log.path").toString());
  ss<<"0 "<<file;//multiple robots what do we do?
  gui->SendCommand("log_sensed_path",ss.str());
}

void MainWindow::LogCommandedPath(bool status){
  string file;
  stringstream ss;
  if(status) file = toStdString(ini->value("log_commanded_path_file","simtest_commanded_path_log.path").toString());
  ss<<"0 "<<file;//multiple robots what do we do?
  gui->SendCommand("log_commanded_path",ss.str());
}

void MainWindow::ChangeSimulationLogFile(){
  QFileDialog f;
  QString openDir = ini->value("log_simulation_file","simtest_log.csv").toString();
  QString filename = f.getSaveFileName(this,"Set Simulation Log File",openDir,tr("CSV File (*.csv);;Any File (*)"));
  ini->setValue("log_simulation_file",QFileInfo(filename).absoluteFilePath());
}

void MainWindow::ChangeContactStateLogFile(){
  QFileDialog f;
  QString openDir = ini->value("log_contact_state_file","simtest_contact_log.csv").toString();
  QString filename = f.getSaveFileName(this,"Set Contact State Log File",openDir,tr("CSV File (*.csv);;Any File (*)"));
  ini->setValue("log_contact_state_file",QFileInfo(filename).absoluteFilePath());
}

void MainWindow::ChangeContactWrenchesLogFile(){
  QFileDialog f;
  QString openDir = ini->value("log_contact_wrenches_file","simtest_wrenches_log.csv").toString();
  QString filename = f.getSaveFileName(this,"Set Contact Wrenches Log File",openDir,tr("CSV File (*.csv);;Any File (*)"));
  ini->setValue("log_contact_wrenches_file",QFileInfo(filename).absoluteFilePath());
}

void MainWindow::ChangeSensedPathLogFile(){
  QFileDialog f;
  QString openDir = ini->value("log_sensed_path_file","simtest_sensed_path_log.path").toString();
  QString filename = f.getSaveFileName(this,"Set Sensed Path Log File",openDir,tr("Path File (*.path);;Any File (*)"));
  ini->setValue("log_sensed_path_file",QFileInfo(filename).absoluteFilePath());
}

void MainWindow::ChangeCommandedPathLogFile(){
  QFileDialog f;
  QString openDir = ini->value("log_commanded_path_file","simtest_commanded_path_log.path").toString();
  QString filename = f.getSaveFileName(this,"Set commanded Path Log File",openDir,tr("Path File (*.path);;Any File (*)"));
  ini->setValue("log_commanded_path_file",QFileInfo(filename).absoluteFilePath());
}

void MainWindow::SetMode(int option){
  if(option==0) gui->SendCommand("pose_mode");
  if(option==1) gui->SendCommand("constrain_point_mode");
  if(option==2) gui->SendCommand("force_application_mode");
    ui->btn_free->setChecked(option==0);
    ui->chk_objects->setVisible(option==0);
    ui->btn_ik->setChecked(option==1);
    ui->lbl_ik->setVisible(option==1);
    ui->line_ik->setVisible(option==1);
    ui->btn_constrain->setVisible(option==1);
    ui->btn_constrain_point->setVisible(option==1);
    if(option==1) {
      ui->btn_constrain_point->setChecked(1);
      ui->btn_constrain->setChecked(0);
      ui->btn_delete->setChecked(0);
    }
    ui->btn_delete->setVisible(option==1);
    ui->btn_force->setChecked(option==2);
    if(option==0) ui->displaywidget->setStatusTip("Free Drag Mode: drag to rotate, shift drag to zoom, ctrl drag to truck");
    else if(option==1) ui->displaywidget->setStatusTip("IK Mode: right drag to pose by IK, c=constrain, d=delete");
    else if(option==2) ui->displaywidget->setStatusTip("Force Application Mode: right drag to apply a force");

}

void MainWindow::LoadResource(){
    gui->LoadFilePrompt();
}

void MainWindow::LoadPath(){
  gui->LoadFilePrompt("last_open_path_directory","*.path *.milestones *.xml");
}

void MainWindow::LoadState(){
  gui->LoadFilePrompt("last_open_state_directory","*.state");
}

void MainWindow::SaveScenario(){
    gui->SaveScenario();
}

void MainWindow::SaveLastScenario(){
    gui->SaveLastScenario();
}

void MainWindow::ShowDriverEdit(){
    gui->driver_tool->show();
}

void MainWindow::ShowController(){
    gui->controller_dialog->show();
}

void MainWindow::ShowPlotOptions(){
    gui->log_options->show();
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

void MainWindow::Reset(){
  gui->SendCommand("reset");
}

void MainWindow::ChangeRecordFile(){
    QString filter="MPG Video (*.mpg)";
    QString recordPath = ini->value("video_record_file",QDir::home().absolutePath()).toString();
    QString recordfilename = QFileDialog::getSaveFileName(this,"Recording Output",recordPath,filter,&filter);
    if(!recordfilename.isNull()) {
      ini->setValue("video_record_file",QFileInfo(recordfilename).absoluteFilePath());
      string str = toStdString(recordfilename);
      ui->displaywidget->moviefile = str;
    }
}

void MainWindow::ChangeResolution(int w, int h)
{
    ui->displaywidget->setMinimumSize(w,h);
    ui->displaywidget->setMaximumSize(w,h);
    //shrink accordingly
    QApplication::processEvents();
    QTimer::singleShot(0,this,SLOT(Shrink()));
}

void MainWindow::FlexibleResolution(){
    ui->displaywidget->setMinimumSize(200,200);
    ui->displaywidget->setMaximumSize(16777215,16777215);
}

void MainWindow::DoMultipleParts(bool status){
    ui->displaywidget->stop_encode=!status;
}

void MainWindow::Encode(){
    ui->displaywidget->EncodeMovie();
}

void MainWindow::DefaultViewport(){
  string appdataPath = AppUtils::GetApplicationDataPath("Klampt");
  string viewFile = appdataPath + string("/simtest_defaultview.txt");
    gui->SendCommand("load_view",viewFile);
}

void MainWindow::LoadViewport(){
  string appdataPath = AppUtils::GetApplicationDataPath("Klampt");
  string viewFile = appdataPath + string("/simtest_view.txt");
    gui->SendCommand("load_view",viewFile);
}

void MainWindow::SaveViewport(){
  string appdataPath = AppUtils::GetApplicationDataPath("Klampt");
  string viewFile = appdataPath + string("/simtest_view.txt");
    gui->SendCommand("save_view",viewFile);
}

void MainWindow::ShowHelp(){
    gui->ShowHelp();
}


void MainWindow::ShowAbout(){
    gui->ShowAbout();
}
