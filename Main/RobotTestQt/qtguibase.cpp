#include "qtguibase.h"

QtGUIBase::QtGUIBase(GenericBackendBase *_backend, RobotWorld *_world) :
    world(_world),GenericGUIBase(_backend)
{

  idle_timer=new QTimer();
  connect(idle_timer,SIGNAL(timeout()),this,SLOT(SendIdle()));
  //idle_timer->start(100);

  //UpdateGUI();
}

QtGUIBase::~QtGUIBase(){
}

bool QtGUIBase::OnCommand(const string &cmd, const string &args){
    if(cmd=="update_config"){
        UpdateGUI();
        return true;
    }
    else return GenericGUIBase::OnCommand(cmd,args);
}

void QtGUIBase::SendMouseMove(QMouseEvent *e){
    GenericGUIBase::SendMouseMove(e->x(),e->y());
}

void QtGUIBase::SendMousePress(QMouseEvent *e){
    int button=e->button();
    if(button==1) button=0;
    GenericGUIBase::SendMouseClick(button,1,e->x(),e->y());
    if(e->modifiers()&&Qt::CTRL)
      GenericGUIBase::SendKeyDown("control");
    else
      GenericGUIBase::SendKeyUp("control");
    if(e->modifiers()&&Qt::SHIFT)
      GenericGUIBase::SendKeyDown("shift");
    else
      GenericGUIBase::SendKeyUp("shift");
    if(e->modifiers()&&Qt::ALT)
      GenericGUIBase::SendKeyDown("alt");
    else
      GenericGUIBase::SendKeyUp("alt");
}

void QtGUIBase::SendMouseRelease(QMouseEvent *e){
    int button=e->button();
    if(button==1) button=0;
    GenericGUIBase::SendMouseClick(button,0,e->x(),e->y());
}

void QtGUIBase::SendMouseWheel(QWheelEvent *e){
    GenericGUIBase::SendMouseWheel(e->delta());
}

void QtGUIBase::SendKeyDown(QKeyEvent *e){
    int key = e->key();
    if(key==Qt::Key_Shift || key==Qt::Key_Control){
        return;}
    if(!(e->modifiers() & Qt::ShiftModifier))key=tolower(key);
    GenericGUIBase::SendKeyDown(string(1,key));
}

void QtGUIBase::SendKeyUp(QKeyEvent *e){
    int key = e->key();
    if(!(e->modifiers() & Qt::ShiftModifier))key=tolower(key);
    GenericGUIBase::SendKeyUp(string(1,key));
}

void QtGUIBase::SendIdle(){
    GenericGUIBase::SendIdle();
}

void QtGUIBase::ShowHelp(){
    QMessageBox *help = new QMessageBox();
    QString text=      "This is but a generic GUi!"
            "Insert help text here";
    help->setText(text);
    help->show();
}

//should be made more informative
void QtGUIBase::ShowAbout(){
    QMessageBox *about = new QMessageBox();
    QString text = "Generic GUI Base't\n"
            "Insert about text here";
    about->setText(text);
    about->show();
}
