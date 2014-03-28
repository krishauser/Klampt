#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QDebug"
#include <QTimer>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->displaywidget->installEventFilter(this);
    ui->displaywidget->setFocusPolicy(Qt::WheelFocus);
}

void MainWindow::Initialize(int _argc,const char** _argv)
{
    argc=_argc;
    argv=_argv;
    /*
    qDebug()<<argc;
    qDebug()<<argv[0];
    qDebug()<<argv[1];
    qDebug()<<argv[2];
    */
    //world=new RobotWorld();
    //SimTestBackend backend(world);
    //if(!ui->displaywidget->LoadAndInitSim(argc,argv)) {
    //  printf("ERROR");
    //}
    Robot robot;
    if(!robot.Load(argv[1])){
        printf("Error");
    }
    const string robname="placeholder";
    world=new RobotWorld();
    //world->AddRobot(robname,&robot);
    world->LoadRobot(argv[1]);
    ui->displaywidget->world=world;
    //ui->displaywidget->LoadFile(argv[1]);
    //JointTestProgram program(&robot,NULL);
    printf("BACKEND LOADED\n");
//    gui=new GenericGUIBase(ui->displaywidget);
    gui=new QtGUIBase(ui->displaywidget,world);
    ui->displaywidget->Start();


    //mediator, can be moved to direct calls
    connect(ui->displaywidget, SIGNAL(MouseMove(QMouseEvent*)),gui,SLOT(SendMouseMove(QMouseEvent*)));
    connect(ui->displaywidget, SIGNAL(MousePress(QMouseEvent*)),gui,SLOT(SendMousePress(QMouseEvent*)));
    connect(ui->displaywidget, SIGNAL(MouseRelease(QMouseEvent*)),gui,SLOT(SendMouseRelease(QMouseEvent*)));
    connect(ui->displaywidget, SIGNAL(MouseWheel(QWheelEvent*)),gui,SLOT(SendMouseWheel(QWheelEvent*)));
    connect(ui->displaywidget, SIGNAL(KeyPress(QKeyEvent*)),gui,SLOT(SendKeyDown(QKeyEvent*)));
    connect(ui->displaywidget, SIGNAL(KeyRelease(QKeyEvent*)),gui,SLOT(SendKeyRelease(QKeyEvent*)));

    refresh_timer=new QTimer();
    connect(refresh_timer, SIGNAL(timeout()),ui->displaywidget,SLOT(updateGL()));
    refresh_timer->start(1000/30);
    //ui->displaywidget->Start();

    ui->displaywidget->installEventFilter(this);
    ui->displaywidget->setFocusPolicy(Qt::WheelFocus);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete world;
    delete gui;
}
