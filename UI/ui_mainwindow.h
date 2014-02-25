/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Sun Feb 23 22:58:16 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "qsimtestbackend.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpen;
    QAction *actionSave_State;
    QAction *actionLoad_MultiPath;
    QAction *actionReset;
    QAction *actionExit;
    QAction *actionSaveView;
    QAction *actionRevertView;
    QAction *actionDefaultView;
    QAction *actionDrivers;
    QAction *actionSensor_Plot;
    QAction *actionOptions;
    QAction *actionSave_State_As;
    QAction *actionRecordFile;
    QAction *actionMultiple_Parts;
    QAction *action640x480;
    QAction *action800x600;
    QAction *action1920x1080_HD;
    QAction *action1280x768;
    QAction *actionResizable;
    QAction *actionEncode;
    QAction *actionSend_Command;
    QAction *actionHelp;
    QAction *actionAbout;
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout_3;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_5;
    QSimTestBackend *displaywidget;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer_3;
    QSpacerItem *horizontalSpacer_6;
    QCheckBox *chk_contacts;
    QCheckBox *chk_bboxes;
    QCheckBox *chk_desired;
    QCheckBox *chk_estimated;
    QCheckBox *chk_poser;
    QCheckBox *chk_wrenches;
    QCheckBox *chk_expanded;
    QSpacerItem *horizontalSpacer_5;
    QSpacerItem *horizontalSpacer_4;
    QFrame *line;
    QHBoxLayout *horizontalLayout_5;
    QSpacerItem *horizontalSpacer;
    QFrame *line_2;
    QLabel *label_2;
    QPushButton *pushButton_4;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QFrame *line_3;
    QLabel *label;
    QPushButton *btn_free;
    QPushButton *btn_ik;
    QPushButton *btn_force;
    QFrame *line_ik;
    QLabel *lbl_ik;
    QPushButton *btn_constrain_point;
    QPushButton *btn_constrain;
    QPushButton *btn_delete;
    QSpacerItem *horizontalSpacer_2;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuMilestones;
    QMenu *menuPreferences;
    QMenu *menuRecord;
    QMenu *menuResolution;
    QMenu *menuHelp;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(688, 600);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QString::fromUtf8("actionOpen"));
        actionSave_State = new QAction(MainWindow);
        actionSave_State->setObjectName(QString::fromUtf8("actionSave_State"));
        actionLoad_MultiPath = new QAction(MainWindow);
        actionLoad_MultiPath->setObjectName(QString::fromUtf8("actionLoad_MultiPath"));
        actionReset = new QAction(MainWindow);
        actionReset->setObjectName(QString::fromUtf8("actionReset"));
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionSaveView = new QAction(MainWindow);
        actionSaveView->setObjectName(QString::fromUtf8("actionSaveView"));
        actionRevertView = new QAction(MainWindow);
        actionRevertView->setObjectName(QString::fromUtf8("actionRevertView"));
        actionDefaultView = new QAction(MainWindow);
        actionDefaultView->setObjectName(QString::fromUtf8("actionDefaultView"));
        actionDrivers = new QAction(MainWindow);
        actionDrivers->setObjectName(QString::fromUtf8("actionDrivers"));
        actionSensor_Plot = new QAction(MainWindow);
        actionSensor_Plot->setObjectName(QString::fromUtf8("actionSensor_Plot"));
        actionOptions = new QAction(MainWindow);
        actionOptions->setObjectName(QString::fromUtf8("actionOptions"));
        actionSave_State_As = new QAction(MainWindow);
        actionSave_State_As->setObjectName(QString::fromUtf8("actionSave_State_As"));
        actionRecordFile = new QAction(MainWindow);
        actionRecordFile->setObjectName(QString::fromUtf8("actionRecordFile"));
        actionMultiple_Parts = new QAction(MainWindow);
        actionMultiple_Parts->setObjectName(QString::fromUtf8("actionMultiple_Parts"));
        actionMultiple_Parts->setCheckable(true);
        action640x480 = new QAction(MainWindow);
        action640x480->setObjectName(QString::fromUtf8("action640x480"));
        action800x600 = new QAction(MainWindow);
        action800x600->setObjectName(QString::fromUtf8("action800x600"));
        action1920x1080_HD = new QAction(MainWindow);
        action1920x1080_HD->setObjectName(QString::fromUtf8("action1920x1080_HD"));
        action1280x768 = new QAction(MainWindow);
        action1280x768->setObjectName(QString::fromUtf8("action1280x768"));
        actionResizable = new QAction(MainWindow);
        actionResizable->setObjectName(QString::fromUtf8("actionResizable"));
        actionEncode = new QAction(MainWindow);
        actionEncode->setObjectName(QString::fromUtf8("actionEncode"));
        actionEncode->setEnabled(false);
        actionSend_Command = new QAction(MainWindow);
        actionSend_Command->setObjectName(QString::fromUtf8("actionSend_Command"));
        actionHelp = new QAction(MainWindow);
        actionHelp->setObjectName(QString::fromUtf8("actionHelp"));
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        horizontalLayout_3 = new QHBoxLayout(centralWidget);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        displaywidget = new QSimTestBackend(centralWidget);
        displaywidget->setObjectName(QString::fromUtf8("displaywidget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(1);
        sizePolicy.setHeightForWidth(displaywidget->sizePolicy().hasHeightForWidth());
        displaywidget->setSizePolicy(sizePolicy);
        displaywidget->setMinimumSize(QSize(200, 200));
        displaywidget->setMaximumSize(QSize(16777215, 16777215));

        verticalLayout_5->addWidget(displaywidget);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_3);

        horizontalSpacer_6 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_6);

        chk_contacts = new QCheckBox(centralWidget);
        chk_contacts->setObjectName(QString::fromUtf8("chk_contacts"));

        horizontalLayout->addWidget(chk_contacts);

        chk_bboxes = new QCheckBox(centralWidget);
        chk_bboxes->setObjectName(QString::fromUtf8("chk_bboxes"));

        horizontalLayout->addWidget(chk_bboxes);

        chk_desired = new QCheckBox(centralWidget);
        chk_desired->setObjectName(QString::fromUtf8("chk_desired"));

        horizontalLayout->addWidget(chk_desired);

        chk_estimated = new QCheckBox(centralWidget);
        chk_estimated->setObjectName(QString::fromUtf8("chk_estimated"));

        horizontalLayout->addWidget(chk_estimated);

        chk_poser = new QCheckBox(centralWidget);
        chk_poser->setObjectName(QString::fromUtf8("chk_poser"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(chk_poser->sizePolicy().hasHeightForWidth());
        chk_poser->setSizePolicy(sizePolicy1);
        chk_poser->setChecked(true);

        horizontalLayout->addWidget(chk_poser);

        chk_wrenches = new QCheckBox(centralWidget);
        chk_wrenches->setObjectName(QString::fromUtf8("chk_wrenches"));
        sizePolicy1.setHeightForWidth(chk_wrenches->sizePolicy().hasHeightForWidth());
        chk_wrenches->setSizePolicy(sizePolicy1);
        chk_wrenches->setChecked(true);

        horizontalLayout->addWidget(chk_wrenches);

        chk_expanded = new QCheckBox(centralWidget);
        chk_expanded->setObjectName(QString::fromUtf8("chk_expanded"));
        chk_expanded->setTristate(false);

        horizontalLayout->addWidget(chk_expanded);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_5);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_4);


        verticalLayout_5->addLayout(horizontalLayout);

        line = new QFrame(centralWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_5->addWidget(line);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer);

        line_2 = new QFrame(centralWidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::VLine);
        line_2->setFrameShadow(QFrame::Sunken);

        horizontalLayout_5->addWidget(line_2);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_5->addWidget(label_2);

        pushButton_4 = new QPushButton(centralWidget);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/Images/Images/yellowflag.gif"), QSize(), QIcon::Normal, QIcon::Off);
        pushButton_4->setIcon(icon);

        horizontalLayout_5->addWidget(pushButton_4);

        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/Images/Images/media-playback-start.svg"), QSize(), QIcon::Normal, QIcon::Off);
        icon1.addFile(QString::fromUtf8(":/Images/Images/media-playback-pause.svg"), QSize(), QIcon::Normal, QIcon::On);
        icon1.addFile(QString::fromUtf8(":/Images/Images/media-playback-pause.svg"), QSize(), QIcon::Active, QIcon::On);
        pushButton->setIcon(icon1);
        pushButton->setCheckable(true);

        horizontalLayout_5->addWidget(pushButton);

        pushButton_2 = new QPushButton(centralWidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(pushButton_2->sizePolicy().hasHeightForWidth());
        pushButton_2->setSizePolicy(sizePolicy2);
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/Images/Images/media-record.svg"), QSize(), QIcon::Normal, QIcon::Off);
        pushButton_2->setIcon(icon2);
        pushButton_2->setCheckable(true);

        horizontalLayout_5->addWidget(pushButton_2);

        line_3 = new QFrame(centralWidget);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);

        horizontalLayout_5->addWidget(line_3);

        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_5->addWidget(label);

        btn_free = new QPushButton(centralWidget);
        btn_free->setObjectName(QString::fromUtf8("btn_free"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/Images/Images/hand.gif"), QSize(), QIcon::Normal, QIcon::Off);
        btn_free->setIcon(icon3);
        btn_free->setCheckable(true);

        horizontalLayout_5->addWidget(btn_free);

        btn_ik = new QPushButton(centralWidget);
        btn_ik->setObjectName(QString::fromUtf8("btn_ik"));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/Images/Images/IK.gif"), QSize(), QIcon::Normal, QIcon::Off);
        btn_ik->setIcon(icon4);
        btn_ik->setCheckable(true);

        horizontalLayout_5->addWidget(btn_ik);

        btn_force = new QPushButton(centralWidget);
        btn_force->setObjectName(QString::fromUtf8("btn_force"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8("Images/spring.gif"), QSize(), QIcon::Normal, QIcon::Off);
        btn_force->setIcon(icon5);
        btn_force->setCheckable(true);

        horizontalLayout_5->addWidget(btn_force);

        line_ik = new QFrame(centralWidget);
        line_ik->setObjectName(QString::fromUtf8("line_ik"));
        line_ik->setFrameShape(QFrame::VLine);
        line_ik->setFrameShadow(QFrame::Sunken);

        horizontalLayout_5->addWidget(line_ik);

        lbl_ik = new QLabel(centralWidget);
        lbl_ik->setObjectName(QString::fromUtf8("lbl_ik"));

        horizontalLayout_5->addWidget(lbl_ik);

        btn_constrain_point = new QPushButton(centralWidget);
        btn_constrain_point->setObjectName(QString::fromUtf8("btn_constrain_point"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8("Images/constrain.gif"), QSize(), QIcon::Normal, QIcon::Off);
        btn_constrain_point->setIcon(icon6);

        horizontalLayout_5->addWidget(btn_constrain_point);

        btn_constrain = new QPushButton(centralWidget);
        btn_constrain->setObjectName(QString::fromUtf8("btn_constrain"));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/Images/Images/constrainpoint.gif"), QSize(), QIcon::Normal, QIcon::Off);
        btn_constrain->setIcon(icon7);

        horizontalLayout_5->addWidget(btn_constrain);

        btn_delete = new QPushButton(centralWidget);
        btn_delete->setObjectName(QString::fromUtf8("btn_delete"));
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/Images/Images/dialog-cancel-3.png"), QSize(), QIcon::Normal, QIcon::Off);
        btn_delete->setIcon(icon8);

        horizontalLayout_5->addWidget(btn_delete);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_5->addItem(horizontalSpacer_2);


        verticalLayout_5->addLayout(horizontalLayout_5);


        horizontalLayout_2->addLayout(verticalLayout_5);


        horizontalLayout_3->addLayout(horizontalLayout_2);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 688, 25));
        menuBar->setNativeMenuBar(false);
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuMilestones = new QMenu(menuBar);
        menuMilestones->setObjectName(QString::fromUtf8("menuMilestones"));
        menuPreferences = new QMenu(menuBar);
        menuPreferences->setObjectName(QString::fromUtf8("menuPreferences"));
        menuRecord = new QMenu(menuBar);
        menuRecord->setObjectName(QString::fromUtf8("menuRecord"));
        menuResolution = new QMenu(menuRecord);
        menuResolution->setObjectName(QString::fromUtf8("menuResolution"));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuPreferences->menuAction());
        menuBar->addAction(menuMilestones->menuAction());
        menuBar->addAction(menuRecord->menuAction());
        menuBar->addAction(menuHelp->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave_State);
        menuFile->addAction(actionSave_State_As);
        menuFile->addAction(actionReset);
        menuFile->addAction(actionExit);
        menuMilestones->addAction(actionSaveView);
        menuMilestones->addAction(actionRevertView);
        menuMilestones->addAction(actionDefaultView);
        menuPreferences->addAction(actionDrivers);
        menuPreferences->addAction(actionSensor_Plot);
        menuPreferences->addAction(actionOptions);
        menuPreferences->addAction(actionSend_Command);
        menuRecord->addAction(actionRecordFile);
        menuRecord->addAction(menuResolution->menuAction());
        menuRecord->addAction(actionMultiple_Parts);
        menuRecord->addAction(actionEncode);
        menuResolution->addAction(action640x480);
        menuResolution->addAction(action800x600);
        menuResolution->addAction(action1280x768);
        menuResolution->addAction(action1920x1080_HD);
        menuResolution->addSeparator();
        menuResolution->addAction(actionResizable);
        menuHelp->addAction(actionHelp);
        menuHelp->addAction(actionAbout);

        retranslateUi(MainWindow);
        QObject::connect(pushButton, SIGNAL(clicked(bool)), MainWindow, SLOT(SetSimulate(bool)));
        QObject::connect(pushButton_4, SIGNAL(clicked()), MainWindow, SLOT(SendMilestone()));
        QObject::connect(pushButton_2, SIGNAL(clicked(bool)), MainWindow, SLOT(SetRecord(bool)));
        QObject::connect(actionOpen, SIGNAL(triggered()), MainWindow, SLOT(LoadResource()));
        QObject::connect(actionSave_State_As, SIGNAL(triggered()), MainWindow, SLOT(SaveLastScenario()));
        QObject::connect(actionDrivers, SIGNAL(triggered()), MainWindow, SLOT(ShowDriverEdit()));
        QObject::connect(actionOptions, SIGNAL(triggered()), MainWindow, SLOT(ShowOptions()));
        QObject::connect(actionReset, SIGNAL(triggered()), MainWindow, SLOT(Reset()));
        QObject::connect(actionExit, SIGNAL(triggered()), MainWindow, SLOT(close()));
        QObject::connect(actionSensor_Plot, SIGNAL(triggered()), MainWindow, SLOT(ShowPlotOptions()));
        QObject::connect(chk_expanded, SIGNAL(clicked(bool)), MainWindow, SLOT(SetExpanded(bool)));
        QObject::connect(chk_wrenches, SIGNAL(clicked(bool)), MainWindow, SLOT(SetWrenches(bool)));
        QObject::connect(chk_bboxes, SIGNAL(clicked(bool)), MainWindow, SLOT(SetBBoxes(bool)));
        QObject::connect(chk_contacts, SIGNAL(clicked(bool)), MainWindow, SLOT(SetContacts(bool)));
        QObject::connect(chk_desired, SIGNAL(clicked(bool)), MainWindow, SLOT(SetDesired(bool)));
        QObject::connect(chk_poser, SIGNAL(clicked(bool)), MainWindow, SLOT(SetPoser(bool)));
        QObject::connect(chk_estimated, SIGNAL(clicked(bool)), MainWindow, SLOT(SetEstimated(bool)));
        QObject::connect(btn_constrain, SIGNAL(clicked()), MainWindow, SLOT(IKConstrain()));
        QObject::connect(btn_delete, SIGNAL(clicked()), MainWindow, SLOT(IKDelete()));
        QObject::connect(actionRecordFile, SIGNAL(triggered()), MainWindow, SLOT(ChangeRecordFile()));
        QObject::connect(action800x600, SIGNAL(triggered()), MainWindow, SLOT(Resolution800x600()));
        QObject::connect(action1920x1080_HD, SIGNAL(triggered()), MainWindow, SLOT(Resolution1920x1080()));
        QObject::connect(action640x480, SIGNAL(triggered()), MainWindow, SLOT(Resolution640x480()));
        QObject::connect(action1280x768, SIGNAL(triggered()), MainWindow, SLOT(Resolution1280x768()));
        QObject::connect(actionResizable, SIGNAL(triggered()), MainWindow, SLOT(FlexibleResolution()));
        QObject::connect(actionMultiple_Parts, SIGNAL(triggered(bool)), MainWindow, SLOT(DoMultipleParts(bool)));
        QObject::connect(actionEncode, SIGNAL(triggered()), MainWindow, SLOT(Encode()));
        QObject::connect(actionMultiple_Parts, SIGNAL(triggered(bool)), actionEncode, SLOT(setEnabled(bool)));
        QObject::connect(actionSaveView, SIGNAL(triggered()), MainWindow, SLOT(SaveViewport()));
        QObject::connect(actionRevertView, SIGNAL(triggered()), MainWindow, SLOT(LoadViewport()));
        QObject::connect(actionDefaultView, SIGNAL(triggered()), MainWindow, SLOT(DefaultViewport()));
        QObject::connect(btn_constrain_point, SIGNAL(clicked()), MainWindow, SLOT(IKConstrainPoint()));
        QObject::connect(btn_force, SIGNAL(clicked()), MainWindow, SLOT(DoForceMode()));
        QObject::connect(btn_free, SIGNAL(clicked()), MainWindow, SLOT(DoFreeMode()));
        QObject::connect(btn_ik, SIGNAL(clicked()), MainWindow, SLOT(DoIKMode()));
        QObject::connect(actionSend_Command, SIGNAL(triggered()), MainWindow, SLOT(ShowCommand()));
        QObject::connect(actionHelp, SIGNAL(triggered()), MainWindow, SLOT(ShowHelp()));
        QObject::connect(actionAbout, SIGNAL(triggered()), MainWindow, SLOT(ShowAbout()));

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "SimTest", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        MainWindow->setStatusTip(QApplication::translate("MainWindow", "start recording", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        actionOpen->setText(QApplication::translate("MainWindow", "Open", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        actionOpen->setStatusTip(QApplication::translate("MainWindow", "ctrl-o", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        actionOpen->setShortcut(QApplication::translate("MainWindow", "Ctrl+O", 0, QApplication::UnicodeUTF8));
        actionSave_State->setText(QApplication::translate("MainWindow", "Save State", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionSave_State->setToolTip(QApplication::translate("MainWindow", "ctrl-s", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        actionSave_State->setStatusTip(QString());
#endif // QT_NO_STATUSTIP
        actionSave_State->setShortcut(QApplication::translate("MainWindow", "Ctrl+S", 0, QApplication::UnicodeUTF8));
        actionLoad_MultiPath->setText(QApplication::translate("MainWindow", "Load MultiPath", 0, QApplication::UnicodeUTF8));
        actionReset->setText(QApplication::translate("MainWindow", "Reset", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", 0, QApplication::UnicodeUTF8));
        actionSaveView->setText(QApplication::translate("MainWindow", "Save View", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionSaveView->setToolTip(QApplication::translate("MainWindow", "v", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionRevertView->setText(QApplication::translate("MainWindow", "Revert", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionRevertView->setToolTip(QApplication::translate("MainWindow", "V", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionDefaultView->setText(QApplication::translate("MainWindow", "Default", 0, QApplication::UnicodeUTF8));
        actionDrivers->setText(QApplication::translate("MainWindow", "Drivers", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionDrivers->setToolTip(QApplication::translate("MainWindow", "ctrl-d", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionDrivers->setShortcut(QApplication::translate("MainWindow", "Ctrl+D", 0, QApplication::UnicodeUTF8));
        actionSensor_Plot->setText(QApplication::translate("MainWindow", "Sensor Plot", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionSensor_Plot->setToolTip(QApplication::translate("MainWindow", "ctrl-p", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionSensor_Plot->setShortcut(QApplication::translate("MainWindow", "Ctrl+P", 0, QApplication::UnicodeUTF8));
        actionOptions->setText(QApplication::translate("MainWindow", "Options", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionOptions->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>ctrl-r</p><p><br/></p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionOptions->setShortcut(QApplication::translate("MainWindow", "Ctrl+R", 0, QApplication::UnicodeUTF8));
        actionSave_State_As->setText(QApplication::translate("MainWindow", "Save State As", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionSave_State_As->setToolTip(QApplication::translate("MainWindow", "ctrl-shift-s", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionSave_State_As->setShortcut(QApplication::translate("MainWindow", "Ctrl+Shift+S", 0, QApplication::UnicodeUTF8));
        actionRecordFile->setText(QApplication::translate("MainWindow", "Change File", 0, QApplication::UnicodeUTF8));
        actionMultiple_Parts->setText(QApplication::translate("MainWindow", "Multiple Parts", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        actionMultiple_Parts->setStatusTip(QApplication::translate("MainWindow", "Does not automatically encode when a recording ends.  Can record 2 parts consecutively into 1 video.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        action640x480->setText(QApplication::translate("MainWindow", "640x480", 0, QApplication::UnicodeUTF8));
        action800x600->setText(QApplication::translate("MainWindow", "800x600", 0, QApplication::UnicodeUTF8));
        action1920x1080_HD->setText(QApplication::translate("MainWindow", "1920x1080 (HD)", 0, QApplication::UnicodeUTF8));
        action1280x768->setText(QApplication::translate("MainWindow", "1280x768", 0, QApplication::UnicodeUTF8));
        actionResizable->setText(QApplication::translate("MainWindow", "Resizable", 0, QApplication::UnicodeUTF8));
        actionEncode->setText(QApplication::translate("MainWindow", "Encode", 0, QApplication::UnicodeUTF8));
        actionSend_Command->setText(QApplication::translate("MainWindow", "Send Command", 0, QApplication::UnicodeUTF8));
        actionHelp->setText(QApplication::translate("MainWindow", "Help", 0, QApplication::UnicodeUTF8));
        actionHelp->setShortcut(QApplication::translate("MainWindow", "F1", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindow", "About", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        displaywidget->setToolTip(QApplication::translate("MainWindow", "free mode", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        chk_contacts->setStatusTip(QApplication::translate("MainWindow", "draw contacts", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chk_contacts->setText(QApplication::translate("MainWindow", "contacts", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chk_bboxes->setStatusTip(QApplication::translate("MainWindow", "draw bounding boxes", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chk_bboxes->setText(QApplication::translate("MainWindow", "bboxes", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chk_desired->setStatusTip(QApplication::translate("MainWindow", "draw desired configuration", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chk_desired->setText(QApplication::translate("MainWindow", "desired", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chk_estimated->setStatusTip(QApplication::translate("MainWindow", "draw estimated configuration", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chk_estimated->setText(QApplication::translate("MainWindow", "estimated", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chk_poser->setStatusTip(QApplication::translate("MainWindow", "draw the poser widget", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chk_poser->setText(QApplication::translate("MainWindow", "poser", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chk_wrenches->setStatusTip(QApplication::translate("MainWindow", "draw wrenches", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chk_wrenches->setText(QApplication::translate("MainWindow", "Wrenches", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        chk_expanded->setStatusTip(QApplication::translate("MainWindow", "draw expanded", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        chk_expanded->setText(QApplication::translate("MainWindow", "expanded", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Simulation", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        pushButton_4->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>Direct the simulation to move to the milestone</p></body></html>", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        pushButton_4->setStatusTip(QApplication::translate("MainWindow", "set a milestone for the robots to acheive", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        pushButton_4->setText(QString());
        pushButton_4->setShortcut(QApplication::translate("MainWindow", "Space", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        pushButton->setToolTip(QApplication::translate("MainWindow", "Start the simulation", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        pushButton->setStatusTip(QApplication::translate("MainWindow", "start or stop the simulation", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        pushButton->setText(QString());
        pushButton->setShortcut(QApplication::translate("MainWindow", "S", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        pushButton_2->setToolTip(QApplication::translate("MainWindow", "Record the simulation", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        pushButton_2->setText(QString());
        pushButton_2->setShortcut(QApplication::translate("MainWindow", "R", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Mode", 0, QApplication::UnicodeUTF8));
        btn_free->setText(QString());
        btn_ik->setText(QString());
        btn_force->setText(QString());
        lbl_ik->setText(QApplication::translate("MainWindow", "IK", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_STATUSTIP
        btn_constrain_point->setStatusTip(QApplication::translate("MainWindow", "constrain a point", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        btn_constrain_point->setText(QString());
#ifndef QT_NO_STATUSTIP
        btn_constrain->setStatusTip(QApplication::translate("MainWindow", "click a robot link to constrain it", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        btn_constrain->setText(QString());
#ifndef QT_NO_STATUSTIP
        btn_delete->setStatusTip(QApplication::translate("MainWindow", "delete a robot constraint", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        btn_delete->setText(QString());
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuMilestones->setTitle(QApplication::translate("MainWindow", "View", 0, QApplication::UnicodeUTF8));
        menuPreferences->setTitle(QApplication::translate("MainWindow", "Windows", 0, QApplication::UnicodeUTF8));
        menuRecord->setTitle(QApplication::translate("MainWindow", "Record", 0, QApplication::UnicodeUTF8));
        menuResolution->setTitle(QApplication::translate("MainWindow", "Resolution", 0, QApplication::UnicodeUTF8));
        menuHelp->setTitle(QApplication::translate("MainWindow", "Help", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
