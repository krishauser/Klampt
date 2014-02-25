/********************************************************************************
** Form generated from reading UI file 'logoptions.ui'
**
** Created: Wed Feb 19 20:16:26 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOGOPTIONS_H
#define UI_LOGOPTIONS_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LogOptions
{
public:
    QWidget *CentralWidget;
    QVBoxLayout *verticalLayout_3;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_4;
    QPushButton *pushButton_3;
    QPushButton *pushButton_2;
    QPushButton *pushButton;
    QVBoxLayout *verticalLayout_2;
    QCheckBox *chk_plot;
    QCheckBox *chk_log;
    QSpacerItem *verticalSpacer;
    QComboBox *comboBox;
    QSpacerItem *horizontalSpacer;
    QLabel *label;
    QLabel *label_3;
    QListWidget *list_show;

    void setupUi(QDialog *LogOptions)
    {
        if (LogOptions->objectName().isEmpty())
            LogOptions->setObjectName(QString::fromUtf8("LogOptions"));
        LogOptions->resize(360, 240);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(LogOptions->sizePolicy().hasHeightForWidth());
        LogOptions->setSizePolicy(sizePolicy);
        CentralWidget = new QWidget(LogOptions);
        CentralWidget->setObjectName(QString::fromUtf8("CentralWidget"));
        CentralWidget->setGeometry(QRect(0, 0, 369, 244));
        sizePolicy.setHeightForWidth(CentralWidget->sizePolicy().hasHeightForWidth());
        CentralWidget->setSizePolicy(sizePolicy);
        verticalLayout_3 = new QVBoxLayout(CentralWidget);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        pushButton_4 = new QPushButton(CentralWidget);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));

        verticalLayout->addWidget(pushButton_4);

        pushButton_3 = new QPushButton(CentralWidget);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));

        verticalLayout->addWidget(pushButton_3);

        pushButton_2 = new QPushButton(CentralWidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        verticalLayout->addWidget(pushButton_2);

        pushButton = new QPushButton(CentralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout->addWidget(pushButton);


        gridLayout->addLayout(verticalLayout, 3, 3, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        chk_plot = new QCheckBox(CentralWidget);
        chk_plot->setObjectName(QString::fromUtf8("chk_plot"));

        verticalLayout_2->addWidget(chk_plot);

        chk_log = new QCheckBox(CentralWidget);
        chk_log->setObjectName(QString::fromUtf8("chk_log"));

        verticalLayout_2->addWidget(chk_log);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        gridLayout->addLayout(verticalLayout_2, 0, 3, 3, 1);

        comboBox = new QComboBox(CentralWidget);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        gridLayout->addWidget(comboBox, 1, 1, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 1, 2, 1, 1);

        label = new QLabel(CentralWidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 2, 1, 1, 2);

        label_3 = new QLabel(CentralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout->addWidget(label_3, 0, 1, 1, 2);

        list_show = new QListWidget(CentralWidget);
        list_show->setObjectName(QString::fromUtf8("list_show"));

        gridLayout->addWidget(list_show, 3, 1, 1, 2);


        verticalLayout_3->addLayout(gridLayout);


        retranslateUi(LogOptions);
        QObject::connect(comboBox, SIGNAL(currentIndexChanged(int)), LogOptions, SLOT(ChangeSensor(int)));
        QObject::connect(list_show, SIGNAL(currentRowChanged(int)), LogOptions, SLOT(ChangeSelectedIndex(int)));
        QObject::connect(pushButton, SIGNAL(clicked()), LogOptions, SLOT(ShowAll()));
        QObject::connect(chk_plot, SIGNAL(clicked(bool)), LogOptions, SLOT(TogglePlot(bool)));
        QObject::connect(chk_log, SIGNAL(clicked(bool)), LogOptions, SLOT(ToggleLogging(bool)));
        QObject::connect(pushButton_2, SIGNAL(clicked()), LogOptions, SLOT(Isolate()));
        QObject::connect(pushButton_3, SIGNAL(clicked()), LogOptions, SLOT(HideItem()));
        QObject::connect(pushButton_4, SIGNAL(clicked()), LogOptions, SLOT(ShowItem()));

        QMetaObject::connectSlotsByName(LogOptions);
    } // setupUi

    void retranslateUi(QDialog *LogOptions)
    {
        LogOptions->setWindowTitle(QApplication::translate("LogOptions", "Plot Options", 0, QApplication::UnicodeUTF8));
        pushButton_4->setText(QApplication::translate("LogOptions", "Show", 0, QApplication::UnicodeUTF8));
        pushButton_3->setText(QApplication::translate("LogOptions", "Hide", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("LogOptions", "Isolate", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("LogOptions", "Show All", 0, QApplication::UnicodeUTF8));
        chk_plot->setText(QApplication::translate("LogOptions", "Plot On", 0, QApplication::UnicodeUTF8));
        chk_log->setText(QApplication::translate("LogOptions", "Logging", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("LogOptions", "Measurements", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("LogOptions", "Sensor", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class LogOptions: public Ui_LogOptions {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOGOPTIONS_H
