/********************************************************************************
** Form generated from reading UI file 'controllersettings.ui'
**
** Created: Sun Feb 23 22:12:32 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONTROLLERSETTINGS_H
#define UI_CONTROLLERSETTINGS_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTableWidget>

QT_BEGIN_NAMESPACE

class Ui_ControllerSettings
{
public:
    QGridLayout *gridLayout;
    QTableWidget *tableWidget;
    QSpacerItem *horizontalSpacer;
    QPushButton *pushButton;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *pushButton_2;

    void setupUi(QDialog *ControllerSettings)
    {
        if (ControllerSettings->objectName().isEmpty())
            ControllerSettings->setObjectName(QString::fromUtf8("ControllerSettings"));
        ControllerSettings->resize(320, 240);
        gridLayout = new QGridLayout(ControllerSettings);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        tableWidget = new QTableWidget(ControllerSettings);
        if (tableWidget->columnCount() < 1)
            tableWidget->setColumnCount(1);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem);
        if (tableWidget->rowCount() < 1)
            tableWidget->setRowCount(1);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        tableWidget->setVerticalHeaderItem(0, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        tableWidget->setItem(0, 0, __qtablewidgetitem2);
        tableWidget->setObjectName(QString::fromUtf8("tableWidget"));
        tableWidget->setStyleSheet(QString::fromUtf8("font: 75 9pt \"Ubuntu\";"));
        tableWidget->setColumnCount(1);
        tableWidget->horizontalHeader()->setVisible(false);

        gridLayout->addWidget(tableWidget, 0, 0, 1, 4);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 1, 0, 1, 1);

        pushButton = new QPushButton(ControllerSettings);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        gridLayout->addWidget(pushButton, 1, 1, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_2, 1, 3, 1, 1);

        pushButton_2 = new QPushButton(ControllerSettings);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));

        gridLayout->addWidget(pushButton_2, 1, 2, 1, 1);


        retranslateUi(ControllerSettings);
        QObject::connect(tableWidget, SIGNAL(cellChanged(int,int)), ControllerSettings, SLOT(OnCellEdited(int,int)));
        QObject::connect(pushButton, SIGNAL(clicked()), ControllerSettings, SLOT(OnApply()));
        QObject::connect(pushButton_2, SIGNAL(clicked()), ControllerSettings, SLOT(hide()));

        QMetaObject::connectSlotsByName(ControllerSettings);
    } // setupUi

    void retranslateUi(QDialog *ControllerSettings)
    {
        ControllerSettings->setWindowTitle(QApplication::translate("ControllerSettings", "Controller Settings", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem = tableWidget->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("ControllerSettings", "Setting", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem1 = tableWidget->verticalHeaderItem(0);
        ___qtablewidgetitem1->setText(QApplication::translate("ControllerSettings", "New Row", 0, QApplication::UnicodeUTF8));

        const bool __sortingEnabled = tableWidget->isSortingEnabled();
        tableWidget->setSortingEnabled(false);
        tableWidget->setSortingEnabled(__sortingEnabled);

        pushButton->setText(QApplication::translate("ControllerSettings", "Apply", 0, QApplication::UnicodeUTF8));
        pushButton_2->setText(QApplication::translate("ControllerSettings", "Close", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ControllerSettings: public Ui_ControllerSettings {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONTROLLERSETTINGS_H
