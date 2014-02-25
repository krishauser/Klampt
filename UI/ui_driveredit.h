/********************************************************************************
** Form generated from reading UI file 'driveredit.ui'
**
** Created: Wed Feb 19 20:16:26 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DRIVEREDIT_H
#define UI_DRIVEREDIT_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_DriverEdit
{
public:
    QDialogButtonBox *buttonBox;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QSpacerItem *horizontalSpacer_4;
    QSpacerItem *horizontalSpacer_2;
    QLabel *lbl_min;
    QSpacerItem *horizontalSpacer_3;
    QComboBox *comboBox;
    QSpacerItem *horizontalSpacer;
    QLabel *lbl_max;
    QDoubleSpinBox *doubleSpinBox;
    QSlider *slider;

    void setupUi(QDialog *DriverEdit)
    {
        if (DriverEdit->objectName().isEmpty())
            DriverEdit->setObjectName(QString::fromUtf8("DriverEdit"));
        DriverEdit->resize(320, 183);
        buttonBox = new QDialogButtonBox(DriverEdit);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(10, 150, 301, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Close);
        gridLayoutWidget = new QWidget(DriverEdit);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(9, 9, 291, 141));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_4, 2, 3, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_2, 0, 3, 1, 2);

        lbl_min = new QLabel(gridLayoutWidget);
        lbl_min->setObjectName(QString::fromUtf8("lbl_min"));

        gridLayout->addWidget(lbl_min, 2, 0, 1, 1);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_3, 2, 1, 1, 1);

        comboBox = new QComboBox(gridLayoutWidget);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        gridLayout->addWidget(comboBox, 0, 2, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 0, 0, 1, 2);

        lbl_max = new QLabel(gridLayoutWidget);
        lbl_max->setObjectName(QString::fromUtf8("lbl_max"));

        gridLayout->addWidget(lbl_max, 2, 4, 1, 1);

        doubleSpinBox = new QDoubleSpinBox(gridLayoutWidget);
        doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));
        doubleSpinBox->setMinimum(99);
        doubleSpinBox->setSingleStep(0.03);

        gridLayout->addWidget(doubleSpinBox, 2, 2, 1, 1);

        slider = new QSlider(gridLayoutWidget);
        slider->setObjectName(QString::fromUtf8("slider"));
        slider->setStyleSheet(QString::fromUtf8(" QSlider::groove:vertical {\n"
"     background: red;\n"
"     position: absolute; /* absolutely position 4px from the left and right of the widget. setting margins on the widget should work too... */\n"
"     left: 4px; right: 4px;\n"
" }\n"
"\n"
" QSlider::handle:vertical {\n"
"     height: 10px;\n"
"     background: green;\n"
"     margin: 0 -4px; /* expand outside the groove */\n"
" }\n"
"\n"
" QSlider::add-page:vertical {\n"
"     background: white;\n"
" }\n"
"\n"
" QSlider::sub-page:vertical {\n"
"     background: pink;\n"
" }"));
        slider->setMinimum(0);
        slider->setMaximum(1000);
        slider->setSingleStep(1);
        slider->setPageStep(0);
        slider->setValue(500);
        slider->setSliderPosition(500);
        slider->setOrientation(Qt::Horizontal);
        slider->setInvertedAppearance(false);
        slider->setTickPosition(QSlider::NoTicks);

        gridLayout->addWidget(slider, 1, 0, 1, 5);

#ifndef QT_NO_SHORTCUT
        lbl_min->setBuddy(doubleSpinBox);
        lbl_max->setBuddy(doubleSpinBox);
#endif // QT_NO_SHORTCUT
        QWidget::setTabOrder(doubleSpinBox, buttonBox);
        QWidget::setTabOrder(buttonBox, comboBox);
        QWidget::setTabOrder(comboBox, slider);

        retranslateUi(DriverEdit);
        QObject::connect(comboBox, SIGNAL(currentIndexChanged(int)), DriverEdit, SLOT(NewSelection(int)));
        QObject::connect(slider, SIGNAL(valueChanged(int)), DriverEdit, SLOT(HandleSlider(int)));
        QObject::connect(doubleSpinBox, SIGNAL(valueChanged(double)), DriverEdit, SLOT(HandleSpinBox(double)));
        QObject::connect(buttonBox, SIGNAL(rejected()), DriverEdit, SLOT(reject()));
        QObject::connect(buttonBox, SIGNAL(accepted()), DriverEdit, SLOT(accept()));

        QMetaObject::connectSlotsByName(DriverEdit);
    } // setupUi

    void retranslateUi(QDialog *DriverEdit)
    {
        DriverEdit->setWindowTitle(QApplication::translate("DriverEdit", "Drivers", 0, QApplication::UnicodeUTF8));
        lbl_min->setText(QApplication::translate("DriverEdit", "TextLabel", 0, QApplication::UnicodeUTF8));
        lbl_max->setText(QApplication::translate("DriverEdit", "TextLabel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class DriverEdit: public Ui_DriverEdit {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DRIVEREDIT_H
