#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QTreeWidgetItem>

#include <Interface/GenericGUI.h>
#include <Modeling/Resources.h>

#include "resourcedockwidget.h"
#include "qresourcetreeitem.h"

namespace ResourceBrowser {
class MainWindow;
}

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    GenericGUIBase* gui;
    explicit MainWindow(QWidget *parent = 0);
    ResourceLibrary library;

    QResourceTreeItem* selected;

    ~MainWindow();
    void RobotPose();
public slots:
    void OpenFile(QString filename);
    virtual void OpenFile(){OpenFile(QString());}
    void ChangeSelectedItem(QTreeWidgetItem *_it);
    void PressedDelete();
    void PressedExpand();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
