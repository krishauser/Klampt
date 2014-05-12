#ifndef PLAYRESOURCEFRAME_H
#define PLAYRESOURCEFRAME_H

#include <QFrame>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>

using namespace std;

namespace Ui {
class PlayResourceFrame;
}

class PlayResourceFrame : public QFrame
{
    Q_OBJECT
    
public:
    explicit PlayResourceFrame(QWidget *parent = 0);
    ~PlayResourceFrame();
    void EnablePath(string args);
public slots:
    void Play();
    void Pause();
    void Record();
    void NewTime(int t);
signals:
    void TimeChanged(double);
private:
    Ui::PlayResourceFrame *ui;
};

#endif // PLAYRESOURCEFRAME_H
