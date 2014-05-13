#ifndef PLAYRESOURCEFRAME_H
#define PLAYRESOURCEFRAME_H

#include <stdio.h>
#include <sstream>
#include <printf.h>

#include <QTimer>
#include <QFrame>

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

    QTimer *timer;

    double duration;
    double time;
    double start;
    bool recording;

public slots:
    void Play();
    void Tick();
    void Pause();
    void Record();
    void NewTime(int t);
signals:
    void TimeChanged(double);
    void ToggleRecording(bool);
    void TakeFrame();
private:
    Ui::PlayResourceFrame *ui;
};

#endif // PLAYRESOURCEFRAME_H
