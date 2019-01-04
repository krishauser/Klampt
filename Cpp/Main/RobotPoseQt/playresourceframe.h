#ifndef PLAYRESOURCEFRAME_H
#define PLAYRESOURCEFRAME_H

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

#include <QTimer>
#include <QFrame>
#include <KrisLibrary/Timer.h>

#define RESOURCE_RECORD_FPS 60

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

    QTimer *timer;
    Timer realTimer;
    double realTimerStartTime;

    double duration;
    double time;
    double start;
    bool recording;

    void Reset();

public slots:
    void Play();
    void Tick();
    void Pause();
    void Record();
    void NewTime(int t);
    void UpdatePlayerTimeRange(double minTime,double maxTime);
    void EnablePlayer(bool enabled);
signals:
    void TimeChanged(double);
    void ToggleRecording(bool);
private:
    Ui::PlayResourceFrame *ui;
};

#endif // PLAYRESOURCEFRAME_H
