#ifndef QTMESSENGER_H
#define QTMESSENGER_H

#include <QObject>
#include <QWidget>

class QtMessenger: public QObject
{
    Q_OBJECT

public:
    QtMessenger();
    virtual ~QtMessenger();

    void emit_step_ended(double vz,double vx);
    void emit_new_phase(double phi);

signals:
    void step_ended(double vz,double vx);
    void new_phase(int phi);
};

#endif // QTMESSENGER_H
