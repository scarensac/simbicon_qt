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
    void emit_fluid_level(double level);

signals:
    void step_ended(double vz,double vx);
    void new_phase(int phi);
    void fluid_level(double level);
};

#endif // QTMESSENGER_H
