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
    void emit_target_sagital_speed(double speed);
    void emit_target_coronal_speed(double speed);
    void emit_desired_heading(double orientation);
    void emit_step_width(double width);
    void emit_fluid_level(double level);
    void emit_liquid_level(double level);

signals:
    void step_ended(double vz,double vx);
    void new_phase(int phi);
    void set_target_sagital_speed(double speed);
    void set_target_coronal_speed(double speed);
    void set_desired_heading(double orientation);
    void set_step_width(double width);
    void fluid_level(double level);
    void set_liquid_level(double level);
};

#endif // QTMESSENGER_H
