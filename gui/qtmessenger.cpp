#include "qtmessenger.h"

QtMessenger::QtMessenger()
{

}

QtMessenger::~QtMessenger()
{

}

void QtMessenger::emit_step_ended(double vz, double vx)
{
    emit step_ended(vz,vx);
}

void QtMessenger::emit_new_phase(double phi)
{
    emit new_phase(phi*10000000);
}

