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

void QtMessenger::emit_target_sagital_speed(double speed) {
    emit set_target_sagital_speed(speed*100);
}

void QtMessenger::emit_target_coronal_speed(double speed) {
    emit set_target_coronal_speed(speed*100);
}

void QtMessenger::emit_desired_heading(double orientation) {
    emit set_desired_heading(orientation*100);
}

void QtMessenger::emit_step_width(double width) {
    emit set_step_width(width*10000);
}

void QtMessenger::emit_liquid_level(double level) {
    emit set_liquid_level(level*100);
}


void QtMessenger::emit_fluid_level(double level){
    fluid_level(level);
}

