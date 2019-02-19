#include "simulationwindow.h"
#include "ui_simulationwindow.h"

#include "Globals.h"
#include "Core/SimGlobals.h"
#include "gui/ControllerEditor.h"
#include <QStringList>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <QPixmap>
#include <QIcon>
#include <vector>
#include "gui/UI.h"


SimulationWindow::SimulationWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SimulationWindow),
    m_nbr_step_until_save(-1)
{
    move(50+1066+18,50);
    ui->setupUi(this);

    std::vector<std::string> vect_path;
    std::vector<QPushButton*> vect_widget;

    vect_path.push_back(Globals::data_folder_path+"buttons/start.png");
    vect_widget.push_back(ui->btn_run);

    vect_path.push_back(Globals::data_folder_path+"buttons/pause.png");
    vect_widget.push_back(ui->btn_pause);

    vect_path.push_back(Globals::data_folder_path+"buttons/reload.png");
    vect_widget.push_back(ui->btn_reload);

    vect_path.push_back(Globals::data_folder_path+"buttons/restart.png");
    vect_widget.push_back(ui->btn_restart);

    vect_path.push_back(Globals::data_folder_path+"buttons/step.png");
    vect_widget.push_back(ui->btn_step);

    //set the images on the butons
    for (int i=0; i<vect_path.size();++i){
        QPixmap pix(QString::fromStdString(vect_path[i]));
        QIcon ButtonIcon(pix);
        vect_widget[i]->setIcon(ButtonIcon);
        vect_widget[i]->setIconSize(pix.rect().size());
        vect_widget[i]->setToolTip(vect_widget[i]->text());
        vect_widget[i]->setText("");
    }

    link_signals();

    load_from_data();

    std::ostringstream oss,oss1;
    oss<<std::fixed<<std::setprecision(4) <<"average sagittal speed: "<<0.000000<<" m.s-1";
    ui->lbl_avg_sagittal_speed->setText(QString::fromStdString(oss.str()));

    oss1<<std::fixed<<std::setprecision(4) <<"average coronal speed: "<<0.000000<<" m.s-1";
    ui->lbl_avg_coronal_speed->setText(QString::fromStdString(oss1.str()));

    ControllerEditor* editor=static_cast<ControllerEditor*>(Globals::app);
    std::vector<std::string> list_name=editor->get_trajectory_name_list();
    QStringList qlist;
    for (int i=0; i<list_name.size();++i){
        QString inter=QString::fromStdString(list_name[i]);
        qlist.append(inter);
    }
    ui->combo_box_curve_editor->addItems(qlist);




}


SimulationWindow::~SimulationWindow()
{
    delete ui;
}

void SimulationWindow::load_from_data()
{
    ui->spin_box_target_sagittal_speed->setValue(SimGlobals::velDSagittal);
    ui->spin_box_target_coronal_speed->setValue(SimGlobals::velDCoronal);
    ui->spin_box_target_orientation->setValue(SimGlobals::desiredHeading);
    ui->spin_box_step_width->setValue(SimGlobals::step_width);

    ui->spin_box_liquid_level->setValue(SimGlobals::water_level);
    ui->spin_box_density->setValue(SimGlobals::liquid_density);
    ui->spin_box_viscosity->setValue(SimGlobals::liquid_viscosity);

    ui->check_box_target_pose->setChecked(Globals::drawDesiredPose);
    ui->check_box_show_shadow->setChecked(Globals::drawShadows);
    ui->check_box_show_forces->setChecked(Globals::drawContactForces);
    ui->check_box_show_ground->setChecked(Globals::drawGroundPlane);
    ui->check_box_collision_primitives->setChecked(Globals::drawCollisionPrimitives);
    ui->check_box_follow_character->setChecked(Globals::followCharacter);
    ui->check_box_force_ipm->setChecked(SimGlobals::force_ipm);
    ui->check_box_stance_foot_contact_controller->setChecked(Globals::use_contact_controller);
    ui->check_box_stance_leg_orientation_controler->setChecked(Globals::use_stance_leg_orientation_controller);
    ui->spin_box_ratio_to_real_time->setValue(std::log10(1.0/Globals::animationTimeToRealTimeRatio));
    ui->check_box_show_fps->setChecked(Globals::drawFPS);


    ui->spin_box_state_duration->setValue(SimGlobals::requested_state_duration);
}

void SimulationWindow::link_signals()
{
    //Menues
    connect(ui->actionShow_console, SIGNAL(triggered(bool)), this, SLOT(action_show_console()));

    //all tabs
    connect(ui->btn_pause, SIGNAL(clicked(bool)), this, SLOT(pause_click()));
    connect(ui->btn_reload, SIGNAL(clicked(bool)), this, SLOT(reload_click()));
    connect(ui->btn_restart, SIGNAL(clicked(bool)), this, SLOT(restart_click()));
    connect(ui->btn_run, SIGNAL(clicked(bool)), this, SLOT(run_click()));
    connect(ui->btn_step, SIGNAL(clicked(bool)), this, SLOT(step_click()));
    connect(ui->btn_camera_back, SIGNAL(clicked(bool)), this, SLOT(click_camera_back()));
    connect(ui->btn_camera_front, SIGNAL(clicked(bool)), this, SLOT(click_camera_front()));
    connect(ui->btn_camera_side, SIGNAL(clicked(bool)), this, SLOT(click_camera_side()));
    connect(ui->btn_camera_45, SIGNAL(clicked(bool)), this, SLOT(click_camera_45()));
    connect(ui->btn_camera_n45, SIGNAL(clicked(bool)), this, SLOT(click_camera_n45()));


    //general tab
    connect(ui->check_box_push_interface, SIGNAL(toggled(bool)), this, SLOT(check_box_push_interface_changed(bool)));
    connect(ui->check_box_curve_editor, SIGNAL(toggled(bool)), this, SLOT(check_box_curve_editor_changed(bool)));
    connect(ui->combo_box_curve_editor, SIGNAL(currentIndexChanged(int)), this, SLOT(combo_box_curve_editor_changed(int)));
    connect(ui->slider_step_width, SIGNAL(valueChanged(int)), this, SLOT(slider_step_width_changed(int)));
    connect(ui->spin_box_step_width, SIGNAL(valueChanged(double)), this, SLOT(spin_box_step_width_changed(double)));
    connect(ui->slider_state_duration, SIGNAL(valueChanged(int)), this, SLOT(slider_state_duration_changed(int)));
    connect(ui->spin_box_state_duration, SIGNAL(valueChanged(double)), this, SLOT(spin_box_state_duration_changed(double)));

    //speed control tab
    connect(ui->slider_target_sagittal_speed, SIGNAL(valueChanged(int)), this, SLOT(slider_target_sagittal_speed_changed(int)));
    connect(ui->slider_target_coronal_speed, SIGNAL(valueChanged(int)), this, SLOT(slider_target_coronal_speed_changed(int)));
    connect(ui->slider_target_orientation, SIGNAL(valueChanged(int)), this, SLOT(slider_target_orientation_changed(int)));

    connect(ui->spin_box_target_sagittal_speed, SIGNAL(valueChanged(double)), this, SLOT(spin_box_target_sagittal_speed_changed(double)));
    connect(ui->spin_box_target_coronal_speed, SIGNAL(valueChanged(double)), this, SLOT(spin_box_target_coronal_speed_changed(double)));
    connect(ui->spin_box_target_orientation, SIGNAL(valueChanged(double)), this, SLOT(spin_box_target_orientation_changed(double)));

    //liquid carac tab
    connect(ui->slider_density, SIGNAL(valueChanged(int)), this, SLOT(slider_density_changed(int)));
    connect(ui->slider_liquid_level, SIGNAL(valueChanged(int)), this, SLOT(slider_liquid_level_changed(int)));
    connect(ui->slider_viscosity, SIGNAL(valueChanged(int)), this, SLOT(slider_viscosity_changed(int)));

    connect(ui->spin_box_density, SIGNAL(valueChanged(double)), this, SLOT(spin_box_density_changed(double)));
    connect(ui->spin_box_liquid_level, SIGNAL(valueChanged(double)), this, SLOT(spin_box_liquid_level_changed(double)));
    connect(ui->spin_box_viscosity, SIGNAL(valueChanged(double)), this, SLOT(spin_box_viscosity_changed(double)));

    connect(ui->check_box_fluid_follow_character, SIGNAL(toggled(bool)), this, SLOT(check_box_fluid_follow_character_changed(bool)));
    connect(ui->check_box_simulate_only_fluid, SIGNAL(toggled(bool)), this, SLOT(check_box_simulate_only_fluid_changed(bool)));
    connect(ui->btn_zero_fluid_velocities, SIGNAL(clicked(bool)), this, SLOT(click_zero_fluid_velocities()));
    connect(ui->check_box_fluid_control_level, SIGNAL(toggled(bool)), this, SLOT(check_box_fluid_control_level_changed(bool)));


    //Options tab
    connect(ui->check_box_follow_character, SIGNAL(toggled(bool)), this, SLOT(check_box_follow_character_changed(bool)));
    connect(ui->check_box_collision_primitives, SIGNAL(toggled(bool)), this, SLOT(check_box_show_collision_primitive_changed(bool)));
    connect(ui->check_box_show_forces, SIGNAL(toggled(bool)), this, SLOT(check_box_show_forces_changed(bool)));
    connect(ui->check_box_show_ground, SIGNAL(toggled(bool)), this, SLOT(check_box_show_ground_plane_changed(bool)));
    connect(ui->check_box_show_shadow, SIGNAL(toggled(bool)), this, SLOT(check_box_show_shadow_changed(bool)));
    connect(ui->check_box_target_pose, SIGNAL(toggled(bool)), this, SLOT(check_box_show_target_pose_changed(bool)));
    connect(ui->check_box_force_ipm, SIGNAL(toggled(bool)), this, SLOT(check_box_force_ipm_changed(bool)));
    connect(ui->check_box_stance_foot_contact_controller, SIGNAL(toggled(bool)), this, SLOT(check_box_stance_foot_contact_controller_changed(bool)));
    connect(ui->check_box_stance_leg_orientation_controler, SIGNAL(toggled(bool)), this, SLOT(check_box_stance_leg_orientation_controller_changed(bool)));
    connect(ui->check_box_show_fps, SIGNAL(toggled(bool)), this, SLOT(check_box_show_fps_changed(bool)));
    connect(ui->slider_ratio_to_real_time, SIGNAL(valueChanged(int)), this, SLOT(slider_ratio_to_real_time_changed(int)));
    connect(ui->spin_box_ratio_to_real_time, SIGNAL(valueChanged(double)), this, SLOT(spin_box_ratio_to_real_time_changed(double)));

    //save tab
    connect(ui->btn_schedule_save, SIGNAL(clicked(bool)), this, SLOT(click_schedule_save()));

}

void SimulationWindow::closeEvent(QCloseEvent *event)
{
    emit close_signal();
}

void SimulationWindow::action_show_console()
{
    emit show_console();
}

void SimulationWindow::pause_click()
{
    Globals::animationRunning=false;
}

void SimulationWindow::reload_click()
{
    if (Globals::app){
        Globals::app->restart();
        load_from_data();
        combo_box_curve_editor_changed(-1);
    }
}

void SimulationWindow::restart_click()
{
    if (Globals::app){
        Globals::app->stand();
    }
}

void SimulationWindow::run_click()
{
    Globals::animationRunning=true;
}

void SimulationWindow::step_click()
{
    if (Globals::animationRunning == 0){
        if (Globals::app){
            Globals::app->processTask();
        }
    }
}

void SimulationWindow::click_camera_back()
{
    camera(0);
}

void SimulationWindow::click_camera_front()
{
    camera(1);
}

void SimulationWindow::click_camera_side()
{
    camera(2);
}

void SimulationWindow::click_camera_45()
{
    camera(3);
}

void SimulationWindow::click_camera_n45()
{
    camera(4);
}

void SimulationWindow::check_box_push_interface_changed(bool val)
{
    Globals::drawPushInterface= val;
}

void SimulationWindow::check_box_curve_editor_changed(bool val)
{
    if (val==true){
        combo_box_curve_editor_changed();
    }else{
        Globals::drawCurveEditor=false;
    }
}

void SimulationWindow::combo_box_curve_editor_changed(int val)
{
    if (ui->check_box_curve_editor->isChecked()){
        ControllerEditor* editor=static_cast<ControllerEditor*>(Globals::app);
        editor->clearEditedCurves();
        int ret_val;
        if (val==-1){
            ret_val=editor->trajectoryToEdit(ui->combo_box_curve_editor->currentIndex());
        }else{
            ret_val=editor->trajectoryToEdit(val);
        }

        if (ret_val==0){
            Globals::drawCurveEditor=true;
        }else{
            Globals::drawCurveEditor=false;
        }
    }else{
        Globals::drawCurveEditor=false;
    }
}

void SimulationWindow::slider_phase_changed(int val)
{
    std::ostringstream oss;
    oss<<std::fixed<<std::setprecision(10) <<"Phase : "<<double(val)/10000000;
    ui->label_phase->setText(QString::fromStdString(oss.str()));
    ui->slider_phase->setValue(val);
}

void SimulationWindow::slider_step_width_changed(int val)
{
    ui->spin_box_step_width->setValue(val/10000.0);
    SimGlobals::step_width=val/10000.0;
}

void SimulationWindow::spin_box_step_width_changed(double val)
{
    ui->slider_step_width->setValue(val*10000);
    SimGlobals::step_width=val;
}

void SimulationWindow::slider_state_duration_changed(int val)
{
    double new_state_duration=val/1000.0;
    ui->spin_box_state_duration->setValue(new_state_duration);
    SimGlobals::requested_state_duration=new_state_duration;
    SimGlobals::state_duration_modified=true;
    std::cout<<"slider trigger: "<<new_state_duration<<std::endl;
}

void SimulationWindow::spin_box_state_duration_changed(double val)
{
    int new_state_duration=val*1000;
    ui->slider_state_duration->setValue(new_state_duration);
    SimGlobals::requested_state_duration=val;
    SimGlobals::state_duration_modified=true;
    std::cout<<"spin box trigger: "<<new_state_duration<<std::endl;
}

void SimulationWindow::slider_target_sagittal_speed_changed(int val)
{
    ui->spin_box_target_sagittal_speed->setValue(val/100.0);
    SimGlobals::velDSagittal=val/100.0;
}

void SimulationWindow::slider_target_coronal_speed_changed(int val)
{
    ui->spin_box_target_coronal_speed->setValue(val/100.0);
    SimGlobals::velDCoronal=val/100.0;
}

void SimulationWindow::slider_target_orientation_changed(int val)
{
    ui->spin_box_target_orientation->setValue(val/100.0);
    SimGlobals::desiredHeading=val/100.0;
}

void SimulationWindow::spin_box_target_sagittal_speed_changed(double val)
{
    ui->slider_target_sagittal_speed->setValue(val*100);
    SimGlobals::velDSagittal=val;
}

void SimulationWindow::spin_box_target_coronal_speed_changed(double val)
{
    ui->slider_target_coronal_speed->setValue(val*100);
    SimGlobals::velDCoronal=val;
}

void SimulationWindow::spin_box_target_orientation_changed(double val)
{
    ui->slider_target_orientation->setValue(val*100);
    SimGlobals::desiredHeading=val;
}

void SimulationWindow::slider_liquid_level_changed(int val)
{
    ui->spin_box_liquid_level->setValue(val/100.0);
    SimGlobals::water_level=val/100.0;

}

void SimulationWindow::slider_density_changed(int val)
{
    ui->spin_box_density->setValue(val);
    SimGlobals::liquid_density=val;
}

void SimulationWindow::slider_viscosity_changed(int val)
{
    ui->spin_box_viscosity->setValue(val/100.0);
    SimGlobals::liquid_viscosity=val/100.0;
}

void SimulationWindow::spin_box_liquid_level_changed(double val)
{
    ui->slider_liquid_level->setValue(val*100);
    SimGlobals::water_level=val;
}

void SimulationWindow::spin_box_density_changed(double val)
{
    ui->slider_density->setValue(val);
    SimGlobals::liquid_density=val;
}

void SimulationWindow::spin_box_viscosity_changed(double val)
{
    ui->slider_viscosity->setValue(val*100);
    SimGlobals::liquid_viscosity=val;
}


void SimulationWindow::check_box_fluid_follow_character_changed(bool val){
    Globals::fluidFollowCharacter = val;
}

void SimulationWindow::check_box_simulate_only_fluid_changed(bool val){
    Globals::simulateOnlyFluid = val;
}

void SimulationWindow::click_zero_fluid_velocities(){
    Globals::zeroFluidVelocities = true;
}


void SimulationWindow::check_box_fluid_control_level_changed(bool val){
    Globals::fluidControlLevel=val;
}


void SimulationWindow::check_box_show_target_pose_changed(bool val)
{
    Globals::drawDesiredPose=val;
}

void SimulationWindow::check_box_show_shadow_changed(bool val)
{
    Globals::drawShadows=val;
}

void SimulationWindow::check_box_show_forces_changed(bool val)
{
    Globals::drawContactForces=val;
}

void SimulationWindow::check_box_show_ground_plane_changed(bool val)
{
    Globals::drawGroundPlane=val;
}

void SimulationWindow::check_box_show_collision_primitive_changed(bool val)
{
    Globals::drawCollisionPrimitives=val;
}

void SimulationWindow::check_box_follow_character_changed(bool val)
{
    Globals::followCharacter=val;
}

void SimulationWindow::check_box_force_ipm_changed(bool val)
{
    SimGlobals::force_ipm=val;
}

void SimulationWindow::check_box_stance_foot_contact_controller_changed(bool val)
{
    Globals::use_contact_controller=val;
}

void SimulationWindow::check_box_stance_leg_orientation_controller_changed(bool val)
{
    Globals::use_stance_leg_orientation_controller=val;
}

void SimulationWindow::check_box_show_fps_changed(bool val)
{
    Globals::drawFPS=val;
}

void SimulationWindow::slider_ratio_to_real_time_changed(int val)
{
    ui->spin_box_ratio_to_real_time->setValue(val/1000.0);
    Globals::animationTimeToRealTimeRatio=1/std::pow(10,(val/1000.0));
}

void SimulationWindow::spin_box_ratio_to_real_time_changed(double val)
{   
    ui->slider_ratio_to_real_time->setValue(val*1000);
    Globals::animationTimeToRealTimeRatio=1/std::pow(10,val);
}

void SimulationWindow::click_schedule_save()
{
    if (ui->check_box_save_position->isChecked()||ui->check_box_save_controller->isChecked()){
        ui->btn_schedule_save->setEnabled(false);
        ui->check_box_save_controller->setEnabled(false);
        ui->check_box_save_position->setEnabled(false);

        m_nbr_step_until_save=ui->spin_box_steps_until_save->value();


        ui->lbl_feedback_save->setText(QString::number(m_nbr_step_until_save) + QString(" steps left until saving"));
    }
}

void SimulationWindow::character_step_ended(double vz,double vx)
{

    sag_speeds.push_back(vz);
    cor_speeds.push_back(vx);

    double avg_z=0;
    double avg_x=0;
    int i;

    auto it_sag=sag_speeds.end()-1;
    auto it_cor=cor_speeds.end()-1;

    for (i=0; i<3;++i){
        if (i>=sag_speeds.size()){
            break;
        }
        avg_z+=*(it_sag-i);
        avg_x+=*(it_cor-i);
    }
    if (i!=0){
        avg_z/=i;
        avg_x/=i;
    }


    std::ostringstream oss,oss1;
    oss<<std::fixed<<std::setprecision(4) <<"average sagittal speed: "<<avg_z<<" m.s-1";
    ui->lbl_avg_sagittal_speed->setText(QString::fromStdString(oss.str()));

    oss1<<std::fixed<<std::setprecision(4) <<"average coronal speed: "<<avg_x<<" m.s-1";
    ui->lbl_avg_coronal_speed->setText(QString::fromStdString(oss1.str()));


    //now I handle the save countdown
    if (m_nbr_step_until_save>0){
        m_nbr_step_until_save--;
        ui->lbl_feedback_save->setText(QString::number(m_nbr_step_until_save) + QString(" steps left until saving"));
    }else if (m_nbr_step_until_save==0){
        ui->lbl_feedback_save->setText(QString("saving next step"));
        m_nbr_step_until_save--;
        Globals::save_to_current_controller=true;
        Globals::save_position = ui->check_box_save_position->isChecked();
        Globals::save_controller = ui->check_box_save_controller->isChecked();
        Globals::drawControlShots = true;
    }else{
        if (!ui->btn_schedule_save->isEnabled()&&!Globals::drawControlShots){
            ui->lbl_feedback_save->setText(QString("saving complete"));
            ui->btn_schedule_save->setEnabled(true);
            ui->check_box_save_controller->setEnabled(true);
            ui->check_box_save_position->setEnabled(true);
        }
    }

    //set the state duration
    if (SimGlobals::requested_state_duration!=ui->spin_box_state_duration->value()){
        ui->spin_box_state_duration->setValue(SimGlobals::requested_state_duration);
    }

}
