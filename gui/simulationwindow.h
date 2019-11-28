#ifndef SIMULATIONWINDOW_H
#define SIMULATIONWINDOW_H

#include <QMainWindow>

namespace Ui {
class SimulationWindow;
}

class SimulationWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit SimulationWindow(QWidget *parent = 0);
    ~SimulationWindow();

    void load_from_data();

signals:
    void show_console();
    void close_signal();

protected:
    void link_signals();

    void closeEvent(QCloseEvent *event);


protected slots:
    //Menues
    void action_show_console();

    //all tabs
    void pause_click();
    void reload_click();
    void restart_click();
    void run_click();
    void step_click();
    void click_camera_back();
    void click_camera_front();
    void click_camera_side();
    void click_camera_45();
    void click_camera_n45();


    //general tab
    void check_box_push_interface_changed(bool val);
    void check_box_curve_editor_changed(bool val);
    void combo_box_curve_editor_changed(int val=-1);
    void slider_phase_changed(int val);
    void slider_step_width_changed(int val);
    void spin_box_step_width_changed(double val);
    void slider_state_duration_changed(int val);
    void spin_box_state_duration_changed(double val);

    //speed control tab
    void slider_target_sagittal_speed_changed(int val);
    void slider_target_coronal_speed_changed(int val);
    void slider_target_orientation_changed(int val);
    void spin_box_target_sagittal_speed_changed(double val);
    void spin_box_target_coronal_speed_changed(double val);
    void spin_box_target_orientation_changed(double val);

    //liquid control tab
    void slider_liquid_level_changed(int val);
    void slider_density_changed(int val);
    void slider_viscosity_changed(int val);
    void spin_box_liquid_level_changed(double val);
    void spin_box_density_changed(double val);
    void spin_box_viscosity_changed(double val);
    void check_box_fluid_follow_character_changed(bool val);
    void check_box_simulate_only_fluid_changed(bool val);
    void click_zero_fluid_velocities();
    void check_box_fluid_control_level_changed(bool val);


    //Options tab
    void check_box_show_target_pose_changed(bool val);
    void check_box_show_shadow_changed(bool val);
    void check_box_show_forces_changed(bool val);
    void check_box_show_ground_plane_changed(bool val);
    void check_box_show_collision_primitive_changed(bool val);
    void check_box_follow_character_changed(bool val);
    void check_box_force_ipm_changed(bool val);
    void check_box_stance_foot_contact_controller_changed(bool val);
    void check_box_stance_leg_orientation_controller_changed(bool val);
    void check_box_show_fps_changed(bool val);
    void slider_ratio_to_real_time_changed(int val);
    void spin_box_ratio_to_real_time_changed(double val);

    //Save tab
    void click_schedule_save();

    //slot for external signals
    void character_step_ended(double vz, double vx);

    //other control tab
    void click_refresh_fps();

private:
    Ui::SimulationWindow *ui;

protected:
    int m_nbr_step_until_save;

private:
    //those members are old static variables that I moved to make sure they are reinitialized...
    //SimulationWindow::character_step_ended
    std::vector<double> sag_speeds;
    std::vector<double> cor_speeds;
};



#endif // SIMULATIONWINDOW_H
