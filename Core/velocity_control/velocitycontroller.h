#ifndef VELOCITYCONTROLLER_H
#define VELOCITYCONTROLLER_H

#include <vector>
#include <MathLib/Vector3d.h>
#include <MathLib/Quaternion.h>


class Character;
class Trajectory;
class TrajectoryComponent;
class Joint;

class VelocityController
{

protected:
    Character* character;

    ///Members for the user's control over the virtual force
    //desired velocity in the sagittal plane
    double velD_sagittal;
    //desired velocity in the coronal plane...
    double velD_coronal;

    //varibales allowing us to control the difficulty to detect the need of recovery steps
    double variation_moy_limit_sagittal;
    double variation_moy_limit_coronal;

    //parameters for the trajectory evolution
    double evo_speed_sagittal;
    double evo_speed_coronal;

    //virtual force limit (absolute values)
    double virt_force_limit;

    //desired heading
    double desired_heading;

    ///Members for the results
    std::vector<Vector3d> _torques;
    bool _need_recovery_steps;

    ///Members for internal computations
    //I'll store the velD trajectory here before I'll use it often
    //also keep the factors that need to be applyed on the curves here
    Trajectory* velD_traj;
    Trajectory* initial_velD_traj;
    bool first_step;
    double limit_switch_sagittal;

public:
    double velD_sagittal_offset;
    double velD_coronal_right_offset;
    double velD_coronal_left_offset;
protected:

    //at each new step I'll update those pointers
    TrajectoryComponent* sagittal_comp;
    TrajectoryComponent* coronal_comp;

    //stored variable for easy acces to the stance
    int stance;
    //this one just store a -1 if we are on left stance and +1 on right stance
    int stance_coefficient;

    //some variables used for the comunication between the storing function and the learning function
    std::vector<double> vel_sagittal;
    std::vector<double> vel_coronal;
    double cumul_speed_z;
    double cumul_speed_x;
    unsigned int times_vel_sampled;
    double last_phi;


    //variables used for the storing function that need to be reseted at the end a each step
    double cur_phi_limit_z;
    int cur_knot_nbr_z;
    double cur_phi_limit_x;
    int cur_knot_nbr_x;


    //variables for continuous storing in the adapt function
    std::vector<double> previous_speeds_z;
    std::vector<double> previous_speeds_x;

    //storing the previous offsets
    std::vector<double> previous_offset_z;
    std::vector<double> previous_offset_x_left;
    std::vector<double> previous_offset_x_right;


    Vector3d virtual_force;

    ///memebrs for previously static values
    int prev_result_sag;
    int prev_result_cor;
    std::vector<double> vec_influence_previous;

public:
    Vector3d last_virt_force;
    Vector3d last_virt_force_cumul;
    Vector3d last_virt_force_signed;
    Vector3d last_virt_force_cumul_signed;


    Vector3d previous_virt_force_signed;


public:
    VelocityController(Character* c);
    virtual ~VelocityController();

    /**
     * @brief used to initialise the pointers on the velD_trajecotry
     * Also allows to give initialization valuesfor the offsets
     */
    void init_velD_trajectory(Trajectory* new_traj,double sagittal_offset=0,double coronal_offset_right=0,double coronal_offset_left=0);

    void restart();

    /**
     * @brief simple getters for results
     */
    std::vector<Vector3d>& torques(){return _torques;}
    bool need_recovery_steps(){return _need_recovery_steps;}
    Vector3d avg_virt_force_signed(){return last_virt_force_signed;}
    Vector3d avg_virt_force(){return last_virt_force;}

    /**
     * @param step_offset number of step from the last one (so default value means the last step)
     */
    double prev_step_sag_speed(int step_offset=0){
        return previous_speeds_z.at(previous_speeds_z.size()-1-step_offset);
    }
    double prev_step_cor_speed(int step_offset=0){
        return previous_speeds_x.at(previous_speeds_x.size()-1-step_offset);
    }

    /**
     * @brief non functionnal method
     */
    void load_trajectory_from_file();//impossible to do for now (need json version of parser)

    /**
     * @brief This method prepare the system for the next character's step.
     * This method has to be called at the very start of the first simulation step following a foot strike
     */
    void init_new_character_step();

    /**
     * @brief this method is used to observe the velocity pattern of the character's CM during the step
     * This method has to be called at every time step of the simulation
     * This method also integrate the variations of orientation
     * @param cur_phi current phase
     * @param v current velocity of the character's CM
     */
    void store_velocity(double cur_phi, Vector3d v);

    /**
     * @brief this function evolve the learning curve depending on the velocity values noted during the step
     * This method has to be called at the end of each character's step before changing the character's stance
     * @param v current velocity of the character's CM
     */
    void adapt_learning_curve(Vector3d v, double phi_last_step);

    /**
     * @brief this function compute the torques resulting from the virtual force
     * @param v current velocity of the character's CM
     */
    void apply_virtual_force(const Vector3d &v);

    /**
     * @brief this function is used to detec if the last virtual force used is near the maximum autorised
     * @param coronal set to true if the coronal limit is reached
     * @param sagittal set to true if the sagittal limit is reached
     * @param limit ratio of the maximum autorised virtual force which need to be reached to trigger the detection
     */
    void virtual_force_near_limit(bool& coronal, bool& sagittal, double limit=0.8);

    /**
     * @brief this function is used to detec if the last virtual force used is near the maximum autorised
     * @param coronal set to true if the coronal limit is reached
     * @param sagittal set to true if the sagittal limit is reached
     * @param limit ratio of the maximum autorised virtual force which need to be reached to trigger the detection
     */
    void has_evolved_during_last_step(bool& coronal, bool& sagittal, double limit=0.8);


    void lower_force_intensity(double coef);
protected:
    void update_components();

    int adapt_learning_component(TrajectoryComponent* affected_component, std::vector<double> & vel_vector,
                                  double sup_point_val, double variation_moy_limit, bool &recovery_step_asked,
                                  double &avg_signed_variation, double variation_ratio_used, double reset_value, int previous_result,
                                 double zero_value, bool force_learning=false);

    /**
        those functions are here to make access to the diferent component of the velD easier
    */
    inline TrajectoryComponent* velD_sagittal_component();
    inline TrajectoryComponent* velD_coronal_component(int stance);

    /**
        this function get the desired sagital velocity (affected by the variation trajectory and the offset)
    */
    inline double get_effective_desired_sagittal_velocity(double phi);

    /**
        this function get the desired coronal velocity (affected by the variation trajectory and the offset)
    */
    inline double get_effective_desired_coronal_velocity(double phi);

    /**
        This method is used to compute the force that the COM of the character should be applying.
    */
    Vector3d compute_virtual_force(const Vector3d& v);

    /**
        this method compute the torques associated to one leg
    */
    void compute_leg_torques(Joint* hip, double leg_ratio);

    /**
    This method returns performes some pre-processing on the virtual torque. The torque is assumed to be in world coordinates,
    and it will remain in world coordinates.
    */
    void preprocess_ankle_virtual_torque(Joint *ankle, Vector3d *ankleVTorque);


public:
    /**
     * @brief change_desired_heading
     * this function have to be called evrytime we want the hange the desired heading
     */
    void change_desired_heading(double new_heading, bool update_in_step_velocity);

    bool is_currently_rotating(){return m_is_currently_rotating;}
    int get_count_steps(){return step_count;}

private:
    //all those members are specials
    //they are old static variables in functions but I could not find what was the problem with them
    //so I put them here so I'm ensured they are reinitialized between each evaluations ...
    //VelocityController::adapt_learning_curve
    double avg_speed_x_previous;
    double previous_offset_delta;
    double prev_sag_offset;
    double prev_prev_sag_offset;
    int step_count;
    int count_chained_steps;
    bool m_is_currently_rotating;
    int end_recovery_step_id;
    int start_rotating_step_id;

    //VelocityController::change_desired_heading
    double current_mult;
    double old_heading;
    double target_heading;
    double start_phi;

    //this memebr is to handle the modification of the desired velocity
    int count_step_force_learn;
};

#endif // VELOCITYCONTROLLER_H
