#ifndef SWING_FOOT_CONTROLLER_H
#define SWING_FOOT_CONTROLLER_H

#include <MathLib/Vector3d.h>


///TODO move the trajectory definition so that I don't have this fucking include
#include "Core/SimBiConState.h"

class Character;
class VelocityController;

class SwingFootController
{
protected:
    Character* character;

    ///Members initialised when loading the application
    //some members for IPM
    double leg_length;
    //    double ankleBaseHeight;
    //    double stepHeight;

    //user defined swing foot trajectory
    Trajectory* swing_foot_traj;


    ///Members initialized at the start of the character's step
    bool _is_recovery_step;
    double velD_sagittal;
    double velD_coronal;
    double cur_state_time;

    Quaternion heading;

    //this variable store the current value of the ipm_alteration system
    double ipm_alt_sagittal;
    double ipm_alt_coronal_left;
    double ipm_alt_coronal_right;

    //desired width between the 2 feet of the characte at the end of the current step
    double coronal_step_width;

    ///Members modified at each time steps
    double velD_sagittal_current_heading;
    double velD_coronal_current_heading;



    ///Members used internally
    //just store if the ipm is currently used
    bool ipm_active;

    //store the phase for easier use
    double phi;
    //also keep the com speed
    Vector3d v;

    //we need to keep track of the position of the swing foot is was when we started using the ipm
    Point3d swing_foot_start_pos;
    double phase_start_pos;

    //this is a desired foot trajectory that we may wish to follow, expressed separately, for the 3 components,
    //and relative to the current location of the CM
    Trajectory1D swing_foot_trajectory_sagittal;
    Trajectory1D swing_foot_trajectory_coronal;


    //I just store the last computed position...
    Point3d desired_position;


    //we'll keep track of the vector that represents each step taken. The vector will be represented in the 'relative' world coordinate frame
    //Vector3d lastStepTaken;
    //this is the position of the foot at the previous state
    //    Point3d lastFootPos;

    //I'll store the COM displacement on the previouses steps so I can estimate the deviation from
    //the correct movement direction
    //    Vector3d com_displacement_last_step;
    //    Vector3d com_displacement_previous_to_last_step;



public:
    SwingFootController(Character* c);
    virtual ~SwingFootController();

    double get_ipm_alt_sagittal(){return ipm_alt_sagittal;}


    /**
     * I need it for quick tests because I need the value from the pose ontrolleur BUT DO NOT USE IT IF POSSIBLE
     * if realy needed add the variable to the pose controlers globals
     */
    double get_cur_state_time(){return cur_state_time;}

    void restart();

    /**
     * @brief this function is used to set the trajectory of the swing foot
     * @param user_traj pointer to the userr's trajectory
     * @return
     */
    void user_swing_foot_traj(Trajectory* user_traj){swing_foot_traj=user_traj;}

    /**
     * @brief this function is used to initialize some parameters
     * this function should be called at the start of ever new character step
     */
    void init_new_character_step(bool is_recovery_step, double cur_state_time,
                                 VelocityController *vel_control);


    /**
     * @brief this function handles all the preprocessing necessary at the start of a new simulation step
     */
    void preprocess_simulation_step(double phase, Vector3d v_com);


    /**
     * @brief this method check if the IPM is needed and launch the computation of th swing foot position
     */
    void simulation_step(Quaternion desired_heading_pelvis);

    /**
     * @brief this funtion will return a simple bolean telling us if the IPM result will be used or if they will be overriden
     * @return true if the IPM should be used
     */
    bool ipm_needed();

    /**
     * @brief only tell us if we are during the early phase of a character step for the systems that need it
     * @return
     */
    bool is_early_step(double limit = 0.2);

    Point3d get_swing_foot_position(){
        return desired_position;
    }

protected:

    
    /**
     * @brief this function compute the swing foot location by using an ipm
     */
    Vector3d ipm_compute_swing_foot_location(const Point3d& comPos, double phase);

    /**
     * @brief returns the required stepping location, as predicted by the inverted pendulum model. The prediction is made
        on the assumption that the character will come to a stop by taking a step at that location. The step location
        is expressed in the character's frame coordinates.
       @return
     */
    Vector3d ipm_compute_step_location();

    /**
    modify the coronal location of the step so that the desired step width results.
    */
    double ipm_adjust_coronal_step_location(double IPPrediction, double phase);

    /**
     * @brief compute the swing foot height for the enx time step
     */
    void adjust_step_height();

    /**
    this function override the results of the ipm with the specified results
    */
    void use_specified_swing_foot_location(double cur_phi, double future_phi);

    /**
     * @brief user_specified_swing_foot_traj_xz_exist
     * @return true is the user has defined an x and z trajectory for the swing foot
     */
    bool user_specified_swing_foot_traj_xz_exist();

    /**
    This method is used to compute the target angles for the swing hip and swing knee that help
    to ensure (approximately) precise foot-placement control.
    */
    void compute_swing_leg_target(double dt, Quaternion desired_heading_pelvis);

    /**
    This method returns a target for the location of the swing foot, based on some state information. It is assumed that the velocity vel
    is expressed in character-relative coordinates (i.e. the sagittal component is the z-component), while the com position, and the
    initial swing foot position is expressed in world coordinates. The resulting desired foot position is also expressed in world coordinates.
    */
    Point3d swing_foot_target_location(double t, const Point3d& com, const Quaternion& charFrameToWorld);

    /**
    This method is used to compute the desired orientation and angular velocity for a parent RB and a child RB, relative to the grandparent RB and
    parent RB repsectively. The input is:
    - the index of the joint that connects the grandparent RB to the parent RB, and the index of the joint between parent and child

    - the distance from the parent's joint with its parent to the location of the child's joint, expressed in parent coordinates

    - two rotation normals - one that specifies the plane of rotation for the parent's joint, expressed in grandparent coords,
    and the other specifies the plane of rotation between the parent and the child, expressed in parent's coordinates.

    - The position of the end effector, expressed in child's coordinate frame

    - The desired position of the end effector, expressed in world coordinates

    - an estimate of the desired position of the end effector, in world coordinates, some dt later - used to compute desired angular velocities
    */
    void compute_inversed_kinematics(int parentJIndex, int childJIndex, const Vector3d& parentAxis, const Vector3d& parentNormal,
                                     const Vector3d& childNormal, const Vector3d& childEndEffector, const Point3d& wP,
                                     bool computeAngVelocities, const Point3d& futureWP, double dt);


    void compute_ipm_alteration(VelocityController* vel_control);

private:
    double phi_end_previous_step;


    //all those members are specials
    //they are old static variables in functions but I could not find what was the problem with them
    //so I put them here so I'm ensured they are reinitialized between each evaluations ...
    //SwingFootController::compute_ipm_alteration
    bool evolve_sagittal;
    int steps_left_sag;

};

#endif // IPMCONTROLLER_H
