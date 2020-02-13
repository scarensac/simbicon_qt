#ifndef POSECONTROLLER2_H
#define POSECONTROLLER2_H

#include <vector>
#include <MathLib/Vector3d.h>


///TODO remove those includes
#include "Physics/joints/Joint.h"

using PartialState=std::pair<Vector3d,Quaternion>;


class Character;
class SimBiConState;
class SwingFootController;
class VelocityController;

///TODO remove this
class Trajectory;

class PoseController
{
protected:
    ///SUBMODULES
    SwingFootController* swing_foot_controller;


    ///Members for results
    std::vector<Vector3d> _torques;

    ///Internal members
    Character* character;
    //this is a collection of the states that are used in the controller
    std::vector<SimBiConState*> _states;

    ///Members set once everysteps
    //this is the index of the controller that is currently active
    int _state_idx;



    ///TODO think about those members (I mean everything after until another blue msg)
    double stanceHipDamping;
    double stanceHipMaxVelocity;
    double rootPredictiveTorqueScale;

    //keep a copy of the initial character state, starting stance and file that contains the character's initial state
    int startingState;
    int startingStance;
    char initialBipState[100];

    Quaternion qRootD;
    Vector3d qVelD;
    ControlParams rootControlParams;


    ///old static members
    double old_phase;


    ///TODO remove those members
    std::vector<ControlParams> controlParams;
    std::vector<ControlParams> controlParams_initial;


public:



    ///TODO remove this members
    Trajectory* velD_traj;

    //this one will be unsused when the control param is transferd to inside the joints
    bool is_controlled(int i){return controlParams[i].controlled;}
    std::vector<ControlParams>& get_control_params(){ return controlParams;}
    ControlParams& get_root_control_params(){ return rootControlParams;}


    ///NOW normal members

    PoseController(Character* c);
    virtual ~PoseController();

    const std::vector<Vector3d>& torques(){return _torques;}
    double ipm_alt_sagittal();
    int starting_stance(){return startingStance;}

    void restart();

    void init_character_step(VelocityController *vel_control);


    void preprocess_simulation_step(double phase, Vector3d v_com);

    void simulation_step(double phase);

    int end_simulation_step(double phi);


    void track_hand_target();


    /**
        This method loads all the pertinent information regarding the simbicon controller from a file.
    */
    void load_from_file(char* fName);


    /**
        This method is used to compute the torques that are to be applied at the next step.
    */
    void compute_torques();

    /**
     * @brief this method compute the torque for a given joint.
     */
    void compute_torque_for_joint(int jIdx, bool relative_to_char_frame);


    /**
     * @brief this method compute the torque for a given joint given a know dq (variation orientation) desired
     */
    Vector3d compute_torque_for_joint_from_angular_displacement(int jIdx, Vector3d angular_displacement);

    /**
        This method is used to compute the PD torque, given the current relative orientation of two coordinate frames (child and parent),
        the relative angular velocity, the desired values for the relative orientation and ang. vel, as well as the virtual motor's
        PD gains. The torque returned is expressed in the coordinate frame of the 'parent'.
    */
    static Vector3d compute_pd_torque(const Quaternion& qRel, const Quaternion& qRelD, const Vector3d& wRel,
                                      const Vector3d& wRelD, ControlParams* cParams, bool to_test=false);




    /**
        This method is used to scale and apply joint limits to the torque that is passed in as a parameter. The orientation that transforms
        the torque from the coordinate frame that it is currently stored in, to the coordinate frame of the 'child' to which the torque is
        applied to (it wouldn't make sense to scale the torques in any other coordinate frame)  is also passed in as a parameter.
    */
    static void scale_and_limit_torque(Vector3d* torque, ControlParams* cParams, const Quaternion& qToChild);

    /**
        This method is used to apply joint limits to the torque passed in as a parameter. It is assumed that
        the torque is already represented in the correct coordinate frame
    */
    static void limit_torque(Vector3d* torque, ControlParams* cParams);
    void limit_torque(Vector3d* torque, int joint_idx);

    /**
    This method is used to ensure that each RB sees the net torque that the PD controller computed for it.
    Without it, an RB sees also the sum of -t of every child.
    */
    void bubbleup_torques();

    double state_time();
    inline int state_idx(){return _state_idx;}

    void target_pose(std::vector<double> &pose);

    void set_character_to_desired_pose(Character* c);

    void set_swing_leg_rotation_angle(double angle);
    void set_leg_forward_to_orientation(Quaternion orientation, bool is_swing, float influence=1.0f);

    void tracking_pose(std::vector<double>& trackingPose, double phiToUse);


    void root_pose_params(Quaternion& q, ControlParams& cp){
        q=qRootD;
        cp=rootControlParams;
    }

    /**
        This method is used to set the current FSM state of the controller to that of the index that
        is passed in.
    */
    void set_fsm_state(int index);

    void set_stance(int nstance);

    bool check_fsm_state();

    SimBiConState* get_fsm_state(int idx=-1){
        if (idx=-1){
            return state(state_idx());
        }
        return state(idx);
    }

    void compute_root_orientation_torques(Vector3d& stance_hip_torque, Vector3d& swing_hip_torque,
                                          const Vector3d& pelvis_torso_torque);

    void writeToFile(FILE *f, char *stateFileName);

    /**
        this method override previously loaded gains with the ones from a another file
    */
    void read_gains(char* fName);

    /**
        this method override previously loaded gains with the ones from a another file
    */
    void read_gains(std::string input_file);

protected:
    /**
      getters
      */
    SimBiConState* state(uint idx ) {
        if( idx >= _states.size()||idx<0 ) return NULL;
        return _states[idx];
    }



    /**
        This method is used to read the gain coefficients, as well as max torque allowed for each joint
        from the file that is passed in as a parameter.
    */
    void read_gains(FILE* f);


    /**
        This method is used to write the gain coefficients, as well as max torque allowed for each joint
        from the file that is passed in as a parameter.
    */
    void write_gains(FILE* f);



    /**
        This method is used to parse the information passed in the string. This class knows how to read lines
        that have the name of a joint, followed by a list of the pertinent parameters. If this assumption is not held,
        then classes extended this one are required to provide their own implementation of this simple parser
    */
    void parse_gain_line(char* line);


    /**
        This method is used to resolve the names (map them to their index) of the joints
    */
    void resolve_joints(SimBiConState* state);


    /**
        This method should be called when the controller transitions to this state.
    */
    void transition_to_state(int idx);

    /**
    This method is used to compute the target orientations using the current FSM
    */
    void evaluate_joints_target(double phi);


    /**
     * @brief control_desired_heading
     * this method update the desired heading. it is currently built so that any given modification of the desired heading
     * is smoothed a little so that the controller do not see abrupt modifications (that could lead to high torques values)
     */
    void control_desired_heading(double phase);

private:
    //all those members are specials
    //they are old static variables in functions but I could not find what was the problem with them
    //so I put them here so I'm ensured they are reinitialized between each evaluations ...
    //PoseController::init_character_step
    double velDsag_old;
    double liquid_lvl_old;

    //PoseController::preprocess_simulation_step
    double old_heading;
    double target_heading;
    double angle_to_target_heading;
    double start_phi_change_heading;
    double desired_heading_pelvis;
    double desired_heading_swing_foot;
    double desired_heading_stance_foot;
    double target_heading_mitigation_coefficient;
    bool first_pass_this_char_step;



    //PoseController::simulation_step
    bool old_swing_foot_is_relative;
    Quaternion target_heading_stance_foot;

    //PoseController::simulation_step
    double start_phi_change_swing_leg_heading;

    //PoseController::compute_torques
    std::vector<Vector3d> vect_speeds[50];

    //PoseController::set_swing_leg_rotation_angle
    double old_val;
    bool trigerred_once;
};

#endif // POSECONTROLLER_H
