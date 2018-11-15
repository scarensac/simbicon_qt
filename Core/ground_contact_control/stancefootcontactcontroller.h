#ifndef STANCEFOOTCONTACTCONTROLLER_H
#define STANCEFOOTCONTACTCONTROLLER_H

#include <vector>
#include <utility>
#include <map>
#include <thread>

#include <MathLib/Vector3d.h>
#include <shark/ObjectiveFunctions/AbstractObjectiveFunction.h>

#include "Core/pose_control/posecontroller.h"
#include "Core/velocity_control/velocitycontroller.h"
#include "Core/medium_control/externalforcefieldscompenser.h"

#include <QMutex>
#include <QSemaphore>


class Character;
class ODEWorld;
class ArticulatedRigidBody;
class RigidBody;
class EstimationWorld;

struct EvalStruct{

    double foot_eval;
    double toes_eval;


    double sum_complete;
    double sum_complete_wo_toes;

    double val_left;
    double val_right;
    double angle_cor;

    double val_back;
    double val_front;
    double angle_sag;

    double toes_on_front;


    EvalStruct(){
        foot_eval=0;
        toes_eval=0;
        sum_complete=0;
        sum_complete_wo_toes=0;

        val_left=0;
        val_right=0;
        angle_cor=0;

        val_back=0;
        val_front=0;
        angle_sag=0;

        toes_on_front=0;
    }

    double value(){return foot_eval;}
};

struct CompleteEval{
    EvalStruct eval_quality;
    EvalStruct eval_distance;

    CompleteEval(){}
};

class StanceFootContactController
{
public:
    //the simulation wold(or worlds for multithread)
    std::vector<EstimationWorld*> m_worlds;
    std::vector<QMutex*> m_worlds_mutex;
    QSemaphore* semaphore;
    //struct to reuse threads
    std::vector<std::thread*> m_reusable_thread;
    std::vector<QSemaphore*> m_semaphore_execute_thread;
    std::vector<QSemaphore*> m_semaphore_end;
    bool m_program_end;


    Character* character;

    //this can be used to try differnet bodie for the estimation world
    //0: the full body
    //1: without the arms
    //2: without arms and head
    //3: without anything above the torso
    //4: only 1 leg
    //5: 2 legs if dual stance 1 leg otherwise.
    int body_config;
    bool print_results;
    bool save_results;

    /// Members for results
    std::vector<Vector3d> m_result_torques;
    std::vector<Vector3d> m_result_torques_parent_coord;


    ///simulation step variables
    double phi;

    /// Memebers for internal computation
    std::vector<Vector3d> m_contact_forces;
    std::vector<Vector3d> m_delta_torques;
    EvalStruct initial_evaluation_details;


    ///evaluation constant
    double limit_cor=0.2;
    double limit_sag=0.2;


    ///members to handle the multi reduction model config
    int m_countdown_switch_to_single_stance;




    ///OTHERS
    int m_current_position;
public:
    PoseController* pose_controller;
    VelocityController* velocity_controller;
    ExternalForceFieldsCompenser* external_force_fields_compenser;

    int count_cma_usage;
    int count_cma_evaluation;
    int count_cma_success;
    int count_cma_usage_global;
    int count_cma_evaluation_global;
    int count_cma_success_global;
    int count_sim_step;
    int count_sim_step_with_problem;

public:
    StanceFootContactController(Character* c);
    virtual ~StanceFootContactController();

    /**
     * @brief getter for the result
     */
    std::vector<Vector3d> result_torque(){return m_result_torques;}

    /**
     * @brief copy the elements that will be  needded for the estimation from the real world to the class world
     */
    void init_estimation_world();

    /**
     * @brief copy the elements that will be  needded for the estimation from the real world to the class world
     */
    void init_new_character_step();


    /**
     * @brief this function handles all the preprocessing necessary at the start of a new simulation step
     */
    void preprocess_simulation_step(double phase);

    /**
     * @brief this function handles all the processing necessary during simulation step
     */
    void simulation_step(std::vector<Vector3d> torques);

    /**
     * @brief this function handles all the postprocessing necessary at the end of a simulation step
     */
    void postprocess_simulation_step(bool new_character_step);

    /**
     * @brief evaluate_simulation_state
     * @return the evaluation
     */


    EvalStruct evaluate_simulation_state(std::vector<Vector3d> f, Quaternion stance_foot_orientation);
    EvalStruct evaluate_distance_to_original(std::vector<Vector3d> f, std::vector<Vector3d> f_initial);

    /**
     * @brief this function find to torques that needs to be applyed by using the cma algorithm
     */
    std::vector<Vector3d> find_cma_solution(std::vector<Vector3d> f_init);

    void create_samples(std::vector<std::vector<Vector3d> >& vect_samples, int nb_samples, std::vector<Vector3d> f_initial);

};


class StanceFootContactControlObjectiveFunction : public shark::SingleObjectiveFunction
{
public:

    StanceFootContactControlObjectiveFunction(StanceFootContactController* i_controller, std::vector<Vector3d> f_init);
    ~StanceFootContactControlObjectiveFunction();

    void proposeStartingPoint(SearchPointType & startingPoint)const {
        //and we set the starting point
        startingPoint=variable_vector;
    }

    std::string name() const { return "StanceFootContactControlObjectiveFunction"; }

    std::size_t numberOfVariables()const{
        return m_dimensions;
    }
    bool hasScalableDimensionality()const{
        return true;
    }
    void setNumberOfVariables(std::size_t numberOfVariables){
        m_dimensions = numberOfVariables;
    }

    ResultType eval(const SearchPointType & input, int offspring_id);

    void clear_sup_data();

private:
    std::size_t m_dimensions;

public:
    //this contain the starting point
    shark::RealVector variable_vector;

    //I need them because I'll norlamise the values so that the cma step do smth ...
    std::vector<Vector3d> sample_min;
    std::vector<Vector3d> sample_max;

    //and a pointer to the controller
    StanceFootContactController* c;
    std::vector<Vector3d> f_initial;

    std::vector<std::pair<int, CompleteEval> > m_sup_data;
    QMutex* m_mutex_sup_data;

    bool handle_toes_joint;

};


void thread_world_execution(StanceFootContactController* c, int selected_world);

#endif // STANCEFOOTCONTACTCONTROLLER_H
