#ifndef ESTIMATIONWORLD_H
#define ESTIMATIONWORLD_H


#include <vector>
#include <map>

#include <MathLib/Vector3d.h>
#include <MathLib/Quaternion.h>

#include <shark/ObjectiveFunctions/AbstractObjectiveFunction.h>

#include "Physics/ContactPoint.h"

class Character;
class ODEWorld;
class ArticulatedRigidBody;
class RigidBody;
class PoseController;

struct EasyODEWorld{
    ODEWorld* m_world;
    ArticulatedRigidBody* m_toes;
    ArticulatedRigidBody* m_foot;
    ArticulatedRigidBody* m_lowerleg;
    ArticulatedRigidBody* m_upperleg;
    std::vector<ArticulatedRigidBody*> m_original_bodies;

    EasyODEWorld(){
        m_world=NULL;
        m_toes=NULL;
        m_foot=NULL;
        m_lowerleg=NULL;
        m_upperleg=NULL;
    }

    ~EasyODEWorld(){
        if (m_world!=NULL){
            delete m_world;
        }
        m_original_bodies.clear();
    }
};

class EstimationWorld
{
    Character* character;

    int body_config;
    bool m_use_complex_fusing;

    /// Members for results
    std::vector<Vector3d> m_result_torques;


    /// Easy wolrds databank
    /// the number and how the data is actualy stored inside depends on the execution
    /// but a simple general rule is that od number worlds are right side ones
    std::vector<EasyODEWorld*> m_worlds_databank;


    /// those are juste  a shortcut to remove all the "if" checking what stance I'm in
    ODEWorld* m_stance_world;
    ArticulatedRigidBody* m_stance_toes;
    ArticulatedRigidBody* m_stance_foot;
    ArticulatedRigidBody* m_stance_lowerleg;
    ArticulatedRigidBody* m_stance_upperleg;
    std::vector<ArticulatedRigidBody*> m_stance_original_bodies;


    ///simulation step variables
    std::vector<Vector3d> m_torques_base;
    Vector3d COM_force;
    Vector3d COM_pt;
    Vector3d COM_velocity;
    Vector3d COM_force_2;//for config 6
    Vector3d COM_pt_2;//for config 6


    /// Memebers for internal computation
    std::vector<Vector3d> m_contact_forces;
    std::vector<Vector3d> m_delta_torques;

public:
    EstimationWorld(Character* c, int i_body_config);

    virtual ~EstimationWorld();

    void generate_arbs(int config, int pos_in_databank);

    /**
     * @brief getters
     * @return
     */
    ODEWorld* get_stance_world(){return m_stance_world;}

    /**
     * @brief copy the elements that will be  needded for the estimation from the real world to the class world
     */
    void init_new_character_step();

    void switch_world(int easy_world_id);

    /**
     * @brief switch_to_single_stance_estimation
     * is used so that I cna switch to the signle stance estimation if I'm in a config handle
     * single and double stance separately.
     */
    void switch_to_single_stance_estimation();

    /**
     * @brief switch_to_double_stance_estimation
     * is used so that I cna switch to the double stance estimation if I'm in a config handle
     * single and double stance separately.
     */
    void switch_to_dual_stance_estimation();



    /**
     * @brief run_simulation
     */
    void run_simulation(double start_phi);


    void set_torques_base(const std::vector<Vector3d>& t){m_torques_base=t;}
    void set_delta_torques(const std::vector<Vector3d>& t){m_delta_torques=t;}

    /**
     * @brief this function prepare the system (compute some variables, typicaly the simulated force for the rest of the character)
     */
    void estimate_missing_bodies_impact();


    std::vector<Vector3d>  get_contact_forces(){return m_contact_forces;}
    Quaternion get_stance_foot_orientation();


    void load_real_world_position_to_estimation_world();


    void load_ode_data();
};


#endif // ESTIMATIONWORLD_H
