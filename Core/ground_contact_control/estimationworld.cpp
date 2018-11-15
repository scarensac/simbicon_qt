#include "estimationworld.h"


#include <iostream>
#include <fstream>
#include <sstream>

#include "Core/Character.h"
#include "Core/SimGlobals.h"
#include "Physics/ODEWorld.h"
#include "Physics/rb/ArticulatedRigidBody.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include "Utils/Utils.h"


#include "Core/pose_control/posecontroller.h"
#include <QMutex>
#include <cstdio>


EstimationWorld::EstimationWorld(Character *c, int i_body_config)
{
    m_use_complex_fusing=false;
    body_config=i_body_config;
    character=c;

    int nb_worlds=2;
    if (body_config==5){
        nb_worlds=4;
    }

    for (int i=0;i<nb_worlds;++i){
        ODEWorld* w= new ODEWorld(true);
        EasyODEWorld* easy_world= new EasyODEWorld();
        easy_world->m_world=w;

        m_worlds_databank.push_back(easy_world);
    }

    //first we create the ground
    std::vector<RigidBody*> vect_rbs;
    ODEWorld * real_world=dynamic_cast<ODEWorld*>(SimGlobals::getRBEngine());
    vect_rbs.push_back(real_world->getRBByName("ground"));
    for (int i=0;i<nb_worlds;++i){
        m_worlds_databank[i]->m_world->loadRBsFromList(vect_rbs);
    }

    if (body_config==5){
        //create the dual stance
        generate_arbs(3,0);

        //create the single stance
        generate_arbs(4,2);

    }else{
        generate_arbs(body_config,0);
    }

    init_new_character_step();

}

void EstimationWorld::generate_arbs(int config, int pos_in_databank)
{

    //and we add the lowerleg and the foot
    ArticulatedRigidBody* left_foot_arb=dynamic_cast<ArticulatedRigidBody*>(character->stance_foot());
    ArticulatedRigidBody* right_foot_arb=dynamic_cast<ArticulatedRigidBody*>(character->swing_foot());

    if (!character->is_left_stance()){
        ArticulatedRigidBody* temp=left_foot_arb;
        left_foot_arb=right_foot_arb;
        right_foot_arb=temp;
    }

    ArticulatedRigidBody* left_lowerleg_arb=left_foot_arb->parent_joint()->parent();
    ArticulatedRigidBody* left_toes_arb=left_foot_arb->child_joints()[0]->child();
    ArticulatedRigidBody* left_upperleg_arb=left_lowerleg_arb->parent_joint()->parent();

    ArticulatedRigidBody* right_lowerleg_arb=right_foot_arb->parent_joint()->parent();
    ArticulatedRigidBody* right_toes_arb=right_foot_arb->child_joints()[0]->child();
    ArticulatedRigidBody* right_upperleg_arb=right_lowerleg_arb->parent_joint()->parent();


    std::vector<ArticulatedRigidBody*> vect_arbs;
    if (config==0){
        vect_arbs.push_back(character->getRoot());
        for (int i=0;i<character->getJointCount();++i){
            vect_arbs.push_back(character->getJoint(i)->child());
        }
    }else if (config==1){
        vect_arbs.push_back(character->getRoot());
        for (int i=0;i<character->getJointCount();++i){
            std::string n=character->getJoint(i)->child()->name();
            if (n.find("arm") == std::string::npos){
                vect_arbs.push_back(character->getJoint(i)->child());
            }
        }
        //*/
    }else if (config==2){
        vect_arbs.push_back(character->getRoot());
        for (int i=0;i<character->getJointCount();++i){
            std::string n=character->getJoint(i)->child()->name();
            if (n.find("arm") == std::string::npos &&
                    n.find("head") == std::string::npos){
                vect_arbs.push_back(character->getJoint(i)->child());
            }
        }
        //*/
    }else if (config==3){
        vect_arbs.push_back(character->getRoot());
        for (int i=0;i<character->getJointCount();++i){
            std::string n=character->getJoint(i)->child()->name();
            if (n.find("arm") == std::string::npos &&
                    n.find("head") == std::string::npos &&
                    n.find("torso") == std::string::npos){
                vect_arbs.push_back(character->getJoint(i)->child());
            }
        }
        //*/
    }



    //left world
        {
            if (config==4){
            vect_arbs.clear();
            vect_arbs.push_back(character->getRoot());
            for (int i=0;i<character->getJointCount();++i){
                std::string n=character->getJoint(i)->child()->name();
                if (n.find("arm") == std::string::npos &&
                        n.find("head") == std::string::npos &&
                        n.find("torso") == std::string::npos &&
                        n.find("r")  != 0){
                    vect_arbs.push_back(character->getJoint(i)->child());
                }
            }
            }

            if (config==6){
                vect_arbs.clear();
                vect_arbs.push_back(character->getRoot());
                for (int i=0;i<character->getJointCount();++i){
                    std::string n=character->getJoint(i)->child()->name();
                    if (n.find("arm") == std::string::npos &&
                            n.find("head") == std::string::npos &&
                            n.find("r")  != 0){
                        vect_arbs.push_back(character->getJoint(i)->child());
                    }
                }
            }

            EasyODEWorld& easy_world=*(m_worlds_databank[pos_in_databank+0]);
            easy_world.m_world->loadARBsFromList(vect_arbs);
            easy_world.m_original_bodies=vect_arbs;
            //now for an easier use I'll keep pointer on the object of the new world
            //I use the fact that they have the ame name as in the old world.
            //though their name won't mean anything later
            easy_world.m_foot=easy_world.m_world->getARBByName(left_foot_arb->name());
            easy_world.m_lowerleg=easy_world.m_world->getARBByName(left_lowerleg_arb->name());
            easy_world.m_toes=easy_world.m_world->getARBByName(left_toes_arb->name());
            easy_world.m_upperleg=easy_world.m_world->getARBByName(left_upperleg_arb->name());

        }

        //right world
        {
            if (config==4){
            vect_arbs.clear();
            vect_arbs.push_back(character->getRoot());
            for (int i=0;i<character->getJointCount();++i){
                std::string n=character->getJoint(i)->child()->name();
                if (n.find("arm") == std::string::npos &&
                        n.find("head") == std::string::npos &&
                        n.find("torso") == std::string::npos &&
                        n.find("l")  != 0){
                    vect_arbs.push_back(character->getJoint(i)->child());
                }
            }
            }

            if (config==6){
                vect_arbs.clear();
                vect_arbs.push_back(character->getRoot());
                for (int i=0;i<character->getJointCount();++i){
                    std::string n=character->getJoint(i)->child()->name();
                    if (n.find("arm") == std::string::npos &&
                            n.find("head") == std::string::npos &&
                            n.find("l")  != 0){
                        vect_arbs.push_back(character->getJoint(i)->child());
                    }
                }
            }

            EasyODEWorld& easy_world=*(m_worlds_databank[pos_in_databank+1]);
            easy_world.m_world->loadARBsFromList(vect_arbs);
            easy_world.m_original_bodies=vect_arbs;

            //now for an easier use I'll keep pointer on the object of the new world
            //I use the fact that they have the ame name as in the old world.
            //though their name won't mean anything later
            easy_world.m_foot=easy_world.m_world->getARBByName(right_foot_arb->name());
            easy_world.m_lowerleg=easy_world.m_world->getARBByName(right_lowerleg_arb->name());
            easy_world.m_toes=easy_world.m_world->getARBByName(right_toes_arb->name());
            easy_world.m_upperleg=easy_world.m_world->getARBByName(right_upperleg_arb->name());
        }

}

EstimationWorld::~EstimationWorld()
{
    m_result_torques.clear();

    for (int i=0;i<m_worlds_databank.size();++i){
        delete m_worlds_databank[i];
        m_worlds_databank[i]=NULL;
    }
    m_worlds_databank.clear();

    m_torques_base.clear();

    m_contact_forces.clear();

    m_delta_torques.clear();

}



void EstimationWorld::init_new_character_step()
{
    switch_to_dual_stance_estimation();
}

void EstimationWorld::switch_world(int easy_world_id)
{

    const EasyODEWorld* w=m_worlds_databank[easy_world_id];

    m_stance_world=w->m_world;
    m_stance_foot=w->m_foot;
    m_stance_lowerleg=w->m_lowerleg;
    m_stance_toes=w->m_toes;
    m_stance_upperleg=w->m_upperleg;
    m_stance_original_bodies=w->m_original_bodies;
}

void EstimationWorld::switch_to_single_stance_estimation()
{
    if (body_config==5){
        int easy_world_id=-1;
        if (character->is_left_stance()){
            easy_world_id=2;
        }else{
            easy_world_id=3;
        }

        switch_world(easy_world_id);
    }
}

void EstimationWorld::switch_to_dual_stance_estimation()
{
    int easy_world_id=-1;
    if (character->is_left_stance()){
        easy_world_id=0;
    }else{
        easy_world_id=1;
    }

    switch_world(easy_world_id);

}






void EstimationWorld::run_simulation(double start_phi)
{
    //load_ode_data();

    load_real_world_position_to_estimation_world();

    //advance the simulation
    int nbr_forward_sim=1;
    //Vector3d* force_foot_real_world=character->force_stance_foot();
    estimate_missing_bodies_impact();

    for (int i=0;i<nbr_forward_sim;++i){
        Vector3d t;
        Vector3d delta_t,delta_t_w;

        //we load the torques computed by the normal system

        //        if (m_stance_world->getJointCount()!=m_torques_base.size()){
        //            std::ostringstream oss;
        //            oss<<"StanceFootContactController::run_simulation: WTF ya des nbr de joint differents ...";
        //            oss<<"old: "<<m_torques_base.size()<<"    new: "<<m_stance_world->getJointCount();
        //            std::cout<<oss.str();
        //        }
        for (int k=0;k<m_stance_world->getJointCount();++k){;
            Joint* j=m_stance_world->getJoint(k);
            j->torque(m_torques_base[j->idx()]);
        }


        int cur_id=0;

        //apply the deltas
        //if we have only 2 variables that means the toes were ignored
        if (m_delta_torques.size()>2){
            t=m_stance_toes->parent_joint()->torque();
            delta_t=m_delta_torques[cur_id++];
            delta_t_w=m_stance_toes->parent_joint()->parent()->getWorldCoordinates(delta_t);
            t+=delta_t_w;
            m_stance_toes->parent_joint()->torque(t);
        }

        t=m_stance_foot->parent_joint()->torque();
        delta_t=m_delta_torques[cur_id++];
        delta_t_w=m_stance_foot->parent_joint()->parent()->getWorldCoordinates(delta_t);
        t+=delta_t_w;
        m_stance_foot->parent_joint()->torque(t);

        t=m_stance_lowerleg->parent_joint()->torque();
        delta_t=m_delta_torques[cur_id++];
        delta_t_w=m_stance_lowerleg->parent_joint()->parent()->getWorldCoordinates(delta_t);
        t+=delta_t_w;
        m_stance_lowerleg->parent_joint()->torque(t);


        //WARNING the fixed joint must be created AFTER setting the new positoons in the engine
        ArticulatedRigidBody* root_bdy=NULL;
        Vector3d sup_torque=Vector3d(0,0,0);

        /*
        if(body_config==3||body_config==4){
            AbstractRBEngine* rbc = SimGlobals::getRBEngine();
            root_bdy= rbc->getARBByName("pelvis");
            for (int i=0;i<root_bdy->child_joints().size();++i){
                std::string n=root_bdy->child_joints()[i]->child()->name();
                if (n.find("torso") != std::string::npos){
                    sup_torque+=m_torques_base[root_bdy->child_joints()[i]->idx()];
                }
                if (body_config==4){
                    if (n.find("Upperleg") != std::string::npos){
                        if (n!=m_stance_upperleg->name()){
                            sup_torque+=m_torques_base[root_bdy->child_joints()[i]->idx()];
                        }
                    }
                }
            }
            //this need to be done since the torques are the child torques
            sup_torque*=-1;
        }
        //*/

        m_stance_world->sendDataToEngine(root_bdy,sup_torque);

        //apply the com force
        if (!m_use_complex_fusing){
            if (body_config!=0){
                //*
                if (body_config==6){

                }else{
                    if (body_config==6){

                        RigidBody* root;
                        root= m_stance_world->getARBByName("torso");
                        m_stance_world->applyForceToWorldPos(root,
                                                             COM_force*1,
                                                             COM_pt);

                        root= m_stance_world->getARBByName("pelvis");
                        m_stance_world->applyForceToWorldPos(root,
                                                             COM_force_2*1,
                                                             COM_pt_2);


                    }else{
                        RigidBody* root;
                        if (body_config==1||body_config==2){
                            root= m_stance_world->getARBByName("torso");
                        }
                        if (body_config==3||body_config==4||body_config==5){
                            root= m_stance_world->getARBByName("pelvis");
                        }
                        //*
                        m_stance_world->applyForceToWorldPos(root,
                                                             COM_force*1,
                                                             COM_pt);
                    }
                }
                //*/
                /*
                ForceStruct Fs;
                Fs.F=COM_force;
                Fs.pt=COM_current_pt;
                SimGlobals::vect_forces.push_back(Fs);
//*/
                //                std::cout<<COM_force.y<<std::endl;

                //*/
                /*
            std::ostringstream oss;
            Vector3d bc=root->getCMPosition();
            oss<<"body  com: "<<bc.x<<" "<<bc.y<<" "<<bc.z;
            oss<<std::endl<<"force com: "<<COM_current_pt.x<<" "<<COM_current_pt.y<<" "<<COM_current_pt.z;
            oss<<std::endl<<"force com: "<<COM_force.x<<" "<<COM_force.y<<" "<<COM_force.z;
            std::cout<<oss.str();
            //*/
            }
        }

        m_stance_world->advanceInTime(SimGlobals::dt);

        /*try{
        }catch(char* c){
            std::string msg(c);
            std::cout<<msg;
            return;
        }catch(const char* c){
            std::string msg(c);
            std::cout<<msg;
            return;
        }catch(...){
            std::cout<<"failed catching it";
            return;
        }
        //*/
        //        w_force_app_point+=eff_com_vel*SimGlobals::dt;


        m_stance_world->readDataFromEngine();
    }

    //rmv the fixed joint
    //        m_world_stance_foot->destroyODEFixedJoint(j_id);



    //now store the data we will need for the evaluation
    std::vector<Vector3d> m_f_foot;
    m_f_foot.resize(5);
    std::vector<ContactPoint>* cfs=m_stance_world->getContactForces();
    character->organize_foot_contact_forces(cfs,m_stance_foot,m_f_foot);

    m_contact_forces=m_f_foot;
}

void EstimationWorld::estimate_missing_bodies_impact()
{

    m_stance_original_bodies;



    //simulate the weight of the character
    COM_force=Vector3d(0,0,0);
    COM_pt=Point3d(0,0,0);
    COM_velocity=Vector3d(0,0,0);
    if (body_config!=0){

        if (!m_use_complex_fusing)
        {
            if (body_config!=6){
                Vector3d COM=character->getCOM(true);
                Vector3d COM_esti_world= Vector3d();

                double curMass = 0.0;
                double totalMass = curMass;
                for (uint i = 0; i <m_stance_original_bodies.size(); i++){
                    curMass = m_stance_original_bodies[i]->getMass();
                    totalMass += curMass;
                    COM_esti_world.addScaledVector(m_stance_original_bodies[i]->getCMPosition(), curMass);
                }

                Vector3d COM_missing_bodies=COM-COM_esti_world;
                double mass_missing_bodies=character->get_mass(true)-totalMass;
                COM_missing_bodies/=mass_missing_bodies;

                Point3d p_com(COM_missing_bodies.x,COM_missing_bodies.y,COM_missing_bodies.z);

                COM_force=Vector3d(0,SimGlobals::gravity,0)*mass_missing_bodies;
                COM_pt=p_com;
            }else{
                //for the config 6 the re are 2 forces
                //the top
                //so first I gather all the arbs I'll fuse
                {
                    std::vector<ArticulatedRigidBody*> vect_arbs;
                    for (int i=0;i<character->getJointCount();++i){
                        if (m_stance_world->getARBByName(character->getJoint(i)->child()->name())==NULL&&
                                character->getJoint(i)->child()->getCMPosition().y>character->getRoot()->getCMPosition().y){
                            vect_arbs.push_back(character->getJoint(i)->child());
                        }
                    }


                    Vector3d COM_missing_bodies= Vector3d();

                    double curMass = 0.0;
                    double totalMass = curMass;
                    for (uint i = 0; i <vect_arbs.size(); i++){
                        curMass = vect_arbs[i]->getMass();
                        totalMass += curMass;
                        COM_missing_bodies.addScaledVector(vect_arbs[i]->getCMPosition(), curMass);
                    }

                    double mass_missing_bodies=totalMass;
                    COM_missing_bodies/=mass_missing_bodies;

                    Point3d p_com(COM_missing_bodies.x,COM_missing_bodies.y,COM_missing_bodies.z);
                    COM_force=Vector3d(0,SimGlobals::gravity,0)*mass_missing_bodies;
                    COM_pt=p_com;
                }
                //the bottom
                {
                    std::vector<ArticulatedRigidBody*> vect_arbs;
                    for (int i=0;i<character->getJointCount();++i){
                        if (m_stance_world->getARBByName(character->getJoint(i)->child()->name())==NULL&&
                                character->getJoint(i)->child()->getCMPosition().y<character->getRoot()->getCMPosition().y){
                            vect_arbs.push_back(character->getJoint(i)->child());
                        }
                    }


                    Vector3d COM_missing_bodies= Vector3d();

                    double curMass = 0.0;
                    double totalMass = curMass;
                    for (uint i = 0; i <vect_arbs.size(); i++){
                        curMass = vect_arbs[i]->getMass();
                        totalMass += curMass;
                        COM_missing_bodies.addScaledVector(vect_arbs[i]->getCMPosition(), curMass);
                    }

                    double mass_missing_bodies=totalMass;
                    COM_missing_bodies/=mass_missing_bodies;

                    Point3d p_com(COM_missing_bodies.x,COM_missing_bodies.y,COM_missing_bodies.z);
                    COM_force_2=Vector3d(0,SimGlobals::gravity,0)*mass_missing_bodies;
                    COM_pt_2=p_com;
                }

            }

        }else
        {
            RigidBody* root;
            if (body_config==1||body_config==2){
                root= m_stance_world->getARBByName("torso");
            }
            if (body_config==3||body_config==4||body_config==5){
                root= m_stance_world->getARBByName("pelvis");
            }

            //so first I gather all the arbs I'll fuse
            std::vector<ArticulatedRigidBody*> vect_arbs;
            vect_arbs.push_back(character->getRoot());
            for (int i=0;i<character->getJointCount();++i){
                if (m_stance_world->getARBByName(character->getJoint(i)->child()->name())==NULL){
                    vect_arbs.push_back(character->getJoint(i)->child());
                }
            }


            Vector3d COM_missing_bodies= Vector3d();

            double curMass = 0.0;
            double totalMass = curMass;
            for (uint i = 0; i <vect_arbs.size(); i++){
                curMass = vect_arbs[i]->getMass();
                totalMass += curMass;
                COM_missing_bodies.addScaledVector(vect_arbs[i]->getCMPosition(), curMass);
            }

            double mass_missing_bodies=totalMass;
            COM_missing_bodies/=mass_missing_bodies;

            Point3d p_com(COM_missing_bodies.x,COM_missing_bodies.y,COM_missing_bodies.z);


            //so the main problem is to compute the moment of inertia of the combined body
            //to do so we use the parralel axis theorem
            Vector3d PMI_fused=Vector3d(0,0,0);
            for (uint i = 0; i <vect_arbs.size(); i++){
                ArticulatedRigidBody* arb=vect_arbs[i];
                Vector3d PMI_bdy = arb->getPMI();

                Vector3d d=p_com-arb->getCMPosition();

                Vector3d d_temp;
                d_temp=d;
                d.x=0;
                PMI_fused.x=PMI_bdy.x+arb->getMass()*d_temp.length()*d_temp.length();

                d_temp=d;
                d.y=0;
                PMI_fused.y=PMI_bdy.y+arb->getMass()*d_temp.length()*d_temp.length();

                d_temp=d;
                d.z=0;
                PMI_fused.z=PMI_bdy.z+arb->getMass()*d_temp.length()*d_temp.length();
            }

            //now we need to translate it back to the root geometric com
            {
                Vector3d d=root->getCMPosition()-p_com;

                Vector3d d_temp;
                d_temp=d;
                d.x=0;
                PMI_fused.x=PMI_fused.x+mass_missing_bodies*d_temp.length()*d_temp.length();

                d_temp=d;
                d.y=0;
                PMI_fused.y=PMI_fused.y+mass_missing_bodies*d_temp.length()*d_temp.length();

                d_temp=d;
                d.z=0;
                PMI_fused.z=PMI_fused.z+mass_missing_bodies*d_temp.length()*d_temp.length();
            }

            //and we set the object properties
            root->setMass(mass_missing_bodies);
            //root->setMOI(PMI_fused);
            m_stance_world->initODEMassProperties();

        }


        if (false){
            //since I'm doing it on 1 timestep I don't need that
            Vector3d characer_velocity=character->getCOMVelocity();
            double mass=character->get_mass(true);
            Vector3d COM_vel=characer_velocity;
            COM_vel*=mass;
            COM_vel.addScaledVector(m_stance_foot->getCMVelocity(), -m_stance_foot->getMass());
            mass-=m_stance_foot->getMass();
            COM_vel.addScaledVector(m_stance_lowerleg->getCMVelocity(), -m_stance_lowerleg->getMass());
            mass-=m_stance_lowerleg->getMass();
            COM_vel.addScaledVector(m_stance_toes->getCMVelocity(), -m_stance_toes->getMass());
            mass-=m_stance_toes->getMass();
            COM_vel.addScaledVector(m_stance_upperleg->getCMVelocity(), -m_stance_upperleg->getMass());
            mass-=m_stance_upperleg->getMass();
            COM_vel/=mass;

            COM_velocity=COM_vel;
            //        m_stance_com_body->setCMVelocity(COM_vel);
        }


    }

}


Quaternion EstimationWorld::get_stance_foot_orientation(){return m_stance_foot->getOrientation();}

void EstimationWorld::load_real_world_position_to_estimation_world()
{
    //copy the current state
    if (m_stance_original_bodies.size()!=m_stance_world->getARBCount()){
        std::cout<<"StanceFootContactController::run_simulation: WTF pas le mm nb d'elements ????";
    }
    for (int i=0;i< m_stance_original_bodies.size();++i){
        m_stance_world->getARBByPositionInData(i)->cpy_state(m_stance_original_bodies[i]);

        if (m_stance_original_bodies[i]->name()!=m_stance_world->getARBByPositionInData(i)->name()){
            std::cout<<"StanceFootContactController::run_simulation: WTF 2 rb associes alors que ce ne sont pas les memes...";
        }
    }
}

void EstimationWorld::load_ode_data()
{
    int init_result=dAllocateODEDataForThread(dAllocateMaskAll);
    if (init_result == false){
        std::cout << "ODE initialisation failed 2" << std::endl;
    }
}
