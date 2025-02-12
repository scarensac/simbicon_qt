/*
    Simbicon 1.5 Controller Editor Framework,
    Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
    All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

    This file is part of the Simbicon 1.5 Controller Editor Framework.

    Simbicon 1.5 Controller Editor Framework is free software: you can
    redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Simbicon 1.5 Controller Editor Framework is distributed in the hope
    that it will be useful, but WITHOUT ANY WARRANTY; without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Simbicon 1.5 Controller Editor Framework.
    If not, see <http://www.gnu.org/licenses/>.
*/

#include "SimBiController.h"
#include <Utils/Utils.h>
#include "SimGlobals.h"
#include "ConUtils.h"
#include "core/pose_control/twolinkik.h"
#include <MathLib/Point3d.h>
#include <fstream>
#include "Globals.h"
#include <sstream>

///Modules
#include "core/velocity_control/velocitycontroller.h"
#include "core/pose_control/swingfootcontroller.h"
#include "core/ground_contact_control/stancefootcontactcontroller.h"
#include "core/medium_control/externalforcefieldscompenser.h"
#include "Core/pose_control/posecontroller.h"

///TODO remove that include when the pose controller has been completely moved
#include "Core/pose_control/posecontrolglobals.h"


SimBiController::SimBiController(Character* b){
    if (b == NULL)
        throwError("Cannot create a SIMBICON controller if there is no associated biped!!");
    /*
    //this test if the quaternion <==> euler conversions actualy works
    {
        Quaternion q=Quaternion::getRotationQuaternion(PI/4,Vector3d(1,0,0));
        Quaternion q2(Vector3d(PI/4,0,0));
        q.getRotationAngles();
        q2.getRotationAngles();

        Vector3d test=q.rotate(Vector3d(0,0,1));
        std::cout<<"test: "<<test.toString()<<std::endl;

        Vector3d test2=q2.rotate(Vector3d(0,0,1));
        std::cout<<"tes2t: "<<test2.toString()<<std::endl;
    }
    {
        Quaternion q=Quaternion::getRotationQuaternion(3*PI/4,Vector3d(0,1,0));
        Quaternion q2(Vector3d(0,3*PI/4,0));
        q.getRotationAngles();
        q2.getRotationAngles();

        Vector3d test=q.rotate(Vector3d(0,0,1));
        std::cout<<"test: "<<test.toString()<<std::endl;

        Vector3d test2=q2.rotate(Vector3d(0,0,1));
        std::cout<<"tes2t: "<<test2.toString()<<std::endl;

        std::cout<<"test3: "<<q2.getRotationAngle(Vector3d(1,0,0))<<std::endl;//LOL
    }
    {
        Quaternion q=Quaternion::getRotationQuaternion(PI/4,Vector3d(0,0,1));
        Quaternion q2(Vector3d(0,0,PI/4));
        q.getRotationAngles();
        q2.getRotationAngles();

        Vector3d test=q.rotate(Vector3d(0,1,0));
        std::cout<<"test: "<<test.toString()<<std::endl;

        Vector3d test2=q2.rotate(Vector3d(0,1,0));
        std::cout<<"tes2t: "<<test2.toString()<<std::endl;
    }

    {
        Quaternion q(Vector3d(1.241593,1.441593,-0.141593));
        q.getRotationAngles();

        Quaternion q2(Vector3d(-1.9,1.7,3));
        q2.getRotationAngles();





        Vector3d test=q.rotate(Vector3d(0,1,0));
        std::cout<<"test: "<<test.toString()<<std::endl;

        Vector3d test2=q2.rotate(Vector3d(0,1,0));
        std::cout<<"tes2t: "<<test2.toString()<<std::endl;
    }
    //*/


    character = b;
    torques.resize(character->getJointCount());


    //characters controlled by a simbicon controller are assumed to have: 2 feet
    RigidBody* lFoot = b->getARBByName("lFoot");
    RigidBody* rFoot = b->getARBByName("rFoot");


    if (rFoot == NULL || lFoot == NULL)
        throwError("The biped must have the rigid bodies lFoot and rFoot!");

    //and two hips connected to the root
    Joint* lHip = b->getJointByName("lHip");
    Joint* rHip = b->getJointByName("rHip");

    if (rHip == NULL || lHip == NULL)
        throwError("The biped must have the joints lHip and rHip!");


    RigidBody* root = b->getRoot();

    if (lHip->parent() != rHip->parent() || lHip->parent() != root)
        throwError("The biped's hips must have a common parent, which should be the root of the figure!");

    phi = 0;



    bodyTouchedTheGround = false;


    //init the speed trajecotries
    velDSagittal = 0.7;//0.95 old value
    velDCoronal = 0;


    //modules
    pose_controller= new PoseController(character);
    velocity_controller= new VelocityController(character);
    external_force_fields_compenser= new ExternalForceFieldsCompenser(character);

    stance_foot_contact_controller=NULL;
    if (Globals::use_contact_controller){
        stance_foot_contact_controller= new StanceFootContactController(character);
        stance_foot_contact_controller->pose_controller=pose_controller;
        stance_foot_contact_controller->velocity_controller=velocity_controller;
        stance_foot_contact_controller->external_force_fields_compenser=external_force_fields_compenser;
    }
}

SimBiController::~SimBiController(void){
    delete velocity_controller;

    if (stance_foot_contact_controller!=NULL){
        delete stance_foot_contact_controller;
    }

    delete external_force_fields_compenser;
    delete pose_controller;
}

int SimBiController::get_start_stance()
{
    return pose_controller->starting_stance();
}

void SimBiController::preprocess_simulation_step(double dt, std::vector<ContactPoint> *cfs)
{
    resetTorques();

    //let the adapter learn for the new phi
    velD_adapter();

    pose_controller->preprocess_simulation_step(getPhase(),character->getCOMVelocity());

    if (stance_foot_contact_controller!=NULL){
        stance_foot_contact_controller->preprocess_simulation_step(getPhase());
    }

    external_force_fields_compenser->preprocess_simulation_step();

}

void SimBiController::simulation_step(double dt, WaterImpact &resulting_impact)
{
    //launch the controller to compute the torques we will need to apply
    computeTorques(resulting_impact);


    //apply the torques to the character (just  copy the actual simulation is done later)
    applyTorques();
}

void SimBiController::postprocess_simulation_step(bool new_step)
{
    if (stance_foot_contact_controller!=NULL){
        stance_foot_contact_controller->postprocess_simulation_step(new_step);
    }
}


/**
    This method is used to set the stance
*/
void SimBiController::setStance(int newStance){
    pose_controller->set_stance(newStance);
}

/**
    This method is used to populate the structure that is passed in with the current state
    of the controller;
*/
void SimBiController::getControllerState(SimBiControllerState* cs){
    cs->stance = getStance();
    cs->phi = this->phi;
    cs->FSMStateIndex = getFSMState();
    cs->bodyGroundContact = this->bodyTouchedTheGround;
}

/**
    This method is used to populate the state of the controller, using the information passed in the
    structure
*/
void SimBiController::setControllerState(const SimBiControllerState &cs){
    pose_controller->set_stance(cs.stance);
    setStance(cs.stance);
    phi = cs.phi;
    pose_controller->set_fsm_state(cs.FSMStateIndex);
    bodyTouchedTheGround = cs.bodyGroundContact;
}

/**
    This method should be called when the controller transitions to this state.
*/
void SimBiController::transitionToState(int stateIndex){

    //	tprintf("Transition to state: %d (stance = %s) (phi = %lf)\n", stateIndex, (stance == LEFT_STANCE)?("left"):("right"), phi);
    //reset the phase...
    //I'll backup the phi that was reached
    phi_last_step = phi;
    phi = 0;

}



/**
    This method is used to advance the controller in time. It takes in a list of the contact points, since they might be
    used to determine when to transition to a new state. This method returns -1 if the controller does not advance to a new state,
    or the index of the state that it transitions to otherwise.
*/
int SimBiController::advanceInTime(double dt, DynamicArray<ContactPoint> *cfs){
    //*
    if (!pose_controller->check_fsm_state()){
        return -1;
    }
    //*/

    bodyTouchedTheGround = false;
    //see if anything else other than the feet touch the ground...
    for (uint i=0;i<cfs->size();i++){
        //if neither of the bodies involved are articulated, it means they are just props so we can ignore them
        if ((*cfs)[i].rb1->isArticulated() == false && (*cfs)[i].rb2->isArticulated() == false)
            continue;

        if (character->is_foot((*cfs)[i].rb1) || character->is_foot((*cfs)[i].rb2))
            continue;

        bodyTouchedTheGround = true;
    }

    //advance the phase of the controller
    advance_phase(dt);



    int newStateIndex=pose_controller->end_simulation_step(phi);

    if (newStateIndex!=-1){
        transitionToState(newStateIndex);
    }
    //if we didn't transition to a new state...
    return newStateIndex;
}




/**
This function get the desired pose from the last step
*/
void SimBiController::getDesiredPose(DynamicArray<double>& trackingPose){
    pose_controller->target_pose(trackingPose);
}

void SimBiController::set_character_to_desired_pose(Character* c)
{
    pose_controller->set_character_to_desired_pose(c);
}

void SimBiController::change_swing_leg_target_rotation(double angle)
{
    pose_controller->set_swing_leg_rotation_angle(angle);
}

/**
    This method makes it possible to evaluate the debug pose at any phase angle
*/
void SimBiController::updateTrackingPose(DynamicArray<double>& trackingPose, double phiToUse){
    pose_controller->tracking_pose(trackingPose,phiToUse);
}



/**
    This method is used to compute the torques that are to be applied at the next step.
*/
void SimBiController::computeTorques(WaterImpact &resulting_impact){
    //*
    if (!pose_controller->check_fsm_state()){
        return;
    }
    //*/

    //first I check if we are near to fall (or even already on the ground
    double h = character->getRoot()->getCMPosition().y;

    double h_limit = 0.4;

    //if we are on the ground I can just skip everything
    if (h <= h_limit){
        for (int i = 0; i <(int) torques.size(); i++){
            torques[i] = Vector3d(0, 0, 0);
        }
        return;
    }

    //we compute the basic target
    pose_controller->simulation_step(phi);


    //compute the torque to reach the target pose
    const std::vector<Vector3d>& pose_control_tq=pose_controller->torques();
    for (int i = 0; i<(int)character->getJointCount(); i++){
        torques[i] = pose_control_tq[i];
    }


    //compute the orientation control
    pose_controller->compute_root_orientation_torques(torques[character->stance_hip()->idx()],
            torques[character->swing_hip()->idx()],torques[character->root_top()->idx()]);


    //virtual force velocity control
    {
        velocity_controller->apply_virtual_force(character->getCOMVelocity());

        std::vector<Vector3d>& tq=velocity_controller->torques();

        for (int i=0; i<tq.size();++i){
            torques[i]+=tq[i];
        }
    }


    //we'll also compute the torques that cancel out the effects of gravity, for better tracking purposes
    external_force_fields_compenser->simulation_step();
    if ((!Globals::parralelFluidAndControl)&&(!Globals::simulateFluid)){
    //    external_force_fields_compenser->compute_fluid_impact_compensation(resulting_impact,0);
    }
    //external_force_fields_compenser->compute_fluid_impact_compensation(resulting_impact,1);
    std::vector<Vector3d>& buff_vec2=external_force_fields_compenser->torques4_gravity();
    //std::vector<Vector3d>& buff_vec3=external_force_fields_compenser->torques4_fluid();



    //this system check if the computed torque opose the one computed by the rest of the controller
    //since this component goal is just to maque the control easier, I cancel the torque if it opose the
    //one computed by the rest of the controller
    double reduction_factor=0;
    for (int i = 0; i<(int)character->getJointCount(); i++){
        //buff_vec2[i]+=buff_vec3[i];

        RigidBody* body=character->getJoint(i)->child();

        /*
        if(body==character->stance_hip()->child()){
            std::cout<<body->name()<<"  : "<<buff_vec3[i].y<<std::endl;
        }
        //*/


        Vector3d calc=torques[i]*buff_vec2[i];
        if (calc.x<0){
            buff_vec2[i].x*=reduction_factor;
        }
        if (calc.y<0){
            buff_vec2[i].y*=reduction_factor;
        }
        if (calc.z<0){
            buff_vec2[i].z*=reduction_factor;
        }

        //and apply the torque
        torques[i] += buff_vec2[i];
    }



    //here I'l start the stance foot control system
    if (Globals::use_contact_controller){
        //*
        Timer t;
        t.restart();

        //*
        stance_foot_contact_controller->simulation_step(torques);
        std::vector<Vector3d> vect_torques_stance_foot_controler=stance_foot_contact_controller->result_torque();
        torques[character->stance_foot()->child_joints().front()->idx()]+=vect_torques_stance_foot_controler[0];
        torques[character->stance_foot()->parent_joint()->idx()]+=vect_torques_stance_foot_controler[1];
        torques[character->stance_foot()->parent_joint()->parent()->parent_joint()->idx()]+=vect_torques_stance_foot_controler[2];


        /*
        static double count_time=0;
        static double delta_time=0;
        delta_time+=t.timeEllapsed();
        if (phi<0.001){
            count_time+=delta_time;
            std::cout<<"time used for forward estimation: "<<count_time<<"  "<<delta_time<<std::endl;
            delta_time=0;
        }
        //*/
    }
    //*




    for (uint i=0;i<torques.size();i++){
        //check for null
        Vector3d t=torques[i];
        if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
            throw("simbicon::compute_torques  undefinned torques   ");
        }
    }

}




/**
    This method loads all the pertinent information regarding the simbicon controller from a file.
*/
void SimBiController::loadFromFile(char* fName){
    pose_controller->load_from_file(fName);

    //now init the componennts
    velocity_controller->init_velD_trajectory(pose_controller->velD_traj);
    velocity_controller->init_new_character_step();

    if (stance_foot_contact_controller!=NULL){
        stance_foot_contact_controller->init_new_character_step();
    }
}

/**
this function will store the velocities every 0.1 phi when on learning mode
when learning learning mode is desactivated the function will adapt the velD_trajectories so they have a better fit on the movement
this system will also update a bolean signaling if the next step will be a recovery step or if it will be a normal step
*/
void SimBiController::velD_adapter(bool learning_mode, bool* trajectory_modified){
    if (learning_mode){
        velocity_controller->store_velocity(getPhase(),character->getCOMVelocity());
    }
}


void SimBiController::advance_phase(double dt){
    this->phi += dt / pose_controller->state_time();
}


bool SimBiController::need_recovery_step(){
    return velocity_controller->need_recovery_steps();
}

Vector3d SimBiController::avg_velocity(int step_offset)
{
    Vector3d avg_vel=Vector3d(0,0,0);
    try{
        avg_vel.z=velocity_controller->prev_step_sag_speed(step_offset);
        avg_vel.x=velocity_controller->prev_step_cor_speed(step_offset);
    }catch (...){}

    return avg_vel;
}

SimBiConState *SimBiController::cur_state()
{
    return pose_controller->get_fsm_state();
}

double SimBiController::ipm_alt_sagittal(){
    return pose_controller->ipm_alt_sagittal();
}

std::vector<ControlParams>& SimBiController::get_control_params()
{
    return pose_controller->get_control_params();
}

ControlParams& SimBiController::get_root_control_params()
{
    return pose_controller->get_root_control_params();
}



Quaternion SimBiController::getCharacterFrame(){
    return character->getHeadingHorizontal();
}


int SimBiController::getFSMState(){
    return pose_controller->state_idx();
}


void SimBiController::add_friction_on_uncontrolled(){
    for (int i=0;i<character->getJointCount();i++){
        if (!pose_controller->is_controlled(i)){
            //torques[i]=character->getJoint(i)->child()->getAngularVelocity()*(-0.05);
            /*
            Vector3d wRel=Vector3d(0,0,0);
            character->getRelativeAngularVelocity(i, &wRel);
            torques[i]=wRel*(-0.05);
            //*/
        }
    }
}

void SimBiController::restart()
{
    phi=0;

    //restart all the modules
    velocity_controller->restart();
    //stance_foot_contact_controller->restart();
    //external_force_fields_compenser->restart();
    pose_controller->restart();
}

void SimBiController::writeToFile(char *fileName, char *stateFileName)
{
    FILE* f;
    if (fileName == NULL || (f = fopen(fileName, "w")) == NULL)
        return;

    pose_controller->writeToFile(f,stateFileName);


    fclose(f);
}

void SimBiController::writeToFile(std::string fileName, std::string *stateFileName)
{
    char fname[256], statename[256];
    strcpy(fname, fileName.c_str());

    if (stateFileName != NULL){
        strcpy(statename, stateFileName->c_str());
        writeToFile(fname, statename);
    }
    else{
        writeToFile(fname);
    }
}


SimBiConState *SimBiController::getState() {
    return pose_controller->get_fsm_state();
}
