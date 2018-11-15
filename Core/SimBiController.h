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

#pragma once

#include "BaseControlFramework.h"
#include <Utils/Utils.h>
#include <Physics/rb/RigidBody.h>
#include "SimBiConState.h"
#include "Trajectory.h"
#include "VirtualModelController.h"
#include <iostream>
#include <sstream>

///Modules
class VelocityController;
class StanceFootContactController;
class ExternalForceFieldsCompenser;
class PoseController;

/**
    This structure is used to store the state of a simbicon controller. It can be used to save/load a controller's
    states, where the state here does not refer to the states in the Finite State Machine. The state refers to the
    phase in the current FSM state, and also the stance.
*/
typedef struct {
    int stance;
    double phi;
    int FSMStateIndex;
    bool bodyGroundContact;
} SimBiControllerState;


/**
 * A simbicon controller is a fancy PoseController. The root (i.e. pelvis or torso), as well as the two hips are controlled
 * relative to a semi-global coordinate frame (it needs only have the y-axis pointing up), but all other joints are controlled
 * exactely like in a normal PoseController. The SimBiController operates on a biped, which means that we can make special
 * assumptions about it: it must have two joints, lHip and rHip, connecting the root to the left and right upper-legs,
 * and it must also have two feet (lFoot and rFoot) as rigid bodies in the articulated linkage.
 */

class SimBiController{

public:
    ///TODO encapsulate that better normaly i should not have to set them to public
    ///Modules
    VelocityController* velocity_controller;
    StanceFootContactController* stance_foot_contact_controller;
    ExternalForceFieldsCompenser* external_force_fields_compenser;
    PoseController* pose_controller;


private:

    /**
    These are quantities that are set only once
    */
    //this is the character that the controller is acting on
    Character* character;
    //and this is the array of torques that will be computed in order for the character to match the desired pose - stored in world coordinates
    std::vector<Vector3d> torques;


    /**
    these are quantities that get updated throughout the simulation
    */

    //the phase parameter, phi must have values between 0 and 1, and it indicates the progress through the current state.
    double phi;

    //this variable, updated everytime the controller state is advanced in time, is set to true if any body other than the feet are in contact
    //with the ground, false otherwise. A higer level process can determine if the controller failed or not, based on this information.
    bool bodyTouchedTheGround;



public:



    //desired velocity in the sagittal plane
    double velDSagittal;
    //desired velocity in the coronal plane...
    double velDCoronal;


    //this variable contains the phi reached at the end of the last step
    double phi_last_step;

protected:

    /**
        This method should be called when the controller transitions to this state.
    */
    void transitionToState(int stateIndex);


    /**
        This method is used to set the stance
    */
    void setStance(int newStance);


public:
    /**
        Default constructor
    */
    SimBiController(Character* b);

    /**
        Destructor
    */
    virtual ~SimBiController(void);

    /**
     * @brief getters
     */
    int getStance(){return character->stance();}
    int get_start_stance();
    Character* get_character(){ return character;}
    bool need_recovery_step();
    Vector3d getTorque(int i){
        if (i < 0 || i >(int)torques.size() - 1)
            return Vector3d(0,0,0);
        return torques[i];
    }
    Vector3d avg_velocity(int step_offset=0);
    SimBiConState* cur_state();
    double ipm_alt_sagittal();
    double get_phi_last_step(){return phi_last_step;}
    std::vector<ControlParams>& get_control_params();
    ControlParams& get_root_control_params();



    SimBiConState* getState();

    /**
     * @brief this function handles all the preprocessing necessary at the start of a new simulation step
     */
    void preprocess_simulation_step(double dt, std::vector<ContactPoint> *cfs);

    /**
     * @brief this function handles all the processing necessary during simulation step
     */
    void simulation_step(double dt, std::map<uint, WaterImpact> &resulting_impact);

    /**
     * @brief this function handles all the postprocessing necessary at the end of a simulation step
     */
    void postprocess_simulation_step(bool new_step);


    /**
     * @brief This function update the velD.
     */
    inline void calc_desired_velocities(){
        //read the parameters from the gui
        velDSagittal = SimGlobals::velDSagittal;
        velDCoronal = SimGlobals::velDCoronal;
    }

    /**
        This method is used to compute the torques that are to be applied at the next step.
    */
    void computeTorques(std::map<uint, WaterImpact>& resulting_impact);

    /**
        This method is used to advance the controller in time. It takes in a list of the contact points, since they might be
        used to determine when to transition to a new state. This method returns -1 if the controller does not advance to a new state,
        or the index of the state that it transitions to otherwise.
    */
    int advanceInTime(double dt, DynamicArray<ContactPoint> *cfs);

    /**
     * @brief this function is here to make phi advance following the specifyed dt
     */
    void advance_phase(double dt);

    /**
        This method is used to populate the structure that is passed in with the current state
        of the controller;
    */
    void getControllerState(SimBiControllerState* cs);

    /**
        This method is used to populate the state of the controller, using the information passed in the
        structure
    */
    void setControllerState(const SimBiControllerState &cs);

    /**
        This method loads all the pertinent information regarding the simbicon controller from a file.
    */
    void loadFromFile(char* fName);

    /**
        This method is used to return the value of bodyGroundContact
    */
    inline bool isBodyInContactWithTheGround(){
        return bodyTouchedTheGround;
    }

    /**
        This method is used to return the value of the phase (phi) in the current FSM state.
    */
    inline double getPhase(){
        return phi;// *(SimGlobals::force_alpha / 100000);
    }

    /**
        This method returns the position of the CM of the stance foot, in world coordinates
    */
    inline Point3d getStanceFootPos(){
        if (character->stance_foot())
            return character->stance_foot()->getCMPosition();
        return Point3d(0,0,0);
    }

    /**
        This method returns the position of the CM of the swing foot, in world coordinates
    */
    inline Point3d getSwingFootPos(){
        if (character->swing_foot())
            return character->swing_foot()->getCMPosition();
        return Point3d(0,0,0);
    }


    /**
        This method is used to return the current state number
    */
    int getFSMState();

    /**
        This method returns the character frame orientation
    */
    Quaternion getCharacterFrame();




    /**
    This function get the desired pose from the last step
    */
    void getDesiredPose(DynamicArray<double>& trackingPose);


    /**
     * @brief set_character_to_desired_pose
     */
    void set_character_to_desired_pose(Character *c);
    void change_swing_leg_target_rotation (double angle);

    /**
        This method makes it possible to evaluate the debug pose at any phase angle
        Negative phase angle = Use the current controller phase angle
    */
    void updateTrackingPose(DynamicArray<double>& trackingPose, double phiToUse = -1);

    /**
    this function will store the velocities every for every phi specified on the velD curve when on learning mode
    when learning learning mode is desactivated the function will adapt the velD_trajectories so they have a better fit on the movement
    this system will also update a bolean signaling if the next step will be a recovery step or if it will be a normal step
    */
    void velD_adapter(bool learning_mode=true, bool* trajectory_modified = NULL);


    /**
                This method is used to apply the torques that are computed to the character that is controlled.
        */
    void applyTorques(){

        for (int i=0;i<character->getJointCount();i++){
            character->getJoint(i)->torque(torques[i]);
        }
    }

    /**
                This method is used to reset the torques that are to be applied.
        */
    void resetTorques(){
        for (int i=0;i<character->getJointCount();i++){
            torques[i] = Vector3d(0,0,0);
        }
    }

    void add_friction_on_uncontrolled();

    void restart();

    /**
    This method is used to write the current controller to a file
    */
    void writeToFile(char* fileName, char* stateFileName = NULL);

    /**
    This method is used to write the current controller to a file
    */
    void writeToFile(std::string fileName, std::string* stateFileName = NULL);


};
