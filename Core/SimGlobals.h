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

#include <MathLib/Vector3d.h>
#include <Physics/ODEWorld.h>


#define LEFT_STANCE 0
#define RIGHT_STANCE 1

/**
        This class is used as a container for all the constants that are pertinent for the physical simulations, the controllers, etc.
*/

class SimGlobals {
public:
    //We will assume that the gravity is in the y-direction (this can easily be changed if need be), and this value gives its magnitude.
    static double gravity;
    //this is the direction of the up-vector
    static Vector3d up;
    //if this is set to true, then the heading of the character is controlled, otherwise it is free to do whatever it wants
    static int forceHeadingControl;
    //this variable is used to specify the desired heading of the character
    static double desiredHeading;
    static double desiredHeading_active;//this is only to be able to show the arrow
    //and this is the desired time interval for each simulation timestep (does not apply to animations that are played back).
    static double dt;

    static AbstractRBEngine* activeRbEngine;
    static AbstractRBEngine* stanceFootWorld;

    //temp..
    static double targetPos;
    static double targetPosX;
    static double targetPosZ;

    static double conInterpolationValue;
    static double bipDesiredVelocity;

    static int constraintSoftness;

    static int CGIterCount;
    static int linearizationCount;

    static double rootSagittal;
    static double rootLateral;
    static double swingHipSagittal;
    static double swingHipLateral;
    static double stanceAngleSagittal;
    static double stanceAngleLateral;
    static double stanceKnee;


    static double COMOffsetX;
    static double COMOffsetZ;

    static double time_factor;

    static bool force_ipm;

    //memebers for the water force
    static double force_alpha;
    static double water_level;
    static double liquid_density;
    static double liquid_viscosity;
    static double left_stance_factor;//0 or 1

    //I'll use the contact point structure but all I want is the position and the force
    static std::vector<ForceStruct> vect_forces;

    //for the deplacement direction control
    static double velDSagittal;
    static double velDCoronal;
    static double velDSagittalOld;
    static double velDCoronalOld;

    static double step_width;

    //those variables are here for the evaluation mode (done in the case of an evolution strategy)
    static bool is_evaluation;
    static int steps_before_evaluation;
    //the length of the evaluation in seconds
    static int evaluation_length;

    //those variablex are here to give us a way to prioritise one strategy of speedcontrol over the others
    static double ipm_alteration_effectiveness;
    static double virtual_force_effectiveness;


    //this variable is for communication beteen the ui and the system
    //it is use the modify the current state time
    //the boolean is used to know if te value has been modified by the ui
    static double requested_state_duration;
    static bool state_duration_modified;


    //this variable is for communication beteen the ui and the system
    //it is use the modify the current state time
    //the boolean is used to know if te value has been modified by the ui
    static int nb_container_boxes;
    static int nb_filler;
    static int nb_collisons_event;
    static int nb_active_objects;

    SimGlobals(void){
    }
    ~SimGlobals(void){
    }

    inline static AbstractRBEngine* getRBEngine(){
        if (activeRbEngine == NULL)
            activeRbEngine = new ODEWorld();
        return activeRbEngine;
        //		return new PhysEng();
    }

    inline static void closeRBEngine(){
        if (activeRbEngine != NULL){
            delete activeRbEngine;
            activeRbEngine = NULL;
        }
    }

    static void reset_sim_globals();


    ///From now on the memebres are not real globals they are just values to do test
    //this one goes to true after the first time the foot is completely flat on the ground
    static bool foot_flat_on_ground;
    //this one tells us if the foot is currently flat on the ground
    static bool foot_flat_on_ground_current;
};
