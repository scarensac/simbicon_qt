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
#include "basecontrolframework.h"
#include "Character.h"
#include <Core/SimBiController.h>

/**
    This structure is used to hold the state of the simbicon framework. This includes the world configuration (i.e. state of the rigid bodies), the
    state of the Simbicon controller that is used and also the contact forces that are acting on the character. The forces are necessary because
    the control will be using them - the simulation will actually be ok without them, since they are recomputed before integration anyway.
*/
typedef struct {
    //hold the world state here
    std::vector<double> worldState;
    //hold the state of the controller here:
    SimBiControllerState conState;
    //position of the last stance foot - used to compute step lengths
    Point3d lastFootPos;
    //and this is used to hold the contact force information
    std::vector<ContactPoint> cfi;



} SimBiConFrameworkState;

/**
    This class is used for Simbicon control.
*/

class SimBiConFramework : public BaseControlFramework{
    friend class ControllerEditor;
private:
    //this is the controller that we will use.
    SimBiController* con;

    //we'll keep track of the vector that represents each step taken. The vector will be represented in the 'relative' world coordinate frame
    Vector3d lastStepTaken;
    //this is the position of the foot at the previous state
    Point3d lastFootPos;


    //set to true if we detect a new character step
    bool _new_character_step;


public:
    //tis variable contain the impact of the water for the previous step
    WaterImpact resulting_impact;

public:
    SimBiConFramework(const char *input, char* conFile = NULL);
    virtual ~SimBiConFramework(void);



    /**
         * @brief this function handles all the preprocessing necessary at the start of a new simulation step
         */
    void preprocess_simulation_step(double dt);

    /**
         * @brief this function handles all the processing necessary during simulation step
         */
    void simulation_step(double dt);

    /**
         * @brief this function handles all the postprocessing necessary at the end of a simulation step
         */
    void postprocess_simulation_step(double dt);


    /**
         * @brief getters
         */
    bool new_character_step(){return _new_character_step;}


    void restart(SimBiConFrameworkState& conFState);
    void stand(SimBiConFrameworkState& conFState);

    /**
            this method is used to return the quaternion that represents the to
            'rel world frame' transformation. This is the coordinate frame that the desired pose
            is computed relative to.
    */
    inline Quaternion getCharacterFrame(){
        return con->getCharacterFrame();
    }

    /**
        This method returns the controller that is being used.
    */
    SimBiController* getController(){
        return con;
    }

    /**
        populates the structure that is passed in with the state of the framework
    */
    void getState(SimBiConFrameworkState* conFState);

    /**
        populates the state of the framework with information passed in with the state of the framework
    */
    void setState(SimBiConFrameworkState& conFState);

    /**
        this method returns the vector that corresponds to the last step taken
    */
    inline Vector3d getLastStepTaken(){
        return lastStepTaken;
    }

    /*
    this function is a quick and easy way to save the current controler and the current position
    the to boolean are here to help choose which one is saved

    */
    /**
     * @brief save
     * @param save_controller
     * @param save_position
     * @return return false if we need to try to save once again the next step
     */
    bool save(bool save_controller = true, bool save_position = true);
};
