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

#include "SimBiConFramework.h"
#include <Utils/Utils.h>
#include <Core/SimBiController.h>
#include "SimGlobals.h"
#include <Physics/ODEWorld.h>
#include "Core\ForcesUtilitary.h"
#include <fstream>
#include <sstream>
#include "Globals.h"
#include "SPlisHSPlasH/Interface.h"

///TODO remoe those incudes
#include "core/velocity_control/velocitycontroller.h"
#include "core/pose_control/swingfootcontroller.h"
#include "Core/pose_control/posecontrolglobals.h"
#include "Core/pose_control/posecontroller.h"

SimBiConFramework::SimBiConFramework(char* input, char* conFile){

    _new_character_step=false;


    //create the physical world...
    pw = SimGlobals::getRBEngine();
    con = NULL;
    bip = NULL;
    bool conLoaded = false;


    //now we'll interpret the input file...
    if (input == NULL)
        throwError("NULL file name provided.");
    FILE *f = fopen(input, "r");
    if (f == NULL)
        throwError("SimBiConFramework:Could not open file: %s", input);


    //have a temporary buffer used to read the file line by line...
    char buffer[200];
    //this is where it happens.
    while (!feof(f)){
        //get a line from the file...
        fgets(buffer, 200, f);
        if (feof(f))
            break;
        if (strlen(buffer)>195)
            throwError("The input file contains a line that is longer than ~200 characters - not allowed");
        char *line = lTrim(buffer);
        int lineType = getConLineType(line);

        std::string path;
        char effective_path[256];
        switch (lineType) {
            case LOAD_RB_FILE:
                //first i need to add the part of the path to go to the configutation_data folder
                path=std::string((trim(line)));
                path = interpret_path(path);

                //and now we cna use it
                strcpy(effective_path, path.c_str());
                pw->loadRBsFromFile(effective_path);
                if (bip == NULL && pw->getAFCount()>0) {
                    bip = new Character(pw->getAF(0));
                    con = new SimBiController(bip);
                }
                break;
            case LOAD_CON_FILE:
                if( conFile != NULL ) break; // Controller file
                if (con == NULL){
                    throwError("The physical world must contain at least one articulated figure that can be controlled!");
                }

                //if I am in save mode I'll simply
                if (Globals::save_mode&&(!Globals::save_mode_controller.empty())){
                    std::ostringstream oss;
                    oss << Globals::data_folder_path;
                    oss << "controllers/";
                    oss << Globals::save_mode_controller;
                    path=oss.str();
                }
                else{
                    //first i need to add the part of the path to go to the configutation_data folder
                    path = std::string((trim(line)));
                    path = interpret_path(path);
                }

                //and now we cna use it
                strcpy(effective_path, path.c_str());
                con->loadFromFile(effective_path);
                conLoaded = true;
                break;
            case CON_NOT_IMPORTANT:
                std::cerr<<"Ignoring input line: "<<line<<std::endl;
                break;
            case CON_COMMENT:
                break;
            default:
                throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
        }
    }
    fclose(f);

    if( conFile != NULL ) {
        if( con == NULL )
            throwError("The physical world must contain at least one articulated figure that can be controlled!");
        con->loadFromFile(conFile);
        conLoaded = true;
    }

    if (!conLoaded)
        throwError("A controller must be loaded!");

    //in case the state has changed while the controller was loaded, we will update the world again...
    //	pw->updateTransformations();

    //initialize the last foot position to the position of the stance foot - this may not always be the best decision, but whatever...
    lastFootPos = con->getStanceFootPos();
    if (!Globals::evolution_mode){
        static_cast<ODEWorld*>(pw)->create_cubes();

        if (Globals::simulateFluid){
            static_cast<ODEWorld*>(pw)->initParticleFluid();
        }
    }
}


SimBiConFramework::~SimBiConFramework(void){
    resulting_impact.clear();
    delete con;
}

void SimBiConFramework::preprocess_simulation_step(double dt)
{
    //here I'll organize the contact forces for easier use later
    con->get_character()->organize_foot_contact_forces(pw->getContactForces());

    //clear the vector that's used to show the forces
    SimGlobals::vect_forces.clear();

    _new_character_step=false;

    ODEWorld* world = dynamic_cast<ODEWorld*>(pw);
    const std::vector<RigidBody*>& vect_rb=world->get_rb_list();
    for (int i=0;i<vect_rb.size();++i){
        vect_rb[i]->update_angular_velocity_avg();
    }

    if (false){
        //test with the foot horientation and the foot com position
        static bool trigger=false;
        if (con->getPhase()<0.000001){
            trigger=false;
        }

        static double stance_foot_target_orientation;
        static Point3d stance_foot_target_position;
        RigidBody* rb=con->get_character()->stance_foot();
        const Vector3d* force_stance_foot=con->get_character()->force_stance_foot();

        if (!trigger){
            if (std::abs(force_stance_foot[2].y)>0.00001||std::abs(force_stance_foot[3].y)>0.00001){
                trigger=true;
                stance_foot_target_orientation=rb->getOrientation().getRotationAngle(SimGlobals::up);
                stance_foot_target_position=rb->getCMPosition();
            }
        }else{
            double delta_to_target_orientation=rb->getOrientation().getRotationAngle(SimGlobals::up)-stance_foot_target_orientation;
            Vector3d sup_torque_stance_foot(0,-delta_to_target_orientation*500,0);
            pw->applyTorqueTo(rb,sup_torque_stance_foot);

            std::ostringstream oss;
            oss<<stance_foot_target_orientation<<"    "<<delta_to_target_orientation;
            std::cout<<oss.str();

            Vector3d delta_to_target_position=rb->getCMPosition()-stance_foot_target_position;
            Vector3d force_stance_foot_com(-delta_to_target_position.x*50000,0,-delta_to_target_position.z*50000);
            pw->applyForceTo(rb,force_stance_foot_com,Vector3d(0,0,0));

            oss.str("");
            oss<<stance_foot_target_position.x<<" "<<stance_foot_target_position.y<<" "<<stance_foot_target_position.z<<"    "
              <<delta_to_target_position.x<<" "<<delta_to_target_position.y<<" "<<delta_to_target_position.z;
            std::cout<<oss.str();


            //        pw->applyForceTo(rb,force_stance_foot_com,Point3d(0,0,0));
        }
    }

    if (true){
        static bool trigger[4]={false,false,false,false};
        static bool trigger_central=false;
        static bool back_trigerred=false;
        static Vector3d delta_integral_vect[4]={Vector3d(0,0,0),Vector3d(0,0,0),Vector3d(0,0,0),Vector3d(0,0,0)};
        static Vector3d delta_integral_central_point=Vector3d(0,0,0);
        static Point3d stance_foot_target_position[4];
        static Point3d central_point_target;
        if (con->getPhase()<0.000001){
            trigger_central=false;
            back_trigerred=false;
            central_point_target=Point3d(0,0,0);
            for (int i=0;i<4;++i){
                trigger[i]=false;
                delta_integral_vect[i]=Vector3d(0,0,0);
                stance_foot_target_position[i]=Vector3d(0,0,0);
                delta_integral_central_point=Vector3d(0,0,0);
            }
            SimGlobals::foot_flat_on_ground=false;
        }

        RigidBody* rb=con->get_character()->stance_foot();
        const Vector3d* force_stance_foot=con->get_character()->force_stance_foot();
        //test with the position of each corner

        /*
        std::vector<ContactPoint> *cfs=pw->getContactForces();
        int count_real_contacts_front=0;
        int count_real_contacts_back=0;
        for (uint i = 0; i<cfs->size(); ++i){
            if (((*cfs)[i].rb1 == con->get_character()->stance_foot()) || ((*cfs)[i].rb2 == con->get_character()->stance_foot())){
                if (con->get_character()->stance_foot()->getLocalCoordinates((*cfs)[i].cp).z>0){
                    count_real_contacts_front++;
                }else{
                    count_real_contacts_back++;
                }
            }
        }
        if (count_real_contacts_back==0||count_real_contacts_front==0){
            SimGlobals::foot_flat_on_ground_current=false;
        }else{
            SimGlobals::foot_flat_on_ground_current=true;
        }
        //*/
        //*
        BoxCDP* box=dynamic_cast<BoxCDP*>(rb->get_cdps()[0]);
        bool all_trigger=true;
        int count_fail=0;
        for (int i=0;i<4;++i){
            if ((std::abs(force_stance_foot[i].y)<10)){
                count_fail++;
                //                trigger[i]=true;
                //                stance_foot_target_position[i]=rb->getWorldCoordinates(local_corner);
                //                stance_foot_target_position[i].y=0;
            }
        }
        if (count_fail>1&&!SimGlobals::foot_flat_on_ground){
            all_trigger=false;
//            std::cout<<count_fail<<std::endl;
        }
        /*
        for (int i=0;i<4;++i){
            Point3d local_corner=box->get_corner_position(i);
            Point3d global_corner=rb->getWorldCoordinates(local_corner);
            if (!trigger[i]){
                all_trigger=false;
                if ((std::abs(force_stance_foot[i].y)>10)){
                    trigger[i]=true;
                    stance_foot_target_position[i]=rb->getWorldCoordinates(local_corner);
                    stance_foot_target_position[i].y=0;
                }
            }else{

                float kp=00000;
                float ki=0;
                float kd=0;
                Vector3d position_err=stance_foot_target_position[i]-rb->getWorldCoordinates(local_corner);
                if (position_err.y>0){
                    position_err.y=0;
                }

                Vector3d velocity_err=-rb->getAbsoluteVelocityForLocalPoint(local_corner);

                Vector3d& delta_integral=delta_integral_vect[i];
                delta_integral+=position_err;


                Vector3d force;
                force.x=position_err.x*kp+delta_integral.x*ki+velocity_err.x*kd;
                force.y=position_err.y*kp+delta_integral.y*ki+velocity_err.y*kd;
                force.z=position_err.z*kp+delta_integral.z*ki+velocity_err.z*kd;

                pw->applyForceTo(rb,force,local_corner);


//                std::ostringstream oss;
//                oss.str("");
//                oss<<i<<"  "<<stance_foot_target_position[i].x<<" "<<stance_foot_target_position[i].y<<" "<<stance_foot_target_position[i].z<<"    "
//                  <<delta_to_target_position.x<<" "<<delta_to_target_position.y<<" "<<delta_to_target_position.z;
//                std::cout<<oss.str();
        }
        }
//*/

        /*
        //this is a system that ralign the target positions ot have a correct rectangle
        if (!back_trigerred&&trigger[0]&&trigger[1]){
            back_trigerred=true;
            Vector3d vect=stance_foot_target_position[1]-stance_foot_target_position[0];
            vect.toUnit();
            double target_length=Vector3d(box->get_corner_position(0),box->get_corner_position(1)).length();
            vect*=target_length;
            Point3d target=stance_foot_target_position[0]+vect;


            stance_foot_target_position[1]=target;


            central_point_target=(stance_foot_target_position[1]+stance_foot_target_position[0])/2;
            trigger_central=true;

            //for now the system only support the back of the foot first
            if (SimGlobals::foot_flat_on_ground){
                exit(534);
            }
        }
        /*
        if ((!SimGlobals::foot_flat_on_ground) && (all_trigger)){
            if (!Globals::evolution_mode&&!Globals::save_mode){
                std::cout<<"flat_on_ground_reached";
            }
            Vector3d vect(0,0,0);
            Vector3d ortho_to=stance_foot_target_position[1]-stance_foot_target_position[0];
            //we set z to one (but we will normalize the vector at some time
            vect.z=1;
            // x=-(z*z_2)/x_2
            vect.x=-(vect.z*ortho_to.z)/ortho_to.x;
            vect.toUnit();
            double target_length=Vector3d(box->get_corner_position(0),box->get_corner_position(2)).length();
            vect*=target_length;

            Point3d target2=stance_foot_target_position[0]+vect;
            Point3d target3=stance_foot_target_position[1]+vect;

            stance_foot_target_position[2]=target2;
            stance_foot_target_position[3]=target3;


            central_point_target=(stance_foot_target_position[0]+stance_foot_target_position[1]+
                    stance_foot_target_position[2]+stance_foot_target_position[3])/4;
            trigger_central=true;
        }

        //*/
        //use a pd control on a central point
        //as long as only the back is on the ground the central point is the center of the back
        //when all the foot touch the center of the foo become the central point
/*
        if (trigger_central){
            float kp=0;
            float ki=0;
            float kd=0;
            Point3d current_location;
            if (all_trigger){
                current_location=(box->get_corner_position(0)+box->get_corner_position(1)+
                                  box->get_corner_position(2)+box->get_corner_position(3))/4;
            }else{
                current_location=(box->get_corner_position(0)+box->get_corner_position(1))/2;
            }



            Vector3d position_err=central_point_target-rb->getWorldCoordinates(current_location);
            if (position_err.y>0){
                position_err.y=0;
            }

            Vector3d velocity_err=-rb->getAbsoluteVelocityForLocalPoint(current_location);

            Vector3d& delta_integral=delta_integral_central_point;
            delta_integral+=position_err;


            Vector3d force;
            force.x=position_err.x*kp+delta_integral.x*ki+velocity_err.x*kd;
            force.y=position_err.y*kp+delta_integral.y*ki+velocity_err.y*kd;
            force.z=position_err.z*kp+delta_integral.z*ki+velocity_err.z*kd;

            pw->applyForceTo(rb,force,current_location);
        }
//*/

        SimGlobals::foot_flat_on_ground=all_trigger;
    }


    con->preprocess_simulation_step(dt,pw->getContactForces());
}

#include <iostream>
void SimBiConFramework::simulation_step(double dt)
{
    ODEWorld* world = dynamic_cast<ODEWorld*>(pw);


    //we simulate the effect of the liquid
    resulting_impact.clear();
    world->compute_water_impact(con->get_character(),SimGlobals::water_level, resulting_impact);


    //compute and apply the necessayr torques
    con->simulation_step(dt,resulting_impact);


    if (Globals::simulateFluid){
        world->sendDataToParticleFluidEngine();
        world->advanceInTimeParticleFluidEngine(dt);
        world->readDataFromParticleFluidEngine();
    }

    //launch the physic simulation
    world->sendDataToEngine();
    world->advanceInTime(dt);
    world->readDataFromEngine();
}

void SimBiConFramework::postprocess_simulation_step(double dt)
{
    int new_state_idx = con->advanceInTime(dt, pw->getContactForces());
    bool newFSMState = (new_state_idx != -1);

    //here we are assuming that each FSM state represents a new step.
    if (newFSMState){

        //adapt the velD trajectories
        con->velocity_controller->adapt_learning_curve(con->get_character()->getCOMVelocity(),con->phi_last_step);
        Globals::avg_speed = con->avg_velocity();


        //read the speed from the gui
        con->calc_desired_velocities();

        lastStepTaken = Vector3d(lastFootPos, con->getStanceFootPos());
        //now express this in the 'relative' character coordinate frame instead of the world frame
        lastStepTaken = con->getCharacterFrame().getComplexConjugate().rotate(lastStepTaken);
        lastFootPos = con->getStanceFootPos();



        //reinitialise the modules
        con->velocity_controller->init_new_character_step();
        con->pose_controller->init_character_step(con->velocity_controller);


        /*
        ODEWorld* world = dynamic_cast<ODEWorld*>(pw);
        const std::vector<RigidBody*>& vect_rb=world->get_rb_list();
        for (int i=0;i<vect_rb.size();++i){
            vect_rb[i]->vect_angular_velocity_avg.clear();
        }
        //*/

        ODEWorld* world = dynamic_cast<ODEWorld*>(pw);
        const std::vector<RigidBody*>& vect_rb=world->get_rb_list();
        for (int i=0;i<vect_rb.size();++i){
            vect_rb[i]->reset_angular_velocity_avg();
        }

        SimGlobals::velDSagittalOld = SimGlobals::velDSagittal;
        SimGlobals::velDCoronalOld = SimGlobals::velDCoronal;

    }
    _new_character_step=newFSMState;

    con->postprocess_simulation_step(newFSMState);
}

void SimBiConFramework::restart(SimBiConFrameworkState &conFState)
{
    SimGlobals::reset_sim_globals();
    setState(conFState);
    con->restart();
}

void SimBiConFramework::stand(SimBiConFrameworkState &conFState)
{
    setState(conFState);
}



/**
        populates the structure that is passed in with the state of the framework
*/
void SimBiConFramework::getState(SimBiConFrameworkState* conFState){
    conFState->worldState.clear();
    //read in the state of the world (we'll assume that the rigid bodies and the ode world are synchronized), and the controller
    pw->getState(&(conFState->worldState));
    con->getControllerState(&(conFState->conState));
    conFState->lastFootPos = lastFootPos;
    //now copy over the contact force information
    conFState->cfi.clear();
    DynamicArray<ContactPoint> *cfs = pw->getContactForces();
    for (uint i=0;i<cfs->size();i++){
        conFState->cfi.push_back(ContactPoint((*cfs)[i]));
    }
}

/**
        populates the state of the framework with information passed in with the state of the framework
*/
void SimBiConFramework::setState(SimBiConFrameworkState& conFState){
    //set the state of the world, and that of the controller
    pw->setState(&(conFState.worldState));
    con->setControllerState(conFState.conState);
    lastFootPos = conFState.lastFootPos;
    DynamicArray<ContactPoint> *cfs = pw->getContactForces();
    cfs->clear();
    //now copy over the contact force information
    for (uint i=0;i<conFState.cfi.size();i++)
        cfs->push_back(ContactPoint(conFState.cfi[i]));
}


/*
this function is a quick and easy way to save the current controler and the current position
the to boolean are here to help choose which one is saved
*/
bool SimBiConFramework::save(bool save_controller, bool save_position){

    std::string line;

    if (Globals::save_to_current_controller){
        if (con->get_start_stance()==con->get_character()->stance()){

            if (save_position){
                //and we write it
                getCharacter()->saveReducedStateToFile(Globals::current_starting_pos_file);
            }


            //we read the name we want for the control file
            if (save_controller){
                //we add the prefix
                std::ostringstream oss2;
                oss2 << Globals::data_folder_path << "controllers/bipV2/";
                oss2 << line;

                //and we write it
                getController()->writeToFile(Globals::current_controller_file, &Globals::current_starting_pos_file);
            }
        }else{
            return false;
        }
    }else{

        std::ostringstream os;
        os << Globals::data_folder_path;
        os << Globals::primary_save_config;

        //the form of that function is to not have to do a lot of copy of the code (and because I'm too lazy to move it into another subfunction
        bool continues = false;
        int save_config_idx = 0;
        do{
            continues = false;
            std::ifstream myfile(os.str());
            if (myfile.is_open())
            {
                std::ostringstream oss;

                //so we read the name we want for the state file

                if (std::getline(myfile, line)){
                    //we add the prefix
                    oss << Globals::data_folder_path << "controllers/bipV2/";
                    oss << line;

                    if (save_position){

                        //and we write it
                        getCharacter()->saveReducedStateToFile(oss.str());
                    }
                }

                //we read the name we want for the control file
                if (std::getline(myfile, line)){
                    if (save_controller){
                        //we add the prefix
                        std::ostringstream oss2;
                        oss2 << Globals::data_folder_path << "controllers/bipV2/";
                        oss2 << line;

                        //and we write it
                        getController()->writeToFile(oss2.str(), &oss.str());
                    }
                }


                myfile.close();
            }
            else{
                std::cout << "failed save"<<std::endl;
                exit(56);
            }

            //now we check if there is a secondary save config
            if (save_config_idx==0&&!Globals::secondary_save_config.empty()){
                os.clear();
                os.str("");
                os << Globals::data_folder_path;
                os << Globals::secondary_save_config;
                continues = true;
                save_config_idx = 1;
            }


        } while (continues);
    }
    return true;
}
