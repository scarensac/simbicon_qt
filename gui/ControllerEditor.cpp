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

#include "ControllerEditor.h"
#include "Globals.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>
#include "gui/UI.h"
#include "Core/velocity_control/velocitycontroller.h"
#include "Physics/softbodypointmass.h"
#include "SPlisHSPlasH/Interface.h"
#include "Physics/EllipticalContacts.h"
/**
 * Constructor.
 */
ControllerEditor::ControllerEditor(void):InteractiveWorld(){
    end_thread=false;
    thread=NULL;
    semaphore_start=NULL;
    semaphore_end=NULL;
    if (Globals::simulateFluid){
        semaphore_start= new QSemaphore(0);
        semaphore_end= new QSemaphore(0);
        thread= new std::thread(fluid_exe_fct,this);
    }



    conF=NULL;

    /*
    std::remove("eval_estimate/hand_position_right.csv");
    std::remove("eval_estimate/hand_position_left.csv");
    std::remove("eval_estimate/head_pos.csv");
    std::remove("eval_estimate/head_angle.csv");
    //*/
    //std::cerr<<"Loading Controller Editor...\n");

    std::ostringstream oss;
    oss << Globals::data_folder_path;
    oss << Globals::init_folder_path;
    if (Globals::evolution_mode&&(Globals::close_after_evaluation)){
        oss << "input_optimisation.conF";
    }else{
        oss << "input.conF";
    }

    strcpy(inputFile,  oss.str().c_str());


    loadFramework();

    Globals::changeCurrControlShotStr( -1 );
    conF->getState(&conState);

    nextControlShot = 0;
    maxControlShot = -1;
    count_step = 0;

    perturbation_force=Vector3d(0,0,0);
    time_since_application=10000;

    reset_members_for_evaluation();


    {
        const std::vector<ControlParams>& cp=conF->getController()->get_control_params();
        const ControlParams& root_param=conF->getController()->get_root_control_params();
        double total_kp=0;
        double total_kd=0;

        total_kp=root_param.kp;
        total_kd=root_param.kd;

        for (int i=0;i<cp.size();++i){
            total_kp+=cp[i].kp;
        }

        for (int i=0;i<cp.size();++i){
            total_kd+=cp[i].kd;
        }

        if (!Globals::evolution_mode){
            double sum_eval=(total_kp+total_kd*10);
            std::cout<<"sum gains: "<<sum_eval<<std::endl;
        }
    }
}


/**
 * Destructor.
 */
ControllerEditor::~ControllerEditor(void){
    //first I close the fluid threading if I'm using it
    if(thread!=NULL){
        end_thread=true;
        semaphore_start->release();
        thread->join();

        delete thread;
        delete semaphore_start;
        delete semaphore_end;
    }

    delete conF;
    worldState.clear();

    nextControlShot = 0;
    maxControlShot = -1;

    clearEditedCurves();
}

/**
 * This method is used to create a physical world and load the objects in it.
 */
void ControllerEditor::loadFramework(){
    loadFramework( -1 );
}


/**
 * This method is used to create a physical world and load the objects in it.
 */
void ControllerEditor::loadFramework( int controlShot ){
    this->world = NULL;
    //create a new world, and load some bodies
    try{
        if( controlShot < 0 ) {
            conF = new SimBiConFramework(inputFile, NULL);
        }
        else {
            char conFile[256];
            sprintf(conFile, "..\\controlShots\\cs%05d.sbc", controlShot);
            conF = new SimBiConFramework(inputFile, conFile);
        }

        Globals::changeCurrControlShotStr( controlShot );
        conF->getState(&conState);
        this->world = conF->getWorld();
    }catch(const char* msg){
        conF = NULL;
        std::cout<<"Error: "<<std::string(msg)<<std::endl;
        logPrint("Error: %s\n", msg);
        exit(-1);
    }

}


/**
    This method draws the desired target that is to be tracked.
*/
void ControllerEditor::drawDesiredTarget(){
    Character* ch = conF->getCharacter();
    //we need to get the desired pose, and set it to the character
    DynamicArray<double> pose;
    //conF->getController()->updateTrackingPose(pose, Globals::targetPosePhase);
    conF->getController()->getDesiredPose(pose);


    glPushMatrix();
    Point3d p = ch->getRoot()->getCMPosition();
    //this is where we will be drawing the target pose
    p.x += 1;
    //p.y = 1;
    p.z += 0;

    Point3d old_cam_pos=Globals::window->getCameraTarget();
    Globals::window->setCameraTarget(p);

    worldState.clear();
    conF->getWorld()->getState(&worldState);

    pose[0] = 0;
    pose[1] = 0;
    pose[2] = 0;

    //    conF->getController()->change_swing_leg_target_rotation(SimGlobals::liquid_viscosity);
    //    ch->setState(&pose);
    conF->getController()->set_character_to_desired_pose(ch);


    //    conF->getController()->change_swing_leg_target_rotation(-SimGlobals::liquid_viscosity);


    glTranslated(p.x, p.y, p.z);

    //    TransformationMatrix tmp;
    //    double val[16];
    //    conF->getCharacterFrame().getRotationMatrix(&tmp);
    //    tmp.getOGLValues(val);
    //    glMultMatrixd(val);

    float tempColor[] = {0.5, 0.5, 0.5, 1.0};
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, tempColor);

    //    ch->stance_foot()->draw(SHOW_CD_PRIMITIVES);
    if (Globals::drawCollisionPrimitives){
        conF->getWorld()->drawRBs(SHOW_CD_PRIMITIVES);
    }else{
        conF->getWorld()->drawRBs(SHOW_MESH);
    }
    p = ch->getRoot()->getCMPosition();
    glDisable(GL_LIGHTING);
    glTranslated(p.x,p.y,p.z);
    GLUtils::drawAxes(0.2);
    glPopMatrix();
    glEnable(GL_LIGHTING);
    //set the state back to the original...
    conF->getWorld()->setState(&worldState);
    Globals::window->setCameraTarget(old_cam_pos);
}


/**
 * This method is called whenever the window gets redrawn.
 */
void ControllerEditor::draw(bool shadowMode){
    int flags = SHOW_MESH;
    if (shadowMode == false){
        if (time_since_application<0.2&& count_step>1){
            Point3d p=this->conF->getCharacter()->getRoot()->getCMPosition();
            Vector3d v=perturbation_force;
            double factor=0.001;
            glDisable(GL_LIGHTING);
            glColor3d(0.0, 0, 1);

            GLUtils::drawSphere( p,0.01*2);
            GLUtils::drawCylinder(0.005*2, v * 9*factor, p);
            GLUtils::drawCone(0.015*2, v * 3 * factor, p + v * 9 * factor);

            glEnable(GL_LIGHTING);

        }

    }
    /*
    if (shadowMode == false){


        Joint* j=conF->getCharacter()->stance_foot()->parent_joint();
        Point3d p=j->child()->getWorldCoordinates(j->child_joint_position());
        Vector3d v=j->parent()->getWorldCoordinates(Vector3d(1,0,0));
        {
            double factor=0.01;
            glDisable(GL_LIGHTING);
            glColor3d(0.0, 0, 1);

            GLUtils::drawSphere( p,0.01*2);
            GLUtils::drawCylinder(0.005*2, v * 9*factor, p);
            GLUtils::drawCone(0.015*2, v * 3 * factor, p + v * 9 * factor);

            glEnable(GL_LIGHTING);
        }

        v=j->parent()->getWorldCoordinates(Vector3d(0,0,1));
        {
            double factor=0.01;
            glDisable(GL_LIGHTING);
            glColor3d(0.0, 1, 0);

            GLUtils::drawSphere( p,0.01*2);
            GLUtils::drawCylinder(0.005*2, v * 9*factor, p);
            GLUtils::drawCone(0.015*2, v * 3 * factor, p + v * 9 * factor);

            glEnable(GL_LIGHTING);
        }

        j=j->parent()->parent_joint();
        p=j->child()->getWorldCoordinates(j->child_joint_position());
        v=j->parent()->getWorldCoordinates(Vector3d(1,0,0));
        {
            double factor=0.01;
            glDisable(GL_LIGHTING);
            glColor3d(0.0, 0, 1);

            GLUtils::drawSphere( p,0.01*2);
            GLUtils::drawCylinder(0.005*2, v * 9*factor, p);
            GLUtils::drawCone(0.015*2, v * 3 * factor, p + v * 9 * factor);

            glEnable(GL_LIGHTING);
        }
    }
//*/

    //if we are drawing shadows, we don't need to enable textures or lighting, since we only want the projection anyway
    if (shadowMode == false){
        flags |= SHOW_COLOURS;

        glEnable(GL_LIGHTING);
        glDisable(GL_TEXTURE_2D);
        if (Globals::drawCollisionPrimitives){
            flags |= SHOW_CD_PRIMITIVES | SHOW_FRICTION_PARTICLES;
            Globals::drawJoints=0;
        }
        if (Globals::drawJoints){
            flags |= SHOW_JOINTS;// | SHOW_BODY_FRAME;
        }


        Character* ch = conF->getCharacter();
        Point3d base=ch->getRoot()->getCMPosition();
        base.y=1.0;
        if ((IS_ZERO(SimGlobals::desiredHeading)&&Globals::motion_cycle_type!=0)||(Globals::motion_cycle_type==3)
                ||(Globals::motion_cycle_type==6)){
            //draw the desired heading
            Vector3d F=Quaternion::getRotationQuaternion(SimGlobals::desiredHeading,Vector3d(0,1,0)).rotate(Vector3d(0,0,1)*0.5/7);

            glDisable(GL_LIGHTING);
            glColor3d(1.0, 0.3, 0.3);
            GLUtils::drawCylinder(0.005*3, F *7 , base);
            GLUtils::drawCone(0.015*3, F  , base + F *7);

            glEnable(GL_LIGHTING);
        }

        //if it's the same heading, do nothing
        if (!ZERO_WITHIN_EPSILON(radian_distance_signed(SimGlobals::desiredHeading_active,SimGlobals::desiredHeading))&&Globals::use_fluid_heading){
            //draw the desired heading
            {
                Vector3d F=Quaternion::getRotationQuaternion(SimGlobals::desiredHeading_active,Vector3d(0,1,0)).rotate(Vector3d(0,0,1)*0.4/7);

                glDisable(GL_LIGHTING);
                glColor3d(0.3, 0.3, 1);
                GLUtils::drawCylinder(0.005*3, F *7 , base);
                GLUtils::drawCone(0.015*3, F  , base + F *7);
            }

            {
                Vector3d F=Quaternion::getRotationQuaternion(SimGlobals::desiredHeading,Vector3d(0,1,0)).rotate(Vector3d(0,0,1)*0.4/7);

                glDisable(GL_LIGHTING);
                glColor3d(0.3, 1, 0.3);
                GLUtils::drawCylinder(0.005*3, F *7 , base);
                GLUtils::drawCone(0.015*3, F  , base + F *7);
            }

            {

                Vector3d F=getConF()->getCharacter()->getHeading().rotate(Vector3d(0,0,1)*0.4/7);

                glDisable(GL_LIGHTING);
                glColor3d(1, 0.3, 0.3);
                GLUtils::drawCylinder(0.005*3, F *7 , base);
                GLUtils::drawCone(0.015*3, F  , base + F *7);
            }

            glEnable(GL_LIGHTING);


        }

        if (Globals::drawContactForces){
            //figure out if we should draw the contacts, desired pose, etc.
            glColor3d(0, 0, 1);

            //*
            DynamicArray<ContactPoint>* cfs = conF->getWorld()->getContactForces();
            for (uint i=0;i<cfs->size();i++){
                double factor = 0.0001;
                ContactPoint *c = &((*cfs)[i]);


                /*
                Vector3d v=Vector3d(0,c->f.y,0);
                GLUtils::drawCylinder(0.005, v* 9 *factor, c->cp);
                GLUtils::drawCone(0.015, v * 1 *factor, c->cp+v*9*factor);
                //*/

                //*
                c->f.x=0;
                c->f.z=0;
                GLUtils::drawCylinder(0.005, c->f * 9 *factor, c->cp);
                GLUtils::drawCone(0.015, c->f * 1 *factor, c->cp+c->f*9*factor);
                //*/
            }


            //*/
            //*
            std::vector<ForceStruct> vect = SimGlobals::vect_forces;
            for (uint i = 0; i < vect.size(); ++i){
                double factor = 0.005;
                glDisable(GL_LIGHTING);
                glColor3d(1, 0, 0);

                GLUtils::drawSphere( vect[i].pt,0.01);
                GLUtils::drawCylinder(0.005, vect[i].F * 9*factor, vect[i].pt);
                GLUtils::drawCone(0.015, vect[i].F * 3 * factor, vect[i].pt + vect[i].F * 9 * factor);

                glEnable(GL_LIGHTING);
                /*
                if (std::abs(vect[i].F.y)<0.0001)
                {
                    GLUtils::drawCylinder(0.005, vect[i].F * 9*factor, vect[i].pt);
                    GLUtils::drawCone(0.015, vect[i].F * 1 * factor, vect[i].pt + vect[i].F * 9 * factor);
                }
                else{
                    factor*=0.08;
                    glDisable(GL_LIGHTING);
                    glColor3d(0.0, 0, 1);

                    GLUtils::drawSphere( vect[i].pt,0.01*4);
                    GLUtils::drawCylinder(0.005*4, vect[i].F * 9*factor, vect[i].pt);
                    GLUtils::drawCone(0.015*4, vect[i].F * 3 * factor, vect[i].pt + vect[i].F * 9 * factor);

                    glEnable(GL_LIGHTING);
                }
                //*/

            }
            //*/

        }
    }

    if (conF == NULL)
        return;

    AbstractRBEngine* rbc = conF->getWorld();

    if (rbc){
        rbc->drawRBs(flags);

#ifdef COMPUTE_ELLIPTICAL_CONTACTS
        {
            conF->getCharacter()->left_contacts->draw();
            conF->getCharacter()->right_contacts->draw();
        }
#endif
    }
    //*
    AbstractRBEngine* rb_fw= SimGlobals::stanceFootWorld;

    if (rb_fw){
        rb_fw->drawRBs(flags);
        //        std::cout<<"drawing wirtual world"<<std::endl;
    }//*/

    if (shadowMode == false){
        //draw the pose if desirable...
        if (Globals::drawDesiredPose && conF && conF->getWorld()){
            drawDesiredTarget();
        }
    }

}



/**
 * This method is used to draw extra stuff on the screen (such as items that need to be on the screen at all times)
 */
void ControllerEditor::drawExtras(){
    if (Globals::drawCurveEditor == 1) {
        for( uint i = 0; i < curveEditors.size(); ++i )
            curveEditors[i]->draw();
    }else
        InteractiveWorld::drawExtras();
}


/**
 * This method is used to restart the application.
 */
void ControllerEditor::restart(){
    conF->restart(conState);
}

void ControllerEditor::stand()
{
    conF->stand(conState);
}


/**
 * This method is used to reload the application.
 */
void ControllerEditor::reload(){

    /*nextControlShot = 0;
    clearEditedCurves();

    delete conF;
    loadFramework();*/
}


/**
    This method is used to undo the last operation
*/
void ControllerEditor::undo(){
    if( nextControlShot <= 0 ) return;
    nextControlShot--;
    clearEditedCurves();
    delete conF;
    loadFramework(nextControlShot-1);
}

/**
    This method is used to redo the last operation
*/
void ControllerEditor::redo(){
    if( nextControlShot >= maxControlShot+1 ) return;
    nextControlShot++;
    clearEditedCurves();
    delete conF;
    loadFramework(nextControlShot-1);
}


void ControllerEditor::stepTaken() {


    if( Globals::updateDVTraj ) {

        if (conF) {

            ///TODO reactivate this
            /*
            SimBiConState *state = conF->getController()->states[ lastFSMState ];

            dTrajX.simplify_catmull_rom( 0.05 );
            dTrajZ.simplify_catmull_rom( 0.05 );
            vTrajX.simplify_catmull_rom( 0.05 );
            vTrajZ.simplify_catmull_rom( 0.05 );

            state->updateDVTrajectories(NULL, NULL, dTrajX, dTrajZ, vTrajX, vTrajZ );

            clearEditedCurves();
            addEditedCurve( state->dTrajX );
            addEditedCurve( state->dTrajZ );
            addEditedCurve( state->vTrajX );
            addEditedCurve( state->vTrajZ );
            //*/
        }

    }

    if( Globals::drawControlShots ) {
        //we just same the position and the controller
        bool not_need_retry=conF->save(Globals::save_controller,Globals::save_position);

        if (not_need_retry){
            if (Globals::close_after_saving){
                stop_gl();
            }

            Globals::drawControlShots = false;
        }
    }

    if (Globals::use_qt_interface){
        signal_messenger.emit_step_ended(Globals::avg_speed.z,Globals::avg_speed.x);
    }
}


/**
 *	This method is used when a mouse event gets generated. This method returns true if the message gets processed, false otherwise.
 */
bool ControllerEditor::onMouseEvent(int eventType, int button, int mouseX, int mouseY){

    //need to figure out if the mouse is in the curve editor (and if we care)...
    if ( Globals::drawCurveEditor == 1 ) {
        for( uint i = 0; i < curveEditors.size(); ++i )
            if( curveEditors[i]->onMouseEvent( eventType, button, mouseX, mouseY ) ) {
                return true;
            }
    }

    return InteractiveWorld::onMouseEvent(eventType, button, mouseX, mouseY);

}

void ControllerEditor::addEditedCurve(Trajectory1D *editedCurve, Vector3d axis) {
    double h_size=255;
    double v_size=h_size*9/16;
    if (curveEditors.size() < 2){
        curveEditors.push_back(new CurveEditor(h_size * curveEditors.size(), 0, h_size, v_size));
    }
    else{
        curveEditors.push_back(new CurveEditor(50+h_size * curveEditors.size(), 0, h_size, v_size));
    }
    curveEditors.back()->setTrajectory( editedCurve, axis );
}

/**
 * This method gets called when the application gets initialized.
 */
void ControllerEditor::init(){
    InteractiveWorld::init();
}

/**
 * This method returns the target that the camera should be looking at
 */
Point3d ControllerEditor::getCameraTarget(){
    if (conF == NULL)
        return Point3d(0,1,0);
    Character* ch = conF->getCharacter();
    if (ch == NULL)
        return Point3d(0,1,0);
    //we need to get the character's position. We'll hack that a little...
    return Point3d(ch->getRoot()->getCMPosition().x, 1, ch->getRoot()->getCMPosition().z);
}

void fluid_exe_fct(ControllerEditor* ce) {
    //*
    std::cout<<"init thread fluid"<<std::endl;
    while(!ce->end_thread){
        //std::cout<<"test2"<<std::endl;
        ce->semaphore_start->acquire();
        if (ce->end_thread){
            return;
        }
        ce->processFluidStepping();
        //std::cout<<"test3"<<std::endl;
        ce->semaphore_end->release();
        //std::cout<<"test4"<<std::endl;
    }//*/
}

/**
* This method will get called on idle. This method should be responsible with doing the work that the application needs to do 
* (i.e. run simulations, and so on).
*/
void ControllerEditor::processTask(){
    //if (Globals::animationRunning == 0){
    //	return;
    //}
    //*

    double simulationTime = 0;
    double maxRunningTime = 0.98/Globals::desiredFrameRate;


    double cur_height = 0;

    static double delta_time=0;



    //if we still have time during this frame, or if we need to finish the physics step, do this until the simulation time reaches the desired value
    //    while(true)
    //    while(simulationTime==0)
    while (simulationTime/maxRunningTime < Globals::animationTimeToRealTimeRatio)
    {

        simulationTime += SimGlobals::dt;


        //we just make sure that we don't try anything before the initialisation of the physical world
        if (conF) {
            /*
            if (count_step==0&&conF->getController()->getPhase()==0){
                Vector3d input;
                bInterface->handleMouseEvent(64, 64+52, &input);
//                bInterface->handleMouseEvent(64, 64-38, &input);
                std::cout<<input.y<<std::endl;
                Vector3d f(-input.x,0,input.y);
                f*=1500;
                apply_perturbation(f,true);;
            }
            //*/

            try{


                //std::chrono::steady_clock::time_point start_f = std::chrono::steady_clock::now();


                //clear the vector that's used to show the forces
                SimGlobals::vect_forces.clear();

                ODEWorld* ode_world = dynamic_cast<ODEWorld*>(world);

                if (Globals::parralelFluidAndControl){

                    //wait for fluid threading
                    semaphore_start->release();
                    //semaphore_end->acquire();//done later

                }else{
                    processFluidStepping();
                }

                if(Globals::simulateOnlyFluid){
                    if (Globals::parralelFluidAndControl){
                        semaphore_end->acquire();
                    }
                    continue;//do not touch the physics simulation
                }

                std::chrono::time_point<std::chrono::high_resolution_clock>start = std::chrono::high_resolution_clock::now();

                //init some variables
                conF->preprocess_simulation_step(SimGlobals::dt);

                //we can now advance the simulation
                conF->simulation_step(SimGlobals::dt);


                //run the simulation of physics bodies
                ode_world->sendDataToEngine();

                //std::chrono::time_point<std::chrono::high_resolution_clock> intermed = std::chrono::high_resolution_clock::now();

                if (Globals::parralelFluidAndControl){
                    semaphore_end->acquire();
                }

                //std::chrono::time_point<std::chrono::high_resolution_clock> intermed2 = std::chrono::high_resolution_clock::now();


                ode_world->advanceInTime(SimGlobals::dt);

                ode_world->readDataFromEngine();


                //post process the simulation
                conF->postprocess_simulation_step(SimGlobals::dt);

                SimGlobals::simu_time+=SimGlobals::dt;


                std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();

                std::chrono::duration<double> diff=end-start;
                delta_time+=diff.count();
/*
                std::chrono::steady_clock::time_point end_f = std::chrono::steady_clock::now();
                float time_iter = -std::chrono::duration_cast<std::chrono::nanoseconds> (start_f - end_f).count() / 1000000.0f;
                float time_solid= -std::chrono::duration_cast<std::chrono::nanoseconds> (intermed2 - end).count() / 1000000.0f;
                float time_control= -std::chrono::duration_cast<std::chrono::nanoseconds> (start - intermed).count() / 1000000.0f;
                static std::vector<double> times_iter;
                static std::vector<double> times_solid;
                static std::vector<double> times_control;

                times_iter.push_back(time_iter);
                times_solid.push_back(time_solid);
                times_control.push_back(time_control);

                if (times_iter.size()>333){
                    times_iter.erase(times_iter.begin());
                    times_solid.erase(times_solid.begin());
                    times_control.erase(times_control.begin());
                }


                for (int i=0;i<times_iter.size()-1;++i){
                    time_iter += times_iter[i];
                    time_solid += times_solid[i];
                    time_control += times_control[i];
                }
                time_iter /= times_iter.size();
                time_solid /= times_iter.size();
                time_control /= times_iter.size();

                std::cout<<"time iter: "<<time_iter<<"  with for control ; soli simu: "<<time_control<<"  "<<time_solid<<std::endl;
//*/
            }catch(char* c){
                std::string msg(c);
                std::cout<<msg<<std::endl;
                Globals::animationRunning=0;

                return;
            }catch(const char* c){
                std::string msg(c);
                std::cout<<msg<<std::endl;
                Globals::animationRunning=0;
                return;
            }catch(std::string s){
                std::cout<<s<<std::endl;
                Globals::animationRunning=0;
                return;
            }catch(...){
                std::cout<<"failed catching it"<<std::endl;
                Globals::animationRunning=0;
                return;
            }


            bool newStep = conF->new_character_step();





            lastFSMState = conF->getController()->getFSMState();



            //get the current phase, pose and state and update the GUI
            double phi = conF->getController()->getPhase();
            Globals::targetPosePhase = phi;


            static int count=0;
            count++;
            /*
            {
                std::ostringstream oss;
                double angle=conF->getController()->get_character()->getHeading().getRotationAngle(Vector3d(0,1,0));
                oss<< phi<< "  " <<count << " "<< angle<<std::endl;
                //std::cout<<oss.str();
                write_to_report_file(oss.str());
            }//*/

            /*
            {
                std::ostringstream oss;
                oss<< phi << " ";
                write_to_report_file(oss.str(),false);
            }//*/

            /*
            //this is a colision test (so I cna have the same thing at every execution
            Vector3d res;
            Point3d p;

            static bool not_done=true;
            if (count_step==5&&phi>0.4&&not_done){
                not_done=false;
                std::string name_buff="dodgeBall";
                RigidBody* dBall = world->getRBByName(name_buff.c_str());

                bool eventHandled = bInterface->handleMouseEvent(120,70,&res);;
                if (eventHandled ){
                    if (res.length() > 0){
                        //get the object that we will be throwing...
                        getDodgeBallPosAndVel(-res.x, res.y, res.length(), &p, &res);
                        dBall->setCMPosition(p);
                        dBall->setCMVelocity(res);
                        //				dBall->updateToWorldTransformation();
                    }
                }
            }
            //*/


            cur_height = conF->getController()->getSwingFootPos().y;

            if (Globals::use_qt_interface){
                signal_messenger.emit_new_phase(phi);
            }

            /*
            //if phi is lower than the last position of our trajectory, it means we changed phase and so we need to
            //reset the trajectory
            if( phi < dTrajX.getMaxPosition() ) {
                dTrajX.clear();
                dTrajZ.clear();
                vTrajX.clear();
                vTrajZ.clear();

            }

            //we add the current position to the trajectory
            Vector3d d = conF->getController()->get_d();
            Vector3d v = conF->getController()->get_v();
            dTrajX.addKnot( phi, d.x * signChange);
            dTrajZ.addKnot( phi, d.z  );
            vTrajX.addKnot( phi, v.x * signChange );
            vTrajZ.addKnot( phi, v.z  );
            //*/

            /*
            Globals::evolution_mode=true;
            Globals::evolution_type=1;
            SimGlobals::steps_before_evaluation = 20;
            SimGlobals::evaluation_length = 5;
            //*/
            bool valid_evolution=false;
            double eval_result=handle_evolution(phi,count_step,valid_evolution);
            Globals::motion_cycle_type=0;
            if (!Globals::evolution_mode){
                if (Globals::motion_cycle_type==95){
                    static unsigned int step_limit=5;
                    if (count_step>step_limit){
                        step_limit+=5;
                        if (eval_push_phase==0){
                            SimGlobals::velDCoronal=0.2;
                        } else if (eval_push_phase==1){
                            SimGlobals::velDCoronal=0.0;
                        } else if (eval_push_phase==2){
                            SimGlobals::velDCoronal=-0.2;
                        }else if (eval_push_phase==3){
                            SimGlobals::velDCoronal=0.0;
                        }else if (eval_push_phase==4){
                            eval_push_phase=-1;
                            if (count_step>190){
                                Globals::animationRunning=0;
                                std::cout<<"time used :"<<delta_time<<std::endl;
                            }
                        }
                        eval_push_phase++;
                    }
                }else if (Globals::motion_cycle_type==96){
                    static unsigned int step_limit=5;
                    if (count_step>step_limit){
                        step_limit+=10;
                        if (eval_push_phase==0){
                            SimGlobals::velDCoronal=0.2;
                            //                        eval_push_phase=6;
                            step_limit-=5;
                        } else if (eval_push_phase==1){
                            SimGlobals::velDSagittal=0.4;
                        } else if (eval_push_phase==2){
                            SimGlobals::velDCoronal=0.0;
                            SimGlobals::velDSagittal=0.0;
                        }else if (eval_push_phase==3){
                            SimGlobals::velDSagittal=0.4;
                        }else if (eval_push_phase==4){
                            SimGlobals::velDSagittal=0.7;
                            step_limit+=5;
                        }else if (eval_push_phase==5){
                            SimGlobals::desiredHeading=1.0;
                            step_limit+=1;
                        }else if (eval_push_phase==6){
                            SimGlobals::desiredHeading=0.0;
                            step_limit+=5;
                        }else if (eval_push_phase==7){

                            Vector3d f(-1,0,1);
                            f*=125;
                            apply_perturbation(f,true);
                            step_limit-=4;
                        }else if (eval_push_phase==8){
                            Vector3d f(1,0,0);
                            f*=100;
                            apply_perturbation(f,true);
                            step_limit-=4;
                        }else if (eval_push_phase==9){
                            Vector3d f(0,0,-1);
                            f*=150;
                            apply_perturbation(f,true);
                        }else if (eval_push_phase==10){
                            Globals::animationRunning=0;

                        }else if (eval_push_phase==8){
                            //                        eval_push_phase=-1;
                            if (count_step>190){
                                std::cout<<"time used :"<<delta_time<<std::endl;
                            }
                        }
                        eval_push_phase++;
                    }
                }else if (Globals::motion_cycle_type==2){
                    static unsigned int step_limit=5;
                    if (count_step>step_limit){
                        step_limit+=10;
                        if (eval_push_phase==0){
                            SimGlobals::velDCoronal=0.2;
                            step_limit-=5;
                        } else if (eval_push_phase==1){
                            SimGlobals::velDSagittal=0.4;
                        } else if (eval_push_phase==2){
                            SimGlobals::velDCoronal=0.0;
                            SimGlobals::velDSagittal=0.0;
                        }else if (eval_push_phase==3){
                            SimGlobals::velDSagittal=0.4;
                            step_limit-=8;
                        }else if (eval_push_phase==4){
                            SimGlobals::velDSagittal=0.7;
                        }
                        eval_push_phase++;
                    }

                }else if (Globals::motion_cycle_type==3){
                    static unsigned int step_limit=6;
                    if (count_step>step_limit){
                        step_limit+=10;
                        if (eval_push_phase==0){
                            SimGlobals::desiredHeading=1;
                            step_limit-=3;
                            std::cout<<"count"<<count_step<<"  "<<step_limit<<std::endl;
                        } else if (eval_push_phase==1){
                            SimGlobals::desiredHeading=0;
                            std::cout<<"count"<<count_step<<"  "<<step_limit<<std::endl;
                            step_limit-=4;
                        }else if (eval_push_phase==2){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=-1.2;
                        }
                        eval_push_phase++;
                    }

                }else if (Globals::motion_cycle_type==4){
                    static unsigned int step_limit=21;
                    if (count_step>step_limit){
                        step_limit+=9;
                        if (eval_push_phase==0){
                            Vector3d f(-1,0,1);
                            f*=85;
                            apply_perturbation(f,true);
                            step_limit-=2;
                        }else if (eval_push_phase==1){
                            Vector3d f(1,0,0);
                            f*=90;
                            apply_perturbation(f,true);
                            step_limit-=4;
                        }else if (eval_push_phase==2){
                            Vector3d f(0,0,-1);
                            f*=150;
                            apply_perturbation(f,true);
                        }
                        eval_push_phase++;
                    }

                }else if (Globals::motion_cycle_type==5){
                    static unsigned int step_limit=11;
                    if (count_step>step_limit){
                        step_limit+=10;
                        if (eval_push_phase==0){
                            Vector3d f(-1,0,1);
                            f*=90;
                            apply_perturbation(f,true);
                            step_limit-=3;
                        }else if (eval_push_phase==1){
                            Vector3d f(1,0,1);
                            f*=105;
                            apply_perturbation(f,true);
                            step_limit-=2;
                        }
                        eval_push_phase++;
                    }

                }else if (Globals::motion_cycle_type==6){
                    static unsigned int step_limit=4;
                    if (count_step>step_limit){
                        step_limit+=10;
                        if (eval_push_phase==0){
                            SimGlobals::desiredHeading=0.2;
                            step_limit-=3;
                            std::cout<<"count"<<count_step<<"  "<<step_limit<<std::endl;
                        } else if (eval_push_phase==1){
                            SimGlobals::desiredHeading=-0.2;
                            std::cout<<"count"<<count_step<<"  "<<step_limit<<std::endl;
                            step_limit-=3;
                        }else if (eval_push_phase==2){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=0.4;
                            step_limit-=3;
                        }else if (eval_push_phase==3){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=-0.4;
                            step_limit-=3;
                        }else if (eval_push_phase==4){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=0.6;
                            step_limit-=3;
                        }else if (eval_push_phase==5){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=-0.6;
                            step_limit-=3;
                        }else if (eval_push_phase==6){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=0.8;
                            step_limit-=3;
                        }else if (eval_push_phase==7){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=-0.8;
                            step_limit-=3;
                        }else if (eval_push_phase==8){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=1;
                            step_limit-=3;
                        }else if (eval_push_phase==9){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=-1;
                            step_limit-=3;
                        }
                        eval_push_phase++;
                    }

                }else if (Globals::motion_cycle_type==7){
                    static unsigned int step_limit=5;
                    if (count_step>step_limit){
                        step_limit+=10;
                        if (eval_push_phase==0){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=0.4;
                            step_limit-=3;
                        }else if (eval_push_phase==1){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=-0.4;
                            step_limit-=3;
                        }else if (eval_push_phase==2){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=0.6;
                            step_limit-=3;
                        }else if (eval_push_phase==3){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=-0.6;
                            step_limit-=3;
                        }else if (eval_push_phase==4){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=0.8;
                            step_limit-=3;
                        }else if (eval_push_phase==5){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=-0.8;
                            step_limit-=3;
                        }else if (eval_push_phase==6){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=1;
                            step_limit-=3;
                        }else if (eval_push_phase==7){
                            std::cout<<"start"<<std::endl;
                            SimGlobals::desiredHeading=-1;
                            step_limit-=3;
                        }
                        eval_push_phase++;
                    }


                }else if (Globals::motion_cycle_type==8){
                    static unsigned int step_limit=5;
                    if (count_step>step_limit){
                        step_limit+=11;
                        if (eval_push_phase==0){
                            SimGlobals::desiredHeading=1.2;
                            step_limit-=3;
                            std::cout<<"count"<<count_step<<"  "<<step_limit<<std::endl;
                        } else if (eval_push_phase==1){
                            SimGlobals::desiredHeading=-0.2;
                            std::cout<<"count"<<count_step<<"  "<<step_limit<<std::endl;
                            step_limit-=4;
                        }
                        eval_push_phase++;
                    }

                }else if (Globals::motion_cycle_type==9){
                    static unsigned int step_limit=15;
                    if (count_step>step_limit){
                        step_limit+=11;
                        if (eval_push_phase==0){
                            SimGlobals::desiredHeading=0.5;
                            step_limit-=3;
                            std::cout<<"count"<<count_step<<"  "<<step_limit<<std::endl;
                        } else if (eval_push_phase==1){
                            SimGlobals::desiredHeading=-0.0;
                            std::cout<<"count"<<count_step<<"  "<<step_limit<<std::endl;
                            step_limit-=1;
                        } else if (eval_push_phase==2){
                            Vector3d f(-1,0,1);
                            f*=60;
                            apply_perturbation(f,true);
                            step_limit-=2;
                        }
                        eval_push_phase++;
                    }

                }else if (Globals::motion_cycle_type==100){
                    static unsigned int step_limit=15;
                    if (count_step>step_limit){
                        step_limit+=15;
                        if (eval_push_phase==0){
                            Vector3d f(-1,0,1);
                            f*=100;
                            apply_perturbation(f,true);
                            write_to_report_file("perturbation declared");
                        }else if (eval_push_phase==1){
                            Vector3d f(1,0,-0.5);
                            f*=100;
                            apply_perturbation(f,true);
                            write_to_report_file("perturbation declared");
                        }else if (eval_push_phase==2){
                            SimGlobals::velDSagittal=0.9;
                        }else if (eval_push_phase==3){
                            SimGlobals::velDSagittal=0.5;
                        }else if (eval_push_phase==4){
                            SimGlobals::velDSagittal=0.7;
                            SimGlobals::velDCoronal=0.2;
                        }
                        eval_push_phase++;
                    }

                }else if (Globals::motion_cycle_type==101){
                    static unsigned int step_limit=20;
                    if (count_step>step_limit){
                        step_limit+=15;
                        if (eval_push_phase==0){
                            SimGlobals::velDSagittal=0.4;
                            write_to_report_file("perturbation declared");
                        }
                        /*
                        else if (eval_push_phase==1){
                            SimGlobals::velDSagittal=0.7;
                            SimGlobals::velDCoronal=0.2;
                            write_to_report_file("perturbation declared");
                        }//*/
                        eval_push_phase++;
                    }

                }
            }

            if(eval_result>=0.0f){

                /*
                std::ostringstream oss30;
                oss30<<"eval_seen1: "<<eval_result<<"  count steps: "<<count_step;
                std::cout<<oss30.str();



                /*
                const std::vector<ControlParams>& cp=conF->getController()->get_control_params();
                const ControlParams& root_param=conF->getController()->get_root_control_params();

                oss30.str("");
                oss30<<"kp kd: "<<root_param.kp<<"   "<<root_param.kd<<std::endl;

                for (int i=0;i<cp.size();++i){
                    oss30<<"kp kd: "<<cp[i].kp<<"   "<<cp[i].kd<<std::endl;
                }
                std::cout<<oss30.str();


                //*/

                //we handle the end of the evalutation
                double old_eval=Globals::simulation_eval;
                Globals::simulation_eval=eval_result;
                count_step=0;

                bool evaluation_end=true;
                int temp_eval_push_phase=eval_push_phase;
                reset_members_for_evaluation();
                eval_push_phase=temp_eval_push_phase;
                if (Globals::evolution_push_type==1){
                    if (eval_push_phase==0){
                        Vector3d res;
                        Point3d p;
                        bool eventHandled = bInterface->handleMouseEvent(110,64,&res);;
                        if (eventHandled ){
                            Vector3d f(-res.x,0,res.y);
                            f*=10000*5;
                            apply_perturbation_on_pelvis(f);
                            //                            getDodgeBallPosAndVel(-res.x, res.y, res.length(), &p, &res);
                        }

                        evaluation_end=false;
                    } else if (eval_push_phase==1){
                        Vector3d res;
                        Point3d p;
                        bool eventHandled = bInterface->handleMouseEvent(18,64,&res);;
                        if (eventHandled ){
                            getDodgeBallPosAndVel(-res.x, res.y, res.length(), &p, &res);
                        }

                        Globals::simulation_eval+=old_eval;
                        evaluation_end=false;
                    } else if (eval_push_phase==2){
                        Globals::simulation_eval+=old_eval;
                    }
                    eval_push_phase++;

                }else if (Globals::evolution_push_type==2){

                    if (eval_push_phase==0){
                        SimGlobals::velDCoronal=0.2;
                        evaluation_end=false;
                    } else if (eval_push_phase==1){
                        Globals::simulation_eval+=old_eval;
                        SimGlobals::velDCoronal=0.0;
                        evaluation_end=false;
                    } else if (eval_push_phase==2){
                        Globals::simulation_eval+=old_eval;
                        SimGlobals::velDCoronal=-0.2;
                        evaluation_end=false;
                    }else if (eval_push_phase==3){
                        Globals::simulation_eval+=old_eval;
                    }
                    eval_push_phase++;
                }


                //*
                if (evaluation_end){
                    Globals::animationRunning=0;
                    reset_members_for_evaluation();
                    if (valid_evolution&&Globals::save_mode){
                        Globals::drawControlShots = true;
                        Globals::close_after_saving = true;
                        Globals::evolution_mode = false;
                        Globals::animationRunning = 1;
                    }else{
                        if (Globals::close_after_evaluation){
                            stop_gl();
                        }else{
                            std::cout<<"eval_seen without close after saving: "<<Globals::simulation_eval<<"  count steps: "<<count_step<<std::endl;
                        }
                        break;
                    }
                }
                //*/
            }


            //we now check if we finished our current step and act accordingly
            if( newStep ) {
                count_step++;
                Vector3d v = conF->getLastStepTaken();


                /*
                {
                    std::ostringstream oss;
                    oss<< count_step << " ";
                    write_to_report_file(oss.str(),false);
                }//*/
                /*
                {
                    std::ostringstream oss;
                    oss<< count_step << " "<< conF->getController()->ipm_alt_sagittal() <<" "<<
                          conF->getController()->velocity_controller->last_virt_force_signed.z << " "<<
                          conF->getController()->velocity_controller->last_virt_force_signed.length()<<" "<<
                            Globals::avg_speed.z;
                    write_to_report_file(oss.str());
                }//*/
                /*
                {
                    std::ostringstream oss;
                    oss<< count_step << " "<<Globals::avg_speed.x<<" "<<Globals::avg_speed.z<< " "<<
                          SimGlobals::velDCoronal<<" "<<SimGlobals::velDSagittal<<std::endl;
                    write_to_report_file(oss.str());
                }//*/


                if (!Globals::evolution_mode&&!Globals::close_after_saving)
                {
                    /*
                    if (conF->getController()->need_recovery_step()){
                        std::cerr<<"recovery: %lf %lf %lf (phi = %lf, avg_speed_x = %lf, avg_speed_z = %lf, TIME = %lf, ipm_alt_sagittal = %lf)\n",
                            v.x, v.y, v.z, phi, Globals::avg_speed.x, Globals::avg_speed.z, step_time_end, conF->ipm_alt_sagittal*SimGlobals::ipm_alteration_effectiveness);

                    }
                    else{
                        std::cerr<<"step: %lf %lf %lf (phi = %lf, avg_speed_x = %lf, avg_speed_z = %lf, TIME = %lf, ipm_alt_sagittal = %lf)\n",
                            v.x, v.y, v.z, phi, Globals::avg_speed.x, Globals::avg_speed.z,  step_time_end, conF->ipm_alt_sagittal*SimGlobals::ipm_alteration_effectiveness);
                    }
                    //*/

                    std::ios_base::fmtflags f( std::cout.flags() );


                    std::cout<<std::setprecision(5);
                    std::cout<<std::fixed;
                    std::cout<<"n°"<<count_step<<"  ";

                    if (conF->getController()->need_recovery_step()){
                        std::cout<<"recovery: ";
                    }else{
                        std::cout<<"step: ";
                    }
                    std::cout<<v.x<<" "<<v.y<<" "<<v.z<<" ";
                    std::cout<<"(";
                    std::cout<<"phi = "<<conF->getController()->get_phi_last_step()<<", ";
                    std::cout<<"velocity(x,z) = "<<Globals::avg_speed.x<<" "<<Globals::avg_speed.z<<", ";
                    std::cout<<"ipm_atl_sagittal = "<<conF->getController()->ipm_alt_sagittal()<<", ";
                    std::cout<<"VelD offset = "<<conF->getController()->velocity_controller->velD_coronal_left_offset <<" "<<
                               conF->getController()->velocity_controller->velD_coronal_right_offset <<" "<<
                               conF->getController()->velocity_controller->velD_sagittal_offset ;
                    std::cout<<" force limit ratio: "<<conF->getController()->velocity_controller->get_force_limit_ratio();
                    std::cout<<")";
                    std::cout<< std::endl;


                    std::cout.flags( f );

                }
                //                std::cout<<"step: %lf %lf %lf (phi = %lf, avg_speed_x = %lf, avg_speed_z = %lf, TIME = %lf, ipm_alt_sagittal = %lf)\n",
                //                        v.x, v.y, v.z, phi, Globals::avg_speed.x, Globals::avg_speed.z,step_time_end,
                //                        conF->getController()->ipm_alt_sagittal();


                //this print is to draw the curve of evo of the speeds
                //std::cerr<<"%lf, %lf\n",Globals::avg_speed.x, Globals::avg_speed.z);


                stepTaken();

                //get the reversed new state...
                /*
                DynamicArray<double> newState;
                if (conF->getController()->getStance() == RIGHT_STANCE)
                    conF->getCharacter()->getReverseStanceState(&newState);
                else
                    conF->getCharacter()->getState(&newState);

                ReducedCharacterState rNew(&newState);

                conF->getCharacter()->saveReducedStateToFile("out\\reducedCharacterState.rs", newState);
                //*/
            }

            //			if (conF->getController()->isBodyInContactWithTheGround()){
            //				std::cerr<<"Lost control of the biped...\n");
            //				Globals::animationRunning = false;
            //				break;
            //			}
            //		break;
        }

        apply_perturbation();


    }
}




void ControllerEditor::processFluidStepping(){
    //std::chrono::steady_clock::time_point start_f = std::chrono::steady_clock::now();

    ODEWorld* ode_world = dynamic_cast<ODEWorld*>(getWorld());
    SimBiConFramework* simbiconframework=getConF();

    if (Globals::simulateFluid){


        if (!Globals::fluidControlLevel){
            signal_messenger.emit_fluid_level(Interface::getFluidLevel());
        }else{
            Interface::handleFLuidLevelControl(SimGlobals::water_level);
        }

        if (Globals::fluidFollowCharacter){
            Point3d pt(simbiconframework->getCharacter()->getCOM().toPoint3d());
            bool moved=Interface::moveFluidSimulation(pt);
            //Interface::moveFluidSimulation(Vector3d(-conF->getCharacter()->getCOM().x,0,-conF->getCharacter()->getCOM().z));
            if (moved){
                //Globals::animationRunning=0;
            }
        }

        if (Globals::simulateOnlyFluid){
            WaterImpact current_impact;

            //if we only do the fluid it mean the rb are paused
            ode_world->sendDataToParticleFluidEngine();
            Interface::handleDynamicBodiesPause(true);
            ode_world->advanceInTimeParticleFluidEngine(SimGlobals::dt);
            //show the forces

            ode_world->readDataFromParticleFluidEngine(current_impact,simbiconframework->getCharacter());


            for (int i=0;i<current_impact.getNumBodies();++i){
                SimGlobals::vect_forces.push_back(current_impact.getBoyancyInOrder(i));
                SimGlobals::vect_forces.push_back(current_impact.getDragInOrder(i));
            }


            //and since the sim of rb is paused the controler step is done :)
            return;
        }else{
            //otherwise they are simulated
            Interface::handleDynamicBodiesPause(false);

            ode_world->sendDataToParticleFluidEngine();
            ode_world->advanceInTimeParticleFluidEngine(SimGlobals::dt);
            ode_world->readDataFromParticleFluidEngine(simbiconframework->resulting_impact,simbiconframework->getCharacter());

            {
                ForceStruct impact=simbiconframework->resulting_impact.getDrag(simbiconframework->getCharacter()->stance_foot()->idx());
                //SimGlobals::vect_forces.push_back(impact_sim);
                //std::cout<<"  "<<impact.F.x<<"  "<<impact.F.y<<"  "<<impact.F.z<<std::endl;
            }
            int starting_step=10;
            bool run_comparison=false;
            if (run_comparison&&count_step>=starting_step){
                if (count_step==starting_step+50){
                    Globals::animationRunning=0;
                }

                //"rUpperleg","rLowerleg","rFoot"
                std::string objName="rLowerleg";

                ForceStruct impact_sim=simbiconframework->resulting_impact.getDrag(simbiconframework->getCharacter()->getARBByName(objName)->idx());
                SimGlobals::vect_forces.push_back(impact_sim);


                ode_world->compute_water_impact(simbiconframework->getCharacter(),1.0, simbiconframework->resulting_impact);

                ForceStruct impact_reduced_drag=simbiconframework->resulting_impact.getDrag(simbiconframework->getCharacter()->getARBByName(objName)->idx());
                ForceStruct impact_reduced_boy=simbiconframework->resulting_impact.getBoyancy(simbiconframework->getCharacter()->getARBByName(objName)->idx());
                ForceStruct impact_reduced;
                impact_reduced.F=impact_reduced_boy.F+impact_reduced_drag.F;
                impact_reduced.M=impact_reduced_boy.M+impact_reduced_drag.M;


                static std::vector<std::vector<ForceStruct> > even_steps_simu;
                static std::vector<std::vector<ForceStruct> > even_steps_esti;
                static std::vector<std::vector<ForceStruct> > odd_steps_simu;
                static std::vector<std::vector<ForceStruct> > odd_steps_esti;


                static std::vector<ForceStruct> cur_step_simu;
                static std::vector<ForceStruct> cur_step_esti;

                static int old_count_step=starting_step;
                if (old_count_step!=count_step){
                    std::vector<std::string> filenames;
                    std::vector<std::vector<std::vector<ForceStruct> >* > vect_steps_data;
                    if((old_count_step%2)==0){
                        even_steps_simu.push_back(cur_step_simu);
                        even_steps_esti.push_back(cur_step_esti);
                        filenames.push_back("fluid_impact_even_simu.csv");
                        vect_steps_data.push_back(&even_steps_simu);
                        filenames.push_back("fluid_impact_even_esti.csv");
                        vect_steps_data.push_back(&even_steps_esti);

                    }else{
                        odd_steps_simu.push_back(cur_step_simu);
                        odd_steps_esti.push_back(cur_step_esti);
                        filenames.push_back("fluid_impact_odd_simu.csv");
                        vect_steps_data.push_back(&odd_steps_simu);
                        filenames.push_back("fluid_impact_odd_esti.csv");
                        vect_steps_data.push_back(&odd_steps_esti);
                    }

                    cur_step_simu.clear();
                    cur_step_esti.clear();
                    //writte to file


                    for (int l=0;l<2;++l){
                        std::string filename = filenames[l];
                        std::vector<std::vector<ForceStruct> >& steps_data=*(vect_steps_data[l]);

                        std::remove(filename.c_str());

                        //we need to look the the min number of iter in a step
                        int min_count=steps_data[0].size();
                        for (int i=1;i<steps_data.size();++i){
                            min_count=((min_count<=steps_data[i].size())?min_count:steps_data[i].size());
                        }

                        std::ostringstream oss;
                        for(int i=0;i<min_count;++i){
                            oss<<i;
                            //compute the avg values and writte them
                            Vector3d force=Vector3d(0,0,0);
                            for (int k=0;k<steps_data.size();++k){
                                ForceStruct impact=steps_data[k][i];
                                force+=impact.F;
                            }
                            force/=steps_data.size();
                            oss<<"  "<<force.x<<"  "<<force.y<<"  "<<force.z<<"  //  ";


                            for (int k=0;k<steps_data.size();++k){
                                ForceStruct impact=steps_data[k][i];
                                oss<<"  "<<impact.F.x<<"  "<<impact.F.y<<"  "<<impact.F.z;
                            }
                            oss<<std::endl;
                        }

                        //and now we can writte
                        std::ofstream myfile;
                        myfile.open(filename);
                        if (myfile.is_open()) {
                            myfile<<oss.str();
                            myfile.close();
                        }
                        else {
                            std::cout << "failed to open file: " << filename << "   reason: " << std::strerror(errno) << std::endl;
                        }
                    }
                }
                old_count_step=count_step;

                //here Ijust have to store the information in the vector

                cur_step_simu.push_back(impact_sim);
                cur_step_esti.push_back(impact_reduced);

            }

        }


    }else{
        //we simulate the effect of the liquid
        ode_world->compute_water_impact(simbiconframework->getCharacter(),SimGlobals::water_level, simbiconframework->resulting_impact);
    }


    if (!Globals::simulateOnlyFluid){

        bool apply_forces=true;
        bool show_forces=false;


        for (int i=0;i<ode_world->getRBCount();++i){
            RigidBody* body=ode_world->getRBByIdx(i);

            ForceStruct impact_drag=simbiconframework->resulting_impact.getDrag(body->idx());
            ForceStruct impact_boyancy=simbiconframework->resulting_impact.getBoyancy(body->idx());

            if (apply_forces){
                /*
                if(body==conF->getCharacter()->stance_hip()->child()||
                        body==conF->getCharacter()->stance_foot()||
                        body==conF->getCharacter()->stance_foot()->parent_joint()->parent()){
                    continue;
                }
                //*/

                //apply the forces and moments
                if (!impact_drag.F.isZeroVector()){
                    ode_world->applyForceTo(body, impact_drag.F, body->getLocalCoordinates(impact_drag.pt));
                    if (!impact_drag.M.isZeroVector()){
                        ode_world->applyTorqueTo(body,impact_drag.M);
                    }
                    //std::cout<<"drag force found: "<<body->name()<<std::endl;
                }

                if (!Globals::simulateFluid){
                    if (!impact_boyancy.F.isZeroVector()){
                        ode_world->applyForceTo(body, impact_boyancy.F, body->getLocalCoordinates(impact_boyancy.pt));
                        if (!impact_boyancy.M.isZeroVector()){
                            ode_world->applyTorqueTo(body,impact_boyancy.M);
                        }
                        //std::cout<<"boyancy force found "<<body->name()<<std::endl;
                    }
                }
            }


            if (show_forces){
                //*
                //this can be used to show the forces
                SimGlobals::vect_forces.push_back(impact_drag);
                //SimGlobals::vect_forces.push_back(impact_boyancy);
                //*/

            }
        }

        //this is to make a test on the boyancy values
        if (false){

            Vector3d sumForces;
            {
                for (int i=0;i<ode_world->getRBCount();++i){
                    RigidBody* body=ode_world->getRBByIdx(i);
                    ForceStruct impact_drag=simbiconframework->resulting_impact.getDrag(body->idx());

                    //apply the forces and moments
                    if (!impact_drag.F.isZeroVector()){
                        sumForces+=impact_drag.F;
                    }
                }
            }

            Vector3d sumGRF;
            {
                //that is the normal thing to do to obtain the grf
                /*
                for(int i=0;i<4;++i){
                    sumGRF+=conF->getCharacter()->get_force_on_foot(true,true);
                    sumGRF+=conF->getCharacter()->get_force_on_foot(false,true);
                }
                //*/

                //this is just a way to highjack that fucntion to compare the fluid force
                //for a random single objet

                ode_world->estimate_water_impact_on_particle_objects(4.0, simbiconframework->resulting_impact);
                for (int i=0;i<ode_world->getRBCount();++i){
                    RigidBody* body=ode_world->getRBByIdx(i);
                    ForceStruct impact_drag=simbiconframework->resulting_impact.getDrag(body->idx());
                    ForceStruct impact_boyancy=simbiconframework->resulting_impact.getBoyancy(body->idx());

                    //apply the forces and moments
                    if (!impact_drag.F.isZeroVector()){
                        sumGRF+=impact_drag.F;
                    }

                    if(!impact_boyancy.F.isZeroVector()){
                        sumGRF+=impact_boyancy.F;
                    }

                }

            }

            std::string filename = "drag_on_linear_sphere.csv";
            static bool first_time=true;
            if (first_time){
                std::remove(filename.c_str());
                first_time=false;
            }


            std::ostringstream oss;
            //oss<<conF->getCharacter()->stance_foot()->getAngularVelocity().y<<" ";
            oss<<ode_world->vect_objects_fluid_interaction[0]->getCMVelocity().z<<" ";
            oss<<sumForces.x<<" "<<sumForces.y<<" "<<sumForces.z<<" "<<sumGRF.x<<" "<<sumGRF.y<<" "<<sumGRF.z<<std::endl;

            //and now we can writte
            std::ofstream myfile;
            myfile.open(filename,std::ios_base::app);
            if (myfile.is_open()) {
                myfile<<oss.str();
                myfile.close();
            }
            else {
                std::cout << "failed to open file: " << filename << "   reason: " << std::strerror(errno) << std::endl;
            }
        }
    }
    /*
    std::chrono::steady_clock::time_point end_f = std::chrono::steady_clock::now();
    float time_iter = std::chrono::duration_cast<std::chrono::nanoseconds> (start_f - end_f).count() / 1000000.0f;

    static std::vector<double> times_iter;

    times_iter.push_back(time_iter);

    if (times_iter.size()>333){
        times_iter.erase(times_iter.begin());
    }

    for (int i=0;i<times_iter.size()-1;++i){
        time_iter += times_iter[i];
    }
    time_iter /= times_iter.size();

    std::cout<<"timefluid: "<<time_iter<<std::endl;
    //*/
}

//*
int ControllerEditor::trajectoryToEdit(int idx){

    SimBiConState* state = getFramework()->getController()->cur_state();
    if( !state ) return  -1;

    Trajectory* trajectory = state->getTrajectory( idx );
    if( !trajectory ) return  -2;


    clearEditedCurves();
    for( uint i = 0; i < trajectory->components.size(); ++i ) {
        /*
        //forimagesforpaper
        if (trajectory->name()=="velD"&&trajectory->components[i]->rotationAxis!=Vector3d(0,0,1)){
            continue;
        }
        //*/

        addEditedCurve( &trajectory->components[i]->baseTraj,trajectory->components[i]->rotationAxis );
    }
    if( trajectory->strengthTraj != NULL ) {
        addEditedCurve( trajectory->strengthTraj );
    }

    return 0;
}

std::vector<std::string> ControllerEditor::get_trajectory_name_list()
{
    std::vector<std::string> name_list;
    SimBiConState* state = getFramework()->getController()->cur_state();
    if(state){
        for (int i=0; i<state->getTrajectoryCount();++i){
            name_list.push_back(state->getTrajectory(i)->name());
        }
    }
    return name_list;
}

double ControllerEditor::handle_evolution(double phi, int count_step , bool &saving_possible)
{

    //if we fall we just stop
    if (!Globals::evolution_mode){
        last_phi=phi;
        return -1;
    }



    saving_possible=false;

    //we stop if we fall

    if(conF->getCharacter()->getRoot()->getCMPosition().y<0.3){
        return (double)1E22;
    }

    //if there is a gain lower than 0 throw the simulation
    if (Globals::evolution_type==1){
        const std::vector<ControlParams>& cp=conF->getController()->get_control_params();
        const ControlParams& root_param=conF->getController()->get_root_control_params();

        if (root_param.kp<=0||root_param.kd<=0){
            return (double)1E21;
        }

        for (int i=0;i<cp.size();++i){
            if (cp[i].kp<=0||cp[i].kd<=0){
                return (double)1E21;
            }
        }
    }

    //just to make sure we always start with a left stance
    int needed_stabilisation_steps=SimGlobals::steps_before_evaluation;
    if (count_step==(needed_stabilisation_steps+1)){
        if (!conF->getCharacter()->is_left_stance()){
            needed_stabilisation_steps++;
        }
    }

    if (count_step>(uint)needed_stabilisation_steps){

        //we stop if we need a recovery step during the evaluation
        if (conF->getController()->need_recovery_step()){
            return (double)1E19;

        }

        count_eval_iter++;

        if (old_count_step<count_step){
            vect_speed_z.push_back(Globals::avg_speed.z);
            vect_speed_x.push_back(Globals::avg_speed.x);
            old_count_step=count_step;
        }

        double eval_result=0;
        cumul_time += SimGlobals::dt;




        //here I'll do the evaluation
        //first some var used by multiples eval
        std::vector<Joint*> vect_lower_body;
        conF->getController()->get_character()->getCharacterBottom(vect_lower_body);
        std::vector<Joint*> vect_joints;
        vect_joints= conF->getController()->get_character()->get_joints();


        //*
        //this version just sum the torques on the lower body
        if (Globals::evolution_type==0){
            double eval_buff_torque = 0;
            for (int i = 0; i < (int)vect_lower_body.size(); ++i){
                eval_buff_torque += conF->getController()->getTorque(vect_lower_body[i]->idx()).length();
            }
            eval_result += 3 * eval_buff_torque / (1.2*1E6);
            //*/
        }

        //this one is the sum of the torques on the whole body
        if (Globals::evolution_type==0){

            //end of one step or end of the evaluation
            if (cumul_time >SimGlobals::evaluation_length ){
                //ponderation (le *1E? sert à normalizer)
                //dividing by the number of evaluation iteration will allow the use of the same weight on all configuration fo timestep
                sum_eval_whole_body_torques *= 1.0 * (0.5 * 1E-2) / count_eval_iter;

                //*
                if (!Globals::close_after_evaluation){
                    std::ostringstream oss;
                    oss<<"sum_torques: "<<sum_eval_whole_body_torques;
                    std::cout<<oss.str();
                }
                //*/
                eval_result += sum_eval_whole_body_torques ;
                sum_eval_whole_body_torques=0;
            }

            for (int i = 0; i < conF->getController()->get_character()->getJointCount(); ++i){
                sum_eval_whole_body_torques += conF->getController()->getTorque(i).length();
            }
        }


        //*
        //this version just sum the drag torque
        if (Globals::evolution_type==0){

            //end of one step or end of the evaluation
            if (cumul_time >SimGlobals::evaluation_length ){
                //ponderation (le *1E? sert à normalizer)
                sum_eval_liquid_drag *= 6 *1E4 / count_eval_iter;

                //*
                if (!Globals::close_after_evaluation){
                    std::ostringstream oss;
                    oss<<"sum_torques: "<<sum_eval_liquid_drag ;
                    std::cout<<oss.str();
                }
                //*/
                eval_result += sum_eval_liquid_drag  ;
                sum_eval_liquid_drag =0;
            }

            double eval_buff_drag = 0;
            throw("this need to be repaired because the water impact structure has been modified");
            ///TODO adapt that loop for the new struture
            /*
            for (auto it = conF->resulting_impact.begin(); it != conF->resulting_impact.end(); ++it){
                WaterImpact impact = it->second;
                eval_buff_drag += impact.drag_torque.length();
            }
            //*/
            sum_eval_liquid_drag  += eval_buff_drag ;
        }
        //*/



        //*


        //this version will minimize the weighted angular acc on both the target positions and the effective result
        //for the lower body !!WARNING THE CURRETN VERSION DOES IT FOR THE WHOLE BODY
        //*
        if (Globals::evolution_type==0){
            ArticulatedRigidBody* char_root=conF->getController()->get_character()->getRoot();

            if (last_phi>phi || cumul_time >SimGlobals::evaluation_length ){
                sum_eval_weighted_acc += cur_step_eval_weighted_acc;
                cur_step_eval_weighted_acc = 0;
                //vect_ang_speed_weighted_acc.clear();
                vect_ang_speed_desired_pose_weighted_acc.clear();
                first_pass_weighted_acc = true;
            }

            if (cumul_time >SimGlobals::evaluation_length ){//(4*1E1);
                //ponderation (le *1E? sert à normalizer)
                //here we use the square of the count since this function use the square of the torques
                sum_eval_weighted_acc *= 1.0 * (1E7) / (count_eval_iter*count_eval_iter);

                //*
                if (!Globals::close_after_evaluation){
                    std::ostringstream oss;
                    oss<<"sum_acc_all: "<<sum_eval_weighted_acc;
                    std::cout<<oss.str();
                }
                //*/
                eval_result += sum_eval_weighted_acc;
                sum_eval_weighted_acc=0;

            }


            if (first_pass_weighted_acc){

                //we init te vector for the desired pos speed
                //we create the interface to modify the target pose
                std::vector<double> desir_pos_buffer;
                conF->getController()->getDesiredPose(desir_pos_buffer);;
                ReducedCharacterState poseRS(&desir_pos_buffer);



                if (vect_ang_speed_weighted_acc.empty()){
                    for (int i = 0; i < (int)vect_joints.size(); ++i){
                        //we init the vector for the actual speeds
                        vect_ang_speed_weighted_acc.push_back(vect_joints[i]->child()->getAngularVelocity());
                    }

                    //we add the root
                    vect_ang_speed_weighted_acc.push_back(char_root->getAngularVelocity());
                }

                for (int i = 0; i < (int)vect_joints.size(); ++i){
                    //we init the vector for the actual speeds
                    vect_ang_speed_desired_pose_weighted_acc.push_back(poseRS.getJointRelativeAngVelocity(vect_joints[i]->idx()));
                }
                //we add the root
                vect_ang_speed_desired_pose_weighted_acc.push_back(poseRS.getAngularVelocity());

            }
            else{
                std::vector<double> desir_pos_buffer;
                conF->getController()->getDesiredPose(desir_pos_buffer);;
                ReducedCharacterState poseRS(&desir_pos_buffer);

                for (int i = 0; i < (int)vect_joints.size(); ++i){


                    //we calc the variation and ponderate by the child mass
                    Vector3d new_val = vect_joints[i]->child()->getAngularVelocity();
                    double d_acc = (new_val - vect_ang_speed_weighted_acc[i]).length() / SimGlobals::dt;
                    cur_step_eval_weighted_acc += d_acc*d_acc*vect_joints[i]->child()->getMass() / 10E10 *0.8;
                    vect_ang_speed_weighted_acc[i] = new_val;

                    //and do the same for the desired pos
                    new_val = poseRS.getJointRelativeAngVelocity(vect_joints[i]->idx());
                    d_acc = (new_val - vect_ang_speed_desired_pose_weighted_acc[i]).length() / SimGlobals::dt;
                    cur_step_eval_weighted_acc += d_acc*d_acc*vect_joints[i]->child()->getMass() / 10E10 *0.2;
                    vect_ang_speed_weighted_acc[i] = new_val;
                }

                //we add the comutation for the root
                //we calc the variation and ponderate by the child mass
                Vector3d new_val = char_root->getAngularVelocity();
                double d_acc = (new_val - vect_ang_speed_weighted_acc.back()).length() / SimGlobals::dt;
                cur_step_eval_weighted_acc += d_acc*d_acc*char_root->getMass() / 10E10 *0.8;
                vect_ang_speed_weighted_acc.back()= new_val;

                //and do the same for the desired pos
                new_val = poseRS.getAngularVelocity();
                d_acc = (new_val - vect_ang_speed_desired_pose_weighted_acc.back()).length() / SimGlobals::dt;
                cur_step_eval_weighted_acc += d_acc*d_acc*char_root->getMass() / 10E10 *0.2;
                vect_ang_speed_weighted_acc.back() = new_val;

            }
            first_pass_weighted_acc = false;
        }
        //*/




        //this one is to keep the head stable
        if (Globals::evolution_type==1){

            //end of one step or end of the evaluation
            if (cumul_time >SimGlobals::evaluation_length ){
                //ponderation (le *1E? sert à normalizer)
                sum_eval_stable_head *= 1.0 / count_eval_iter;


                //*
                if (!Globals::close_after_evaluation){
                    std::cout<<"sum eval head: "<<sum_eval_stable_head<<std::endl;
                }
                //*/
                eval_result += sum_eval_stable_head;
                sum_eval_stable_head=0;
            }else{


                //*
                //this version is based on the angle
                //I'll limit the angle on the x and z axis
                ArticulatedRigidBody* arb=conF->getController()->get_character()->getARBByName("head");
                Quaternion h=arb->getOrientation();
                Quaternion h_x=h.decomposeRotation(Vector3d(1,0,0));
                Quaternion h_z=h.decomposeRotation(Vector3d(0,0,1));

                double angle_x=h_x.getRotationAngle(Vector3d(1,0,0));
                double angle_z=h_z.getRotationAngle(Vector3d(0,0,1));


                //*
                //check if the angle x stay between the correct limits
                if (angle_x<-0.025||angle_x>0.025){
                    double error=std::abs(angle_x)-0.015;
                    sum_eval_stable_head+=std::min(1.0,std::pow(error/0.01,2))*1E8;

                    //                                        std::cout<<"angle x error: "<<angle_x<<std::endl;
                }
                //*/

                //*
                //and chekc the limit of the angle_z
                if (angle_z<-0.01||angle_z>0.01){
                    double error=std::abs(angle_z)-0.01;
                    sum_eval_stable_head+=std::min(1.0,std::pow(error/0.01,2))*1E8;
                    //                                        std::cout<<"angle z error: "<<angle_z<<std::endl;
                }
                //*/



                Vector3d head_relative_pos=arb->getCMPosition()-conF->getController()->get_character()->getRoot()->getCMPosition();
                //check the limits of the position of the head relative to the pelvis

                //*
                //for x
                if (head_relative_pos.x<-0.01||head_relative_pos.x>0.01){
                    double error=std::abs(head_relative_pos.x)-0.01;
                    sum_eval_stable_head+=std::min(1.0,std::pow(error/0.01,2))*1E8;
                    //                                            std::cout<<"x error: "<<head_relative_pos.x<<std::endl;
                }

                //*/

                //*
                //for z
                if (head_relative_pos.z<0.01||head_relative_pos.z>0.05){
                    double error=std::min(std::abs(head_relative_pos.z-0.01),std::abs(head_relative_pos.z-0.05));
                    sum_eval_stable_head+=std::min(1.0,std::pow(error/0.01,2))*1E8;
                    //                                        std::cout<<"z error: "<<head_relative_pos.z<<std::endl;
                }
                //*/

                // this shoudl stay between 1.660 and 1.695
                //*
                double head_height=arb->getCMPosition().y;
                if (head_height<1.660||head_height>1.695){
                    double error=std::min(std::abs(head_height-1.660),std::abs(head_height-1.695));
                    sum_eval_stable_head+=std::min(1.0,std::pow(error/0.02,2))*1E8;
                    //                                        std::cout<<"y error: "<<head_height<<std::endl;
                }
                //*/

                /*
                {
                    //Vector3d head_relative_pos=arb->getCMPosition()-conF->getController()->get_character()->getRoot()->getCMPosition();
                    std::ofstream ofs1 ("eval_estimate/head_pos.csv", std::ofstream::app);
                    ofs1<<count_eval_iter<<";"<<head_relative_pos.x<<";"<<head_height<<";"<<head_relative_pos.z;
                    ofs1<<std::endl;
                    ofs1.close();

                    std::ofstream ofs ("eval_estimate/head_angle.csv", std::ofstream::app);
                    ofs<<count_eval_iter<<";"<<angle_x<<";"<<angle_z;
                    ofs<<std::endl;
                    ofs.close();

                }
                //*/

            }
        }


        //this one is to keep the hands stable
        if (Globals::evolution_type==1){

            //end of one step or end of the evaluation
            if (cumul_time >SimGlobals::evaluation_length ){
                //ponderation (le *1E? sert à normalizer)
                sum_eval_stable_hands *= 1.0 / count_eval_iter;


                //*
                if (!Globals::close_after_evaluation){
                    std::cout<<"sum eval hands: "<<sum_eval_stable_hands<<std::endl;
                }
                //*/
                eval_result += sum_eval_stable_hands;
                sum_eval_stable_hands=0;
            }else{


                Point3d target_pt=conF->getController()->get_character()->getRoot()->getCMPosition();
                target_pt.z+=0.2;
                target_pt.y=1.4;
                double coronal_offset=0.15;
                //*




                //left
                {
                    ArticulatedRigidBody* arb=conF->getController()->get_character()->getARBByName("lLowerarm");
                    Point3d hand_pos=arb->getCMPosition()+arb->getWorldCoordinates(Vector3d(0.15,0,0));
                    Point3d left_target_pt=target_pt;
                    left_target_pt.x+=coronal_offset;
                    Vector3d pos_error=left_target_pt-hand_pos;
                    //we reduce the error to make the system more laxinst since the hand positions
                    //are not that critials
                    //                    pos_error=pos_error*0.7;


                    //check the limits of the position of the head relative to the pelvis

                    /*
                    //for x //0.04
                    if (pos_error.x<-0.1||pos_error.x>0.1){
                        sum_eval_stable_hands+=1E8;
                        //                        std::cout<<"left x error: "<<pos_error.x<<std::endl;
                    }

                    //for y //0.015
                    if (pos_error.y<-0.1||pos_error.y>0.1){
                        sum_eval_stable_hands+=1E8;
                        //                         std::cout<<"left y error: "<<pos_error.y<<std::endl;
                    }
                    //for z //0.04
                    if (pos_error.z<-0.1||pos_error.z>0.1){
                        sum_eval_stable_hands+=1E8;
                        //                         std::cout<<"left z error: "<<pos_error.z<<std::endl;
                    }
                    //*/

                    //do the error penalization on the length
                    if (pos_error.length()>0.04){
                        double error=pos_error.length()-0.04;
                        sum_eval_stable_hands+=std::min(1.0,std::pow(error/0.02,2))*1E8;
                    }

                    /*
                    {
                        std::ofstream ofs ("eval_estimate/hand_position_left.csv", std::ofstream::app);
                        ofs<<count_eval_iter<<";"<<pos_error.x<<";"<<pos_error.y<<";"<<pos_error.z<<";"<<
                             count_eval_iter<<";"<<pos_error.length();
                        ofs<<std::endl;
                        ofs.close();
                    }
                    //*/
                }

                {
                    ArticulatedRigidBody* arb=conF->getController()->get_character()->getARBByName("rLowerarm");
                    Point3d hand_pos=arb->getCMPosition()+arb->getWorldCoordinates(Vector3d(-0.15,0,0));
                    Point3d right_target_pt=target_pt;
                    right_target_pt.x+=-coronal_offset;
                    Vector3d pos_error=right_target_pt-hand_pos;
                    //check the limits of the position of the head relative to the pelvis

                    /*
                    //for x
                    if (pos_error.x<-0.1||pos_error.x>0.1){
                        sum_eval_stable_hands+=1E8;
                        //                        std::cout<<"right x error: "<<pos_error.x<<std::endl;
                    }

                    //for y
                    if (pos_error.y<-0.1||pos_error.y>0.1){
                        sum_eval_stable_hands+=1E8;
                        //                        std::cout<<"right y error: "<<pos_error.y<<std::endl;
                    }
                    //for z
                    if (pos_error.z<-0.1||pos_error.z>0.1){
                        sum_eval_stable_hands+=1E8;
                        //                         std::cout<<"right z error: "<<pos_error.z<<std::endl;
                    }
                    //*/

                    //do the error penalization on the length
                    if (pos_error.length()>0.04){
                        sum_eval_stable_hands+=1E8;
                    }

                    /*
                    {
                        std::ofstream ofs ("eval_estimate/hand_position_right.csv", std::ofstream::app);
                        ofs<<count_eval_iter<<";"<<pos_error.x<<";"<<pos_error.y<<";"<<pos_error.z<<";"<<
                             count_eval_iter<<";"<<pos_error.length();
                        ofs<<std::endl;
                        ofs.close();
                    }
                    //*/
                }

            }
        }








        //look that the pelvis stay straight
        if (Globals::evolution_type==-1){

            //end of the evaluation
            if ( cumul_time >SimGlobals::evaluation_length ){
                //ponderation (le *1E? sert à normalizer)
                sum_eval_stable_pelvis *=  10 *  (1.0 * 1E1) / count_eval_iter;

                //*
                if (!Globals::close_after_evaluation){
                    std::cout<<"sum_angle_pelvis: "<<sum_eval_stable_pelvis<<std::endl;
                }
                //*/

                eval_result += sum_eval_stable_pelvis;
                sum_eval_stable_pelvis=0;
            }else{



                ArticulatedRigidBody* arb=conF->getController()->get_character()->getRoot();
                Quaternion h=arb->getOrientation();
                Quaternion h_x=h.decomposeRotation(Vector3d(1,0,0));
                Quaternion h_z=h.decomposeRotation(Vector3d(0,0,1));

                double angle_x=h_x.getRotationAngle(Vector3d(1,0,0));
                double angle_z=h_z.getRotationAngle(Vector3d(0,0,1));

                sum_eval_stable_pelvis+=std::abs(angle_x);
                sum_eval_stable_pelvis+=std::abs(angle_z);

                Vector3d vel=arb->getCMVelocity();
                Vector3d acc;
                if (old_vel_pelvis!=Vector3d(0,0,0)){
                    acc=vel-old_vel_pelvis;
                    //                sum_eval_stable_pelvis+=acc.length()*1;
                }
                old_vel_pelvis=vel;

                Vector3d ang_vel=arb->getAngularVelocity();
                Vector3d ang_acc;
                if (old_angular_vel_pelvis!=Vector3d(0,0,0)){
                    ang_acc=ang_vel-old_angular_vel_pelvis;
                    sum_eval_stable_pelvis+=ang_acc.length()*0.5;
                }
                old_angular_vel_pelvis=ang_vel;



                /*
            std::ostringstream oss4;
            oss4<<"angle x: "<< angle_x<< "   angle z: "<< angle_z<< "  acc.length()"<<acc.length()<<" ang_acc.length()"<<ang_acc.length();
            std::cout<<oss4.str();

            //*/

            }
        }



        //we stop if we have enougth
        if (cumul_time >SimGlobals::evaluation_length ){



            //we penalise this simulation if the speed ain't correct
            //the accepted error is 5%

            double accepted_error = std::fmax(std::abs(SimGlobals::velDSagittal / 20.0), 0.02);


            for (int i=0; i<(int)vect_speed_z.size()-1;++i){
                double z_speed = (vect_speed_z[i]+vect_speed_z[i+1])/2.0;
                if (std::abs(z_speed - SimGlobals::velDSagittal) > accepted_error){
                    eval_result += (double)1E10*std::min(1.0,std::pow((std::abs(z_speed - SimGlobals::velDSagittal) - accepted_error)/accepted_error,4));
                    //                    std::cout<<"vel error z: "<<z_speed- SimGlobals::velDSagittal<<std::endl;
                }
            }
            vect_speed_z.clear();

            //we do the same for the x axis
            accepted_error = std::fmax(std::abs(SimGlobals::velDCoronal / 20.0), 0.02);


            //std::cout<<vect_speed_x.size()<<" "<<accepted_error<<std::endl;
            for (int i=0; i<(int)vect_speed_x.size()-1;++i){
                double x_speed = (vect_speed_x[i]+vect_speed_x[i+1])/2.0;
                if (std::abs(x_speed - SimGlobals::velDCoronal) > accepted_error){
                    eval_result += (double)1E10*std::min(1.0,std::pow((std::abs(x_speed - SimGlobals::velDCoronal) - accepted_error)/accepted_error,4));
                    //                                        std::cout<<"vel error x: "<<std::abs(x_speed - SimGlobals::velDCoronal)<<std::endl;
                }
            }
            vect_speed_x.clear();

            //I penalize the simulation that use phi>0.90 and <0.70 to have a movement that will be able to handle
            //interferences without having to phi motion that are on the limit of what we can control

            if (Globals::evolution_type==0){
                if (conF->getController()->phi_last_step > 0.95){
                    eval_result += (double)10E10*std::abs(conF->getController()->phi_last_step - 0.95);
                }
                if (conF->getController()->phi_last_step - 0.85){
                    eval_result += (double)10E10*std::abs(0.85 - conF->getController()->phi_last_step);
                }
            }




            //this one tries to minimise the Kp and Kd
            if (Globals::evolution_type==1){
                const std::vector<ControlParams>& cp=conF->getController()->get_control_params();
                const ControlParams& root_param=conF->getController()->get_root_control_params();
                double total_kp=0;
                double total_kd=0;

                total_kp=root_param.kp;
                total_kd=root_param.kd;

                for (int i=0;i<cp.size();++i){
                    total_kp+=cp[i].kp;
                }

                for (int i=0;i<cp.size();++i){
                    total_kd+=cp[i].kd;
                }

                double sum_eval=(total_kp+total_kd*10);

                if (Globals::look_for_min_gains){
                    //ponderation (le 0.f*1E? sert à normalizer)
                    sum_eval *= 5 *  (0.25 * 1E-3);
                }else{
                    sum_eval=Globals::max_gains_limit_sum-sum_eval;
                    //ponderation (le 0.f*1E? sert à normalizer)
                    sum_eval *= 5 *  (0.25 * 1E-3);
                }

                if (!Globals::use_normalized_sum){
                    eval_result+=sum_eval;
                }

                //*
                if (!Globals::close_after_evaluation)
                {
                    std::cout<<"sum_gains"<<sum_eval<<"      total_kp: "<<total_kp<<"  total_kd: "<<total_kd<<std::endl;
                }
                //*/



            }
            //*
            if (!Globals::close_after_evaluation){
                std::cout<<"eval_result before system penalisation: "<<eval_result<< "   eval_iter: "<<count_eval_iter<<std::endl;
            }
            //*/

            //*
            //this passage penalise the usage of the speed strategies
            if (Globals::evolution_type==0){
                eval_result*=(1+0.1*std::abs(conF->getController()->ipm_alt_sagittal()/0.09));
                eval_result*=(1+0.01*conF->getController()->velocity_controller->avg_virt_force().length());
            }

            //*/
            //*
            if (!Globals::close_after_evaluation){
                std::cout<<"eval_result after system penalisation: "<<eval_result<< "   eval_iter: "<<count_eval_iter<<std::endl;
            }
            //*/

            double return_value=eval_result;
            eval_result=0;
            cumul_time=0;
            count_step=0;

            saving_possible=true;


            return (double)return_value;
        }
    }

    last_phi=phi;

    //if we arrive here it means the eval is not finished
    return -1;
}

void ControllerEditor::reset_members_for_evaluation(){
    last_phi = 0;
    count_eval_iter=0;
    vect_speed_z.clear();
    vect_speed_x.clear();
    cumul_time = 0;
    sum_eval_whole_body_torques=0;
    sum_eval_liquid_drag =0;
    cur_step_eval_weighted_acc = 0;
    sum_eval_weighted_acc = 0;
    first_pass_weighted_acc = true;
    vect_ang_speed_weighted_acc.clear();
    vect_ang_speed_desired_pose_weighted_acc.clear();
    sum_eval_stable_head=0;
    sum_eval_stable_hands=0;
    sum_eval_stable_pelvis=0;
    eval_push_phase=0;
    old_vel_head=Vector3d(0,0,0);
    old_vel_pelvis=Vector3d(0,0,0);
    old_angular_vel_pelvis=Vector3d(0,0,0);
    old_count_step=0;
}
//*/












/*
int updateTargetPose(ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){
    ControllerEditor* obj = (ControllerEditor*)clientData;
    return TCL_OK;
}
*/

// Following are wrappers for TCL functions that can access the object
/*
int controllerUndo (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

    ControllerEditor* obj = (ControllerEditor*)clientData;
    obj->undo();
    return TCL_OK;
}
*/

// Following are wrappers for TCL functions that can access the object
/*
int controllerRedo (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

    ControllerEditor* obj = (ControllerEditor*)clientData;
    obj->redo();
    return TCL_OK;
}
*/

// Following are wrappers for TCL functions that can access the object
/*
int getStateNames (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

    ControllerEditor* obj = (ControllerEditor*)clientData;

    SimBiController* controller = obj->getFramework()->getController();

    DynamicArray<const char*> stateNames;
    uint i = 0;
    while( true ) {
        SimBiConState* state = controller->getState( i++ );
        if( !state ) break;
        stateNames.push_back( state->getDescription() );
    }

    char* result = stringListToTclList( stateNames );
    Tcl_SetResult(interp, result, TCL_DYNAMIC);

    return TCL_OK;
}
*/

/*
int getTrajectoryNames (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

    // getComponentNames stateIdx
    if( argc != 2 ) return TCL_ERROR;

    ControllerEditor* obj = (ControllerEditor*)clientData;

    int idx = atoi( argv[1] );
    SimBiConState* state = obj->getFramework()->getController()->getState( idx );

    if( !state ) return  TCL_ERROR;

    DynamicArray<const char*> trajectoryNames;
    for( int i = 0; i < state->getTrajectoryCount(); ++i ) {
        Trajectory* trajectory = state->getTrajectory( i );
        trajectoryNames.push_back( trajectory->jName );
    }

    char* result = stringListToTclList( trajectoryNames );
    Tcl_SetResult(interp, result, TCL_DYNAMIC);

    return TCL_OK;
}
*/

/*
int getComponentNames (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

    // getComponentNames stateIdx trajectoryIdx
    if( argc != 3 ) return TCL_ERROR;

    ControllerEditor* obj = (ControllerEditor*)clientData;

    int idx = atoi( argv[1] );
    SimBiConState* state = obj->getFramework()->getController()->getState( idx );
    if( !state ) return  TCL_ERROR;
    idx = atoi( argv[2] );
    Trajectory* trajectory = state->getTrajectory( idx );
    if( !trajectory ) return  TCL_ERROR;

    DynamicArray<const char*> componentNames;
    for( uint i = 0; i < trajectory->components.size(); ++i ) {
        char* componentName = new char[ 32 ];
        sprintf( componentName, "Component %d", i );
        componentNames.push_back( componentName );
    }

    char* result = stringListToTclList( componentNames );

    for( uint i = 0; i < componentNames.size(); ++i )
        delete[] componentNames[i];

    Tcl_SetResult(interp, result, TCL_DYNAMIC);
    return TCL_OK;
}
*/


/**
 * Registers TCL functions specific to this application
 */
/*
void ControllerEditor::registerTclFunctions() {	

    Application::registerTclFunctions();

    Tcl_CreateCommand(Globals::tclInterpreter, "getStateNames", getStateNames, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateCommand(Globals::tclInterpreter, "getTrajectoryNames", getTrajectoryNames, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateCommand(Globals::tclInterpreter, "getComponentNames", getComponentNames, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateCommand(Globals::tclInterpreter, "trajectoryToEdit", trajectoryToEdit, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateCommand(Globals::tclInterpreter, "updateTargetPose", updateTargetPose, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateCommand(Globals::tclInterpreter, "controllerUndo", controllerUndo, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateCommand(Globals::tclInterpreter, "controllerRedo", controllerRedo, (ClientData)this, (Tcl_CmdDeleteProc *) NULL);

}
*/

/**
 * This method is to be implemented by classes extending this one. The output of this function is a point that
 * represents the world-coordinate position of the dodge ball, when the position in the throw interface is (x, y).
 */
void ControllerEditor::getDodgeBallPosAndVel(double x, double y, double strength, Point3d* pos, Vector3d* vel){
    vel->x = x;
    vel->y = 0;
    vel->z = y;

    *vel = conF->getCharacter()->getHeadingHorizontal().rotate(*vel) * 20;
    *pos = conF->getCharacter()->getRoot()->getCMPosition();
    *pos = *pos + conF->getCharacter()->getRoot()->getCMVelocity() * 0.5;
    pos->y +=1;
    *pos = *pos + vel->unit() * (-2);


    /*
    std::ostringstream oss;
    oss<<"force applyed: "<<f.x<<"  "<<f.y<<"  "<<f.z;
    std::cout<<oss.str();
    //*/
}

void ControllerEditor::apply_perturbation_on_pelvis(Vector3d f)
{
    /*
    if (!Globals::evolution_mode){
        std::ostringstream oss;
        oss<<"applyed perturbation: "<<f.x<<" "<<f.y<<" "<<f.z;
        std::cout<<oss.str()<<std::endl;
    }
    //*/
    ODEWorld* world = dynamic_cast<ODEWorld*>(conF->pw);
    world->applyForceTo(conF->getCharacter()->getRoot(), f,Point3d(0,0,0));
}

void ControllerEditor::apply_perturbation(Vector3d perturbation, bool new_perturbation)
{
    if (new_perturbation){
        perturbation_force=perturbation;
        time_since_application=0.0;
    }else{
        if (time_since_application<0.3){
            time_since_application+=SimGlobals::dt;

            apply_perturbation_on_pelvis(perturbation_force);
        }
    }

}


