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

#include <GL/glew.h>
#include <stdio.h>
#include <string.h>
#include "UI.h"
#include "GLWindow.h"
#include "Application.h"
#include "ControllerEditor.h"
#include "Globals.h"
#include <GL/freeglut.h>
#include <chrono>
#include "SPlisHSPlasH/Interface.h"


/**
 * This method is used to initialize the main window.
 */
void InitMainWindow(int argc, const char **argv){
    int width = 1066;
    int height = 600;
    if (argc>3){
        sscanf(argv[2], "%d", &width);
        sscanf(argv[3], "%d", &height);
    }

    //dummies parameters
    char *myargv [1];
    int myargc=1;
    myargv [0]=strdup ("Myappname");
    //Initialize glut
    glutInit(&myargc,myargv);

    //create the GLUT window...
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_STENCIL);

    if (!Globals::use_gl_interface){
        glutInitWindowPosition(5000,5000);
    }else{
        glutInitWindowPosition(50,50);
    }
    glutInitWindowSize(width, height);
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    Globals::glut_window_id=glutCreateWindow("Simbicon 1.5 Controller Editing Framework");

    if (!Globals::use_gl_interface){
        //do not use the gl window for faster exe
        glutHideWindow();
    }

    //set up the callback methods
    glutDisplayFunc(appRender);
    glutIdleFunc(appProcess);
    glutMouseFunc(processMouse);
    glutMouseWheelFunc(process_mouse_wheel);
    glutMotionFunc(processMouseActiveMotion);
    glutPassiveMotionFunc(processPassiveMouseActiveMotion);
    glutKeyboardFunc(processKeyboard);

    //and an openGL window that we will use for rendering
    Globals::window = new GLWindow(50, 50, width, height);
    Globals::window->init();


    //also create an application that will be used to run the whole thing.
    //FOr the time being let's say e will only use the controller editor (which we do anyway...)
    //if (strcmp(argv[1], "ControllerEditor") == 0){

    //}else{
    //	Globals::app = new Application();
    //}


    //iniialize glew
    // Initialize GLEW
    GLenum err = glewInit();

    if (GLEW_OK != err)
    {
        std::cerr << "Error: " << glewGetErrorString(err) << std::endl;
        exit(EXIT_FAILURE);
    }


}

/**
    This is the callback method for mouse events
*/
void processMouse(int button, int state, int x, int y){
    int mButton = MOUSE_LBUTTON;
    int mEvent = MOUSE_UP;
    if (button == GLUT_RIGHT_BUTTON) mButton = MOUSE_RBUTTON;
    if (button == GLUT_MIDDLE_BUTTON) mButton = MOUSE_MBUTTON;


    if (state == GLUT_DOWN) mEvent = MOUSE_DOWN;
    if (Globals::window)
        Globals::window->onMouseEvent(mEvent, mButton, x, y);
}

/**
    This is the callback method for mouse wheel events
*/
void process_mouse_wheel( int wheel, int direction, int x, int y ){
    int mButton = MOUSE_LBUTTON;
    int mEvent = MOUSE_UP;
    if (direction == 1) mButton = MOUSE_WHEEL_DOWN;
    if (direction == -1) mButton = MOUSE_WHEEL_UP;
    if (Globals::window)
        Globals::window->onMouseEvent(mEvent, mButton, x, y);
}

/**
    This is the callback method for mouse (drag) events
*/
void processMouseActiveMotion(int x, int y){
    if (Globals::window)
        Globals::window->onMouseEvent(MOUSE_DRAG, -1, x, y);
}


/**
    This is the callback method for mouse (drag) events
*/
void processPassiveMouseActiveMotion(int x, int y){
    if (Globals::window)
        Globals::window->onMouseEvent(MOUSE_MOVE, -1, x, y);
}

void processKeyboard(unsigned char key, int x, int y){
    if (Globals::window)
        Globals::window->onKeyEvent((int)key);
}

/**
    This method gets called when the GL window should be redisplayed.
*/
void appRender(void){
    if (Globals::window)
        Globals::window->draw();
    glutSwapBuffers();
}

/**
 *	This method is responsible with initiating the work that needs to be done before one new frame is drawn.
 */
void appProcess(void){
    //we will check the tk/tcl messages as well...
    //CheckTCLTKEvents();

    if (!Globals::window || !Globals::app)
        return;

    Globals::window->onStartProcessing();

    std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();

    if (Globals::simulateFluid){
        if (Globals::zeroFluidVelocities){
            Interface::zeroFluidVelocities();
            Globals::zeroFluidVelocities=false;
        }
    }

    //we will only process the task if the animation should be running
    if (Globals::animationRunning)
        (Globals::app)->processTask();

    std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff=end-start;
    Globals::window->timeSpentProcessing2+=diff.count();

    Globals::window->onStopProcessing();

    if (Globals::followCharacter)
        Globals::window->setCameraTarget(Globals::app->getCameraTarget());

    //check the tk/tcl messages again
    //CheckTCLTKEvents();


    //force a redraw now...
    glutPostRedisplay();
}


/**
 * terminate the application.
 */
/*
int quit (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){
    exit(0);
    return TCL_OK;
}
*/


/**
  * setup the camera orientation
  */
//*
void camera (int orientation){

    if (Globals::window){
        switch ( orientation )
        {
            case 0:
                //back
                Globals::window->setCameraRotation(Vector3d(0,PI,0));
                break;
            case 1:
                //front
                Globals::window->setCameraRotation(Vector3d(-0.4,0,0));
                break;
            case 2:
                //side
                //Globals::window->setCameraRotation(Vector3d(0,-PI/2,0.19));
                                Globals::window->setCameraRotation(Vector3d(-0.4,-0.6,0));
                //                Globals::window->setCameraRotation(Vector3d(0,-PI/2,0));
                break;
            case 3:
                //45
//                Globals::window->setCameraRotation(Vector3d(-0.36,-PI/4,0));
                Globals::window->setCameraRotation(Vector3d(-0.36,-0.785,0));
                break;
            case 4:
                //n45
//                Globals::window->setCameraRotation(Vector3d(-0.36,-3*PI/4,0));
                Globals::window->setCameraRotation(Vector3d(-0.3,-0.8,0));
                break;
        }
    }
}
//*/

///////////////////////////////////////////////////////////
// Start|Stop the animation
///////////////////////////////////////////////////////////
/*
int animation (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){
    if (argc < 2) {
        Tcl_AppendResult(interp, "Usage: anim start|stop|step|restart");
        return TCL_OK;
    }

    if (strcmp(argv[1], "start") == 0){
        if (Globals::animationRunning == 0){
            std::cerr<<"Starting animation!\n");
        }
        Globals::animationRunning = 1;
    }
    else if(strcmp(argv[1], "pause") == 0){
        if (Globals::animationRunning == 1){
            std::cerr<<"Animation paused!\n");
        }
        Globals::animationRunning = 0;
    }
    else if(strcmp(argv[1],"step")==0){
        if (Globals::animationRunning == 0){
            std::cerr<<"Advancing the animation!\n");
            if (Globals::app)
                Globals::app->processTask();
        }
    }
    else if (strcmp(argv[1], "restart") == 0){
        std::cerr<<"Restarting the animation!\n");
        if (Globals::app)
            Globals::app->restart();
    }
    else if (strcmp(argv[1], "reload") == 0){
        std::cerr<<"Reloading the animation!\n");
        if (Globals::app)
            Globals::app->reload();
    }

    return TCL_OK;
}

*/

/**
 * This method is used to launch the animation
 */
/*
int launch (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){
    if (argc < 2) {
        std::cerr<<"Usage: launch Simbicon3D | mocap");
        return TCL_OK;
    }
    InitMainWindow(argc, argv);

    return TCL_OK;
}
*/

/**
* This method is used to launch the animation
*/
int core_init(){
    InitMainWindow();
    Globals::app = new ControllerEditor();
    Globals::app->init();
    return 0;
}

void core_close()
{
    Globals::animationRunning=false;
    delete Globals::app;
    Globals::app=NULL;

    if (Globals::window!=NULL){
        delete Globals::window;
        Globals::window=NULL;
    }

}

void launch_gl(){
    glutMainLoop();
}

void stop_gl()
{
    //glutDestroyWindow(Globals::glut_window_id);
    glutLeaveMainLoop();
}

/**
  * setup the camera orientation
  */
/*
int instantChar (ClientData clientData, Tcl_Interp *interp, int argc, CONST84 char **argv){

    if (argc < 1) {
        Tcl_AppendResult(interp, "Usage: instantChar");
        return TCL_OK;
    }

    return TCL_OK;
}
*/



void init_evaluate_solution( bool save, bool close_after_evaluation)
{
    Globals::evolution_mode = true;
    Globals::animationRunning = 1;
    Globals::save_mode=false;
    Globals::close_after_saving=false;
    SimGlobals::steps_before_evaluation = 15;
    SimGlobals::evaluation_length = 5;
    SimGlobals::liquid_density = 1000;
    SimGlobals::force_alpha = 0;
    Globals::use_contact_controller=false;

    /*
    for (int i = 2; i < argc; ++i){
        std::string  cur_arg= ws2s(std::wstring(argv[i]));
        if (cur_arg == "save"){

            SimGlobals::steps_before_evaluation = 50;
            Globals::save_mode = true;
            Globals::primary_save_config = "controllers/bipV2/primary_save_config.txt";
            Globals::secondary_save_config = "controllers/bipV2/learning_files_names.txt";
            if (argc >= i){
                std::string  save_controller = ws2s(std::wstring(argv[i+1]));
                Globals::save_mode_controller = save_controller;
            }
            break;
        }
    }*/



    if (save){
        //I don't want the secondary save in that program (just needed to fill that file for later use)
        //so I disactivate the secondary save
        Globals::save_mode=true;
        Globals::close_after_saving=true;
    }

    if (close_after_evaluation){
        Globals::close_after_evaluation=true;
        Globals::use_qt_interface=false;
        Globals::use_gl_interface=false;
    }


    Globals::ipm_alteration_cost = 0.1;

    //init the system
    core_init();
}

double evaluate_solution()
{
    if (Globals::app==NULL){
        exit(6598);
    }


    //start the simulation
    launch_gl();


    //the only way to reach this place it that the simulation has been stoped
    //So I need to delete the structures
    core_close();

    if (Globals::simulation_eval<0){
        //std::cout<<Globals::simulation_eval;
        //std::string test;
        //std::cin>>test;
        //we crashed ...
        exit(-1);
    }


    double result=Globals::simulation_eval;
    Globals::simulation_eval=-1;
    return result;
}

#include <fstream>
#include <sstream>
void update_saving_config(std::string evo_folder, std::string primary_config, std::string secondary_config)
{
    std::ostringstream oss;
    oss.clear();
    oss.str("");
    oss << Globals::data_folder_path;
    oss << primary_config;

    std::ofstream myfile1(oss.str());
    if (myfile1.is_open())
    {
        myfile1 << evo_folder << "/learning_walk_waterlvl" << SimGlobals::water_level << "_state.rs" << std::endl;
        myfile1 << evo_folder << "/learning_walk_waterlvl" << SimGlobals::water_level << ".sbc" << std::endl;
    }
    myfile1.close();

    //*
    std::stringstream oss2;
    oss2 << Globals::data_folder_path;
    oss2 << secondary_config;
    std::ofstream myfile2(oss2.str());
    if (myfile2.is_open())
    {
        myfile2 << "learning_walk_state.rs" << std::endl;
        myfile2 << "learning_walk.sbc" << std::endl;
    }
    myfile2.close();
}



