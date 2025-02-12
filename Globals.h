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

#include "gui\Application.h"
#include "gui\GLWindow.h"
#include <stdlib.h>
#include <GL/freeglut.h>
#include <string>
#include "MathLib/Vector3d.h"
#include "Core/ForcesUtilitary.h"

//disable all the 'deprecated function' warnings
#pragma warning( disable : 4996)

/**
        This class is used as a container for all the global variables that are needed in the system.
*/
class Globals{
public:
    //a reference to the application that is running
    static Application* app;
    //a reference to the openGL window
    static GLWindow* window;
    //id of glut windw to destroy it
    static int glut_window_id ;
    //indicates whether or not the animation (i.e. simulation, play back, etc) is playing
    static int animationRunning;
    //gives the ratio of animation time to real time.
    static double animationTimeToRealTimeRatio;
    //this is the desired frame rate
    static double desiredFrameRate;
    //flag that controls the drawing of the FPS information
    static int drawFPS;
    //flag that controls the drawing of the cubeMap
    static int drawCubeMap;
    //flag that controls the drawing of the golbal axes
    static int drawGlobalAxes;
    //flag that controls the drawing of the shadows
    static int drawShadows;
    //flag that controls the drawing of the collision primitives
    static int drawCollisionPrimitives;
    //flag that controls the drawing of the ground
    static int drawGroundPlane;
    //flag that controls the drawing of the contact forces
    static int drawContactForces;
    //flag that controls the drawing of the desired pose that is being tracked
    static int drawDesiredPose;
    //controls the phase at which the target pose is drawn
    static double targetPosePhase;
    //flag that controls the drawing of the push interface
    static int drawPushInterface;
    //flag that controls the drawing of the curve editor
    static int drawCurveEditor;
    //flag that controls the drawing of the canvas
    static int drawCanvas;
    //flag that controls the camera tracking
    static int followCharacter;
    //flag that controls the joint position display
    static int drawJoints;
    //flag that controls the drawing of screenshots
    static int drawScreenShots;
    //flag that controls the capturing of the 3D world in OBJ files
    static int drawWorldShots;
    //flag that controls the capturing of the controller
    static int drawControlShots;
    //a text variable to display the current control shot displayed
    static char currControlShotStr[200];
    //flat that indicates if the controller D and V trajectories should be updated on next step
    static int updateDVTraj;
    //indicates wether or not to use shaders
    static bool useShader;
    //indicates wether or not to use the console
    static bool useConsole;
    //this string will contain the path to the init folder
    static std::string init_folder_path;
    //this string will contain the path to the data folder
    static std::string data_folder_path;
    //this string will contain the path to the binaries folder
    static std::string binaries_folder_path;

    //those booleans are used to activate or disactivate the tk and gl interfaces (for evolution)
    static bool use_qt_interface;
    static bool use_gl_interface;
    static bool limitfps;

    //this bollean specify that we are in the evolution mode
    static bool evolution_mode;
    //this var indicate the type of evolution 0 the position one, 1 the gain one
    static int evolution_type;
    static int evolution_push_type;
    static bool look_for_min_gains;
    static int max_gains_limit_sum;
    static bool use_normalized_sum;
    static bool use_hand_position_tracking;
    static double simulation_eval;
    //those variable contain the cost of the usage of each speed control strategy
    static double ipm_alteration_cost;
    static double virtual_force_cost;


    //and this one specify if we want a save at the end of the evolution
    //the name is here to indicate a second config to save the current simulation
    static bool save_mode;
    static bool save_to_current_controller;
    static std::string current_controller_file;
    static std::string current_starting_pos_file;
    //file to read the save config if they are not the current config
    static std::string primary_save_config;
    static std::string secondary_save_config;
    //a parameter to tell what we want to save
    static int save_controller;
    static int save_position;
    //this variable is here to let the user specifya controller that overide the input.conf
    static std::string save_mode_controller;
    //this bool is here to tell the system  to close the program after saving the state
    static bool close_after_saving;

    //this bool is here to tell the system  to close the program after finishing the evloution
    static bool close_after_evaluation;

    //those are just some var to do the communication to show the speed on the gl window
    static Vector3d avg_speed;

    //these params define the ground plane - for drawing purposes only
    static double a, b, c, d;

    //just some value to realise the paper (ignore them)
    static double ref_paper_eval, cur_paper_eval;


    static bool use_contact_controller;
    static int foot_contact_controller_config;
    static bool use_stance_leg_orientation_controller;

    static int motion_cycle_type;
    static bool use_fluid_heading;


    static bool estimatedFluidDirectSampleApplication;
    static bool estimatedFluidDrawDrag;
    static bool simulateFluid;
    static bool zeroFluidVelocities;
    static bool simulateOnlyFluid;
    static bool fluidFollowCharacter;
    static bool fluidControlLevel;
    static bool parralelFluidAndControl;


    static bool manualFPSRefresh;
    static bool refreshFPS;

    static int video_index;

    static bool gamepad;

    static bool use_motion_combiner;


    //I'll use the contact point structure but all I want is the position and the force
    static std::vector<ForceStruct> vect_forces;
    static std::vector<ForceStruct> vect_forces_estimated_fluid;

    Globals(void);
    ~Globals(void);

    static void changeCurrControlShotStr( int currControlShot );


    static void reset();
};


//print in an openGL window. The raster position needs to be already defined.
void gprintf(const char *format, ...);

//print in an openGL window with a large font. The raster position needs to be already defined.
void glargeprintf(const char *format, ...);



//declare some useful constants and data types here as well
#define STRLEN 200
#define MAX_LINE 255
typedef char STR[STRLEN];


#define MOUSE_LBUTTON 1
#define MOUSE_RBUTTON 2
#define MOUSE_MBUTTON 3
#define MOUSE_WHEEL_DOWN 4
#define MOUSE_WHEEL_UP 5

#define MOUSE_DOWN 1
#define MOUSE_DRAG 2
#define MOUSE_UP 3
#define MOUSE_MOVE 4


void write_to_report_file(std::string text, bool assure_end_line=true);


