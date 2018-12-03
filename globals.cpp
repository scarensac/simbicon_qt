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

#include "Globals.h"

//initialize the static variables to some sensible values
Application* Globals::app = NULL;
GLWindow* Globals::window = NULL;
int Globals::glut_window_id = 0;
int Globals::animationRunning = 0;
double Globals::animationTimeToRealTimeRatio = 1;
double Globals::desiredFrameRate = 30;

//double Globals::dt = 1.0/(6000.0);
int Globals::drawFPS = 1;
int Globals::drawCubeMap = 0;
int Globals::drawGlobalAxes = 0;
int Globals::drawShadows = 1;
int Globals::drawCollisionPrimitives = 0;
int Globals::drawGroundPlane = 0;
int Globals::followCharacter = 1;
int Globals::drawJoints = 0;
int Globals::drawContactForces = 0;
int Globals::drawDesiredPose = 0;
double Globals::targetPosePhase = 0;
int Globals::drawPushInterface = 0;
int Globals::drawCurveEditor = 0;
int Globals::drawCanvas = 0;
int Globals::drawScreenShots = 0;
int Globals::drawWorldShots = 0;
int Globals::drawControlShots = 0;
int Globals::updateDVTraj = 0;
char Globals::currControlShotStr[200] = "";
bool Globals::useShader = true;
bool Globals::useConsole = true;
std::string Globals::init_folder_path = "init/";
std::string Globals::data_folder_path = "../configuration_data/";
std::string Globals::binaries_folder_path = "../Binaries/";
bool Globals::use_qt_interface = true;
bool Globals::use_gl_interface = true;
bool Globals::evolution_mode = false;
int Globals::evolution_type = 0;
int Globals::evolution_push_type = 0;
bool Globals::look_for_min_gains = true;
int Globals::max_gains_limit_sum= 25000;
bool Globals::use_normalized_sum= false;
bool Globals::close_after_evaluation = false;
double Globals::simulation_eval = -1;
double Globals::ipm_alteration_cost = 0;
double Globals::virtual_force_cost = 0;

bool Globals::save_mode = false;
bool Globals::save_to_current_controller=false;
std::string Globals::current_controller_file = "";
std::string Globals::current_starting_pos_file = "";
std::string Globals::primary_save_config = "";
std::string Globals::secondary_save_config = "";
std::string Globals::save_mode_controller = "";
int Globals::save_controller=true;
int Globals::save_position=true;
bool Globals::close_after_saving = false;
Vector3d Globals::avg_speed=Vector3d(0,0,0);


double Globals::a=0, Globals::b=1, Globals::c=0, Globals::d=0;

double Globals::ref_paper_eval=0, Globals::cur_paper_eval=0;

bool Globals::use_contact_controller=false;
int Globals::foot_contact_controller_config=0;
bool Globals::use_stance_leg_orientation_controller=true;
bool Globals::use_hand_position_tracking=true;
bool Globals::use_fluid_heading=true;
int Globals::motion_cycle_type=0;


bool Globals::simulateFluid = true;
bool Globals::zeroFluidVelocities = false;
bool Globals::simulateOnlyFluid = false;
bool Globals::fluidFollowCharacter = false;

Globals::Globals(void){
}

Globals::~Globals(void){

}


void Globals::changeCurrControlShotStr( int currControlShot ) {
    if( currControlShot < 0 )
		strcpy( currControlShotStr, "Initial" );
	else
		sprintf( currControlShotStr, "%05d", currControlShot );
}


//print in an openGL window. The raster position needs to be already defined.
void gprintf(const char *fmt, ...){
    char		text[256];								// Holds Our String
    va_list		ap;										// Pointer To List Of Arguments

	if (fmt == NULL)									// If There's No Text
		return;											// Do Nothing

    va_start(ap, fmt);									// Parses The String For Variables
        vsprintf(text, fmt, ap);						// And Converts Symbols To Actual Numbers
    va_end(ap);											// Results Are Stored In Text

	glDisable(GL_DEPTH_TEST);
    int len = (int) strlen(text);
    for (int i = 0; i < len; i++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
	glEnable(GL_DEPTH_TEST);
}



//print in an openGL window with a large font. The raster position needs to be already defined.
void glargeprintf(const char *fmt, ...){
    char		text[256];								// Holds Our String
    va_list		ap;										// Pointer To List Of Arguments

	if (fmt == NULL)									// If There's No Text
		return;											// Do Nothing

    va_start(ap, fmt);									// Parses The String For Variables
        vsprintf(text, fmt, ap);						// And Converts Symbols To Actual Numbers
    va_end(ap);											// Results Are Stored In Text

	glDisable(GL_DEPTH_TEST);
    int len = (int) strlen(text);
    for (int i = 0; i < len; i++)
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, text[i]);
	glEnable(GL_DEPTH_TEST);
}



#include<fstream>
#include<iostream>
void write_to_report_file(std::string text, bool assure_end_line){
    std::string file_name="velocity_evolution_low.csv";
    static bool first_time=true;
    if (first_time){
        first_time=false;
        std::remove(file_name.c_str());
    }
    std::ofstream myfile;
    myfile.open(file_name,std::ios::app);
    myfile << text ;
    if (assure_end_line&&text.find('\n')== std::string::npos){
       myfile << std::endl ;
    }
    myfile.close();
}


