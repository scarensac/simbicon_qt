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

#include "SimGlobals.h"

//initialize all the parameters to some sensible values.

//give this a very high value so that we can use the scripted values in the rb specs for the value to use
double SimGlobals::gravity = -9.8;
Vector3d SimGlobals::up = Vector3d(0, 1, 0);
int SimGlobals::forceHeadingControl = 1;
double SimGlobals::desiredHeading = 0;
double SimGlobals::desiredHeading_active = 0;
double SimGlobals::dt = 1.0/(300.0);
AbstractRBEngine* SimGlobals::activeRbEngine = NULL;
AbstractRBEngine* SimGlobals::stanceFootWorld = NULL;

double SimGlobals::conInterpolationValue;
double SimGlobals::bipDesiredVelocity;


double SimGlobals::targetPos = 0;


double SimGlobals::targetPosX = 0;
double SimGlobals::targetPosZ = 0;

int SimGlobals::constraintSoftness = 1;
int SimGlobals::CGIterCount = 0;
int SimGlobals::linearizationCount = 1;


double SimGlobals::rootSagittal = 0;
double SimGlobals::rootLateral = 0;
double SimGlobals::swingHipSagittal = 0;
double SimGlobals::swingHipLateral = 0;
double SimGlobals::stanceAngleSagittal = 0;
double SimGlobals::stanceAngleLateral = 0;
double SimGlobals::stanceKnee = 0;

double SimGlobals::COMOffsetX = 0;
double SimGlobals::COMOffsetZ = 0;

double SimGlobals::time_factor = 1;

bool SimGlobals::force_ipm= false;

double SimGlobals::force_alpha = 0;
double SimGlobals::water_level = 0;
double SimGlobals::liquid_density = 1000;
double SimGlobals::liquid_viscosity = 1;
double SimGlobals::left_stance_factor = 0;
std::vector<ForceStruct> SimGlobals::vect_forces = std::vector<ForceStruct>();

//for the deplacement direction control
double SimGlobals::velDSagittal = 0.7;//0.95;
double SimGlobals::velDCoronal = 0;
double SimGlobals::velDSagittalOld = 0.7;//0.95;
double SimGlobals::velDCoronalOld = 0;


double SimGlobals::step_width = 0.1;



bool SimGlobals::is_evaluation = false;
int SimGlobals::steps_before_evaluation = 2;
int SimGlobals::evaluation_length= 1;


double SimGlobals::ipm_alteration_effectiveness=1;
double SimGlobals::virtual_force_effectiveness=1;

bool SimGlobals::foot_flat_on_ground=false;
bool SimGlobals::foot_flat_on_ground_current=false;

double SimGlobals::requested_state_duration=-1;
bool SimGlobals::state_duration_modified=false;


int SimGlobals::nb_container_boxes=1;
int SimGlobals::nb_filler=9;
int SimGlobals::nb_collisons_event=0;
int SimGlobals::nb_active_objects=0;



void SimGlobals::reset_sim_globals()
{
    SimGlobals::velDSagittal = 0.7;//0.95;
    SimGlobals::velDCoronal = 0;

    SimGlobals::desiredHeading = 0;
    SimGlobals::force_ipm= false;

    SimGlobals::force_alpha = 0;
    SimGlobals::water_level = 0.25;
    SimGlobals::liquid_density = 1000;
    SimGlobals::liquid_viscosity = 1;
    SimGlobals::left_stance_factor = 0;

    SimGlobals::step_width = 0.1;

    SimGlobals::foot_flat_on_ground=false;
    SimGlobals::stanceFootWorld=NULL;
}


