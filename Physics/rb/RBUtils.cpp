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

#include <Physics/rb/RBUtils.h>
#include <string.h>

typedef struct key_word{
	char keyWord[25];
	int retVal;
}KeyWord;

/**
	This method is used to determine the type of a line that was used in the input file for a rigid body.
	It is assumed that there are no white spaces at the beginning of the string that is passed in. the pointer buffer
	will be updated to point at the first character after the keyword.
*/
int getRBLineType(char* &buffer){
	KeyWord keywords[] = {
		{"RigidBody", RB_RB},
		{"/End", RB_END_RB},
		{"A_RigidBody", RB_ARB},
		{"mesh", RB_MESH_NAME},
		{"mass", RB_MASS},
		{"moi", RB_MOI},
		{"colour", RB_COLOUR},
		{"root", RB_ROOT},
		{"ArticulatedFigure", RB_ARTICULATED_FIGURE},
		{"child", RB_CHILD},
		{"parent", RB_PARENT},
		{"/ArticulatedFigure", RB_END_ARTICULATED_FIGURE},
		{"name", RB_NAME},
		{"Joint", RB_JOINT},
		{"/Joint", RB_END_JOINT},
		{"jointPPos", RB_PPOS},
		{"jointCPos", RB_CPOS},
		{"CDP_Sphere", RB_SPHERE},
		{"CDP_Capsule", RB_CAPSULE},
		{"CDP_Plane", RB_PLANE},
		{"static", RB_LOCKED},
		{"position", RB_POSITION},
		{"orientation", RB_ORIENTATION},
		{"velocity", RB_VELOCITY},
		{"angularVelocity", RB_ANGULAR_VELOCITY},
		{"frictionCoefficient", RB_FRICTION_COEFF},
		{"restitutionCoefficient", RB_RESTITUTION_COEFF},
		{"minBoundingSphere", RB_MIN_BDG_SPHERE},
		{"hingeJoint" ,RB_JOINT_TYPE_HINGE},
		{"jointLimits", RB_JOINT_LIMITS},
		{"universalJoint", RB_JOINT_TYPE_UNIVERSAL},
		{"ballInSocketJoint", RB_JOINT_TYPE_BALL_IN_SOCKET},
		{"CDP_Box", RB_BOX},
		{"planar", RB_PLANAR},
		{"ODEGroundParameters", RB_ODE_GROUND_COEFFS},
		{"softBody", RB_SOFT_BODY}
	};

	//declare a list of keywords
	int keyWordCount = sizeof(keywords)/sizeof(keywords[0]);

	for (int i=0;i<keyWordCount;i++){
		if (strncmp(buffer, keywords[i].keyWord, strlen(keywords[i].keyWord)) == 0){
			buffer += strlen(keywords[i].keyWord);
			return keywords[i].retVal;
		}
	}

	return RB_NOT_IMPORTANT;
}

