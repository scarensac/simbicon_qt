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

#include <Physics/rb/AbstractRBEngine.h>
#include "Character.h"
#include "Character.h"
#include <Utils/Utils.h>


class BaseControlFramework{
public:
	//this is the physical world that contains all the objects inside.
	AbstractRBEngine* pw;
	//this is the character that we want to control (we can easily have more than one if we wanted to).
	Character* bip;

public:
	/**
		Default constructor
	*/
	BaseControlFramework(void);

	/**
		destructor
	*/
	virtual ~BaseControlFramework(void);

	/**
		returns a reference to the physical world
	*/
	AbstractRBEngine* getWorld(){
		return pw;
	}

	/**
		this method is used to return a reference to the character 
	*/
	inline Character* getCharacter(){
		return bip;
	}

};
