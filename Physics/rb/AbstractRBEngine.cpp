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

#include "Physics/rb/AbstractRBEngine.h"
#include <Physics/rb/RBUtils.h>
#include <Utils/Utils.h>
#include <iostream>



AbstractRBEngine::AbstractRBEngine(void){
	this->objects = DynamicArray<RigidBody*>(300);
	this->objects.clear();
}

AbstractRBEngine::~AbstractRBEngine(void){
	//delete all the rigid bodies in this world
	for (uint i=0;i<objects.size();i++)
		delete objects[i];

	//delete the references to the articulated figures that we hold as well
	for (uint i=0;i<AFs.size();i++)
		delete AFs[i];
}

/**
	This method is used to draw all the rigid bodies in the world
*/
void AbstractRBEngine::drawRBs(int flags){
    for (uint i=0;i<objects.size();i++){
		objects[i]->draw(flags);
    }
}

/**
	This method renders all the rigid bodies as a set of vertices 
	and faces that will be appended to the passed OBJ file.

	vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
	multiple different meshes to the same OBJ file
	
	Returns the number of vertices written to the file
*/
uint AbstractRBEngine::renderRBsToObjFile(FILE* fp, uint vertexIdxOffset){
	
	uint nbVerts = 0;
	for (uint i=0;i<objects.size();i++)
		nbVerts += objects[i]->renderToObjFile(fp, vertexIdxOffset + nbVerts);

	return nbVerts;
}

/**
	This method returns the reference to the articulated rigid body with the given name, or NULL if it is not found
*/
ArticulatedRigidBody* AbstractRBEngine::getARBByName(std::string name){
    if (name == "")
		return NULL;
	for (uint i=0;i<ABs.size();i++)
        if (name == ABs[i]->name())
			return ABs[i];
	return NULL;
}

/**
	This method returns the reference to the rigid body with the given name, or NULL if it is not found
*/
RigidBody* AbstractRBEngine::getRBByName(std::string name){
    if (name == "")
		return NULL;
	for (uint i=0;i<this->objects.size();i++)
        if (name == objects[i]->name())
			return objects[i];
	return NULL;
}

/**
	This method reads a list of rigid bodies from the specified file.
*/
void AbstractRBEngine::loadRBsFromFile(char* fName){
	if (fName == NULL)
		throwError("NULL file name provided.");
	FILE *f = fopen(fName, "r");
	if (f == NULL)
        throwError("AbstractRBEngine::loadRBsFromFile:Could not open file: %s", fName);

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	RigidBody* newBody = NULL;
	ArticulatedFigure* newFigure = NULL;
	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer)>195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		int lineType = getRBLineType(line);
		switch (lineType) {
			case RB_RB:
				//create a new rigid body and have it load its own info...
                newBody = new RigidBody(f);
				objects.push_back(newBody);
				break;
			case RB_ARB:
				//create a new articulated rigid body and have it load its own info...
                newBody = new ArticulatedRigidBody(f);
				objects.push_back(newBody);
				//remember it as an articulated rigid body to be able to link it with other ABs later on
                ABs.push_back(static_cast<ArticulatedRigidBody*>(newBody));
				break;
			case RB_ARTICULATED_FIGURE:
				//we have an articulated figure to worry about...
                newFigure = new ArticulatedFigure();
				AFs.push_back(newFigure);
				newFigure->load_from_file(f, this);
				newFigure->add_joints_to_list(&jts);

				break;
			case RB_NOT_IMPORTANT:
				if (strlen(line)!=0 && line[0] != '#')
                    std::cerr<<"Ignoring input line: "<<line<<std::endl;
				break;
			default:
				throwError("Incorrect rigid body input file: \'%s\' - unexpected line.", buffer);
		}
	}

	//now we'll make sure that the joint constraints are satisfied
	for (uint i=0;i<AFs.size();i++)
        AFs[i]->fix_joint_constraints();


	fclose(f);
	//and now make sure that each rigid body's toWorld transformation is updated
//	for (uint i=0;i<objects.size();i++){
//		objects[i]->updateToWorldTransformation();
//	}
    if (!AFs.empty()){
        AFs.back()->to_xml(0);
    }

}

/**
	This method is used to get the state of all the rigid body in this collection.
*/
void AbstractRBEngine::getState(DynamicArray<double>* state){
	for (uint i=0;i<this->objects.size();i++){
		state->push_back(objects[i]->state.position.x);
		state->push_back(objects[i]->state.position.y);
		state->push_back(objects[i]->state.position.z);

		state->push_back(objects[i]->state.orientation.s);
		state->push_back(objects[i]->state.orientation.v.x);
		state->push_back(objects[i]->state.orientation.v.y);
		state->push_back(objects[i]->state.orientation.v.z);

		state->push_back(objects[i]->state.velocity.x);
		state->push_back(objects[i]->state.velocity.y);
		state->push_back(objects[i]->state.velocity.z);

        state->push_back(objects[i]->state.angular_velocity.x);
        state->push_back(objects[i]->state.angular_velocity.y);
        state->push_back(objects[i]->state.angular_velocity.z);
	}
}

/**
	This method is used to set the state of all the rigid body in this collection.
*/
void AbstractRBEngine::setState(DynamicArray<double>* state, int start){
	int i = start;
	for (uint j=0;j<this->objects.size();j++){
		objects[j]->state.position = Point3d((*state)[i+0], (*state)[i+1], (*state)[i+2]);
		i+=3;
		objects[j]->state.orientation = Quaternion((*state)[i+0], (*state)[i+1], (*state)[i+2], (*state)[i+3]);
		i+=4;
		objects[j]->state.velocity = Vector3d((*state)[i+0], (*state)[i+1], (*state)[i+2]);
		i+=3;
        objects[j]->state.angular_velocity = Vector3d((*state)[i+0], (*state)[i+1], (*state)[i+2]);
		i+=3;
//		objects[j]->updateToWorldTransformation();
	}
}

