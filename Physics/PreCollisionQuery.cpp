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

#include <Physics/PreCollisionQuery.h>
#include <iostream>
#include <sstream>

PreCollisionQuery::PreCollisionQuery(void){

}


PreCollisionQuery::~PreCollisionQuery(void){

}

/**
    This method returns true if the pair of rigid bodies here should be checked for collisions, false otherwise.
    Note that the ground does not have a rigid body associated with it, so if a rigid body is NULL, it means it
    corresponds to the ground. The joined variable is set to true if the two bodies are connected by a joint,
    false otherwise.
*/
bool PreCollisionQuery::shouldCheckForCollisions(RigidBody *rb1, RigidBody* rb2){
    //don't allow collisions between static things
    if (rb1->isLocked() && rb2->isLocked())
        return false;

    //we do not want to check the objects for collisions if they are connected by a joint (joint limits should be used for that).
    //for that we need articuled rigid bodies
    ArticulatedRigidBody* arb1=dynamic_cast<ArticulatedRigidBody*>(rb1);
    ArticulatedRigidBody* arb2=dynamic_cast<ArticulatedRigidBody*>(rb2);

    if (arb1!=NULL&&arb2!=NULL){
        for (int i=0; i<arb1->child_joints().size();++i){
            if (arb2->parent_joint()==arb1->child_joints()[i]){
                return false;
            }
        }
        //*
        for (int i=0; i<arb2->child_joints().size();++i){
            if (arb1->parent_joint()==arb2->child_joints()[i]){
                return false;
            }
        }
        //*/


        //don't do intersections between solids from a single body
        if (arb1->af() != NULL){
            if (arb1->af() == arb2->af()){
                return false;
            }
        }
    }
    /*
    //debug print
    if (rb1->name()!="ground"&&rb2->name()!="ground"){
        std::ostringstream oss;
        oss<<rb1->name()<<" "<<rb2->name();
        std::cout<<oss.str();
    }
    //*/
    return true;
}

