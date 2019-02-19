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

#include "Physics/rb/articulatedrigidbody.h"
#include "Physics/joints/Joint.h"


ArticulatedRigidBody::ArticulatedRigidBody():RigidBody(){
    _parent_joint = NULL;
    _af = NULL;
}

ArticulatedRigidBody::ArticulatedRigidBody(FILE *f):ArticulatedRigidBody()
{
    loadFromFile(f);
}

ArticulatedRigidBody *ArticulatedRigidBody::cpy(bool keep_graphics){
    ArticulatedRigidBody* bdy=new ArticulatedRigidBody();
    bdy->state=state;
    bdy->props=props;
    bdy->_name=_name;
    bdy->_idx=_idx;


    //in depth copy for the cdps
    for (int i=0;i<cdps.size();++i){
        bdy->cdps.push_back(cdps[i]->cpy());
    }

    //same for the meshs
    //forget it for now it's useless and I don't even know what to copy in it ...
    if (keep_graphics){
        //bdy->meshes=meshes;
    }



    return bdy;
}

ArticulatedRigidBody::~ArticulatedRigidBody(void){
    //delete all the child joints
    for (uint i=0;i<_child_joints.size();i++){
        delete _child_joints[i];
    }
    child_joints().clear();
}

void ArticulatedRigidBody::add_link(Joint *j, ArticulatedRigidBody *child){
    //add the child joint
    _child_joints.push_back(j);

    //set the pointeurs inside the joint
    j->set_parent_child(this,child);

    //add the parent joint to the child
    child->_parent_joint=j;
    child->af(_af);
}

#include <sstream>
std::string ArticulatedRigidBody::to_xml(int nb_tab)
{
    std::ostringstream oss;
    Joint* j=parent_joint();
    if (j!=NULL){
        Point3d wp=this->getCMPosition();
        Point3d p = j->parent()->getLocalCoordinates(wp);
        oss<<tab_string(nb_tab++)<<"<body childclass=\"base_class\" name=\""<<name()<<"\" pos=\""<<
             -p.x<<" "<<p.z<<" "<<p.y<<
             "\">"<<std::endl;

        oss<<j->to_xml(nb_tab);

    }else{
        oss<<tab_string(nb_tab++)<<"<body childclass=\"base_class\" name=\""<<name()<<"\" pos=\""<<"0 0 2"<<"\">"<<std::endl;
        oss<<tab_string(nb_tab)<<"<joint class=\"free\"/>"<<std::endl;
    }

    oss<<tab_string(nb_tab)<<"<inertial "<<
         "pos=\""<<0<<" "<<0<<" "<<0<<"\" "<<
         "mass=\""<<getMass()<<"\" "<<
         "diaginertia=\""<<props.MOI_local.x<<" "<<props.MOI_local.z<<" "<<props.MOI_local.y<<"\" "<<
         "/>"<<std::endl;
    //diaginertia="2.46813e-005 1.77029e-005 1.71079e-005" />


    for (int i=0;i<cdps.size();++i){
        oss<<cdps[i]->to_xml(nb_tab);
    }

    for (int i=0;i<child_joints().size();++i){
        j=child_joints()[i];
        oss<<j->child()->to_xml(nb_tab);
    }

    oss<<tab_string(--nb_tab)<<"</body>"<<std::endl;

    return oss.str();
}

/**
    This method draws the current rigid body.
*/
void ArticulatedRigidBody::draw(int flags){
    RigidBody::draw(flags);

    if (!_parent_joint)
        return;

    if (flags & SHOW_JOINTS){
        //we will draw a little sphere at the location of the joint (we'll do it twice - once for the parent and one for the child. They should coincide
		//if the joint constraint is properly satisfied
        GLboolean lighting = glIsEnabled(GL_LIGHTING);
        glDisable(GL_LIGHTING);
        glColor3d(0.8, 0, 0.0);
        GLUtils::drawSphere(this->getWorldCoordinates(_parent_joint->child_joint_position()), 0.02, 4);
        GLUtils::drawSphere(_parent_joint->parent()->getWorldCoordinates(_parent_joint->parent_joint_position()), 0.02, 4);
        if (lighting){
            glEnable(GL_LIGHTING);
        }
	}
}

