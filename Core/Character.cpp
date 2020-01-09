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

#include "Character.h"
#include <Utils/Utils.h>
#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <Globals.h>
#include "Physics/EllipticalContacts.h"

/**
    the constructor
*/
Character::Character(ArticulatedFigure* ch){
    if (ch == NULL)
        throwError("Can't build a character without an articulated figure!!!\n");
    this->af = ch;

    //populate the joints while at it
    joints.clear();
    af->add_joints_to_list(&joints);

    ///TODO when you have time change the way the idxs are initialised
    //and now set the idxs of the joint
    init_idxs();

    //we init the mass of the af
    af->compute_mass();


    //we now init some pointer to some elemnts that we will need
    _root_top=NULL;
    for (int i=0;i<joints.size();++i){
        if (getJoint(i)->parent()==getRoot()){
            if (getJoint(i)->name().find("Hip")==std::string::npos){
                if (root_top()!=NULL){
                    throw "Character::Character multiples root top found ...";
                }
                _root_top=getJoint(i);
            }
        }
    }

    //initialize the elliptical contact
    left_contacts=new EllipticalContacts(getJointByName("lAnkle")->child(),true);
    right_contacts=new EllipticalContacts(getJointByName("rAnkle")->child(),false);

    left_contacts->computeForces();
    right_contacts->computeForces();

}

/**
    the destructor
*/
Character::~Character(void){
    //nothing to do. We'll let whoever created the world deal with freeing it up
}

bool Character::is_swing_foot_in_contact()
{
    for (int i=0; i<4;++i){
        if (_force_swing_foot[i].length()>0.001){
            return true;
        }
    }
    if (_force_swing_toes.length()>0.001){
        return true;
    }

    return false;
}

double Character::get_mass(bool count_stance_leg)
{
    double curMass = af->root->getMass();
    double totalMass = curMass;
    for (uint i = 0; i <joints.size(); i++){
        curMass = joints[i]->child()->getMass();
        totalMass += curMass;
    }

    if (!count_stance_leg){
        double stance_leg_mass=0;
        Joint* cur_joint=stance_hip();
        stance_leg_mass+=cur_joint->child()->getMass();
        while(cur_joint->child()!=NULL&&!cur_joint->child()->child_joints().empty()){
            cur_joint=cur_joint->child()->child_joints().front();
            stance_leg_mass+=cur_joint->child()->getMass();
        }
        totalMass-=stance_leg_mass;
    }

    return totalMass;
}

void Character::update_stance(int stance)
{
    _stance=stance;

    _swing_foot = getJointByName("lAnkle")->child();
    _stance_foot = getJointByName("rAnkle")->child();
    _swing_hip = getJointByName("lHip");
    _stance_hip = getJointByName("rHip");
    if (is_left_stance()){
        ArticulatedRigidBody* buff=_swing_foot;
        _swing_foot=_stance_foot;
        _stance_foot=buff;

        Joint* buff2=_swing_hip;
        _swing_hip=_stance_hip;
        _stance_hip=buff2;
    }


}

#include <fstream>
#include <sstream>


void Character::to_xml()
{
    af->to_xml(0);
}



void Character::update_d_and_v(Vector3d &d, Vector3d &v)
{
    Quaternion characterFrame = getHeadingHorizontal();
//    Quaternion characterFrame = Quaternion::getRotationQuaternion(SimGlobals::desiredHeading,SimGlobals::up);

    Vector3d comPosition = getCOM();

    d = Vector3d(stance_foot()->getCMPosition(), comPosition);
    //d is now in world coord frame, so we'll represent it in the 'character frame'
    d = characterFrame.getComplexConjugate().rotate(d);
    //compute v in the 'character frame' as well
    ///no keep it in the world frame
//    v = characterFrame.getComplexConjugate().rotate(getCOMVelocity());
    v = getCOMVelocity();

}

bool Character::is_foot(RigidBody *body)
{
    if (body==swing_foot()||body==stance_foot()){
        return true;
    }
    return false;
}

void Character::organize_foot_contact_forces(DynamicArray<ContactPoint> *cfs){
    //*
    left_contacts->computeForces();
    right_contacts->computeForces();


    ArticulatedRigidBody* swing_body = _swing_foot;
    ArticulatedRigidBody* stance_body = _stance_foot;
    ArticulatedRigidBody* swing_toe = static_cast<ArticulatedRigidBody*>(_swing_foot)->child_joints().front()->child();
    ArticulatedRigidBody* stance_toe = static_cast<ArticulatedRigidBody*>(_stance_foot)->child_joints().front()->child();

    //so I start by resetting the values
    for (int i = 0; i < 4; i++){
        _force_stance_foot[i] = Vector3d(0,0,0);
        _force_swing_foot[i] = Vector3d(0, 0, 0);
    }
    _force_stance_toes = Vector3d(0, 0, 0);
    _force_swing_toes = Vector3d(0, 0, 0);
    Point3d tmpP;
//    std::vector<std::string> test_vector1;
//    std::vector<std::string> test_vector2;
    for (uint i = 0; i<cfs->size(); i++){
//        test_vector1.push_back((*cfs)[i].rb1->name());
//        test_vector2.push_back((*cfs)[i].rb2->name());
        if (((*cfs)[i].rb1 == stance_body) || ((*cfs)[i].rb2 == stance_body)){

            tmpP = stance_body->getLocalCoordinates((*cfs)[i].cp);
            if (tmpP.z < 0){
                if (tmpP.x > 0){
                    _force_stance_foot[0] += (*cfs)[i].f;
                }
                else{
                    _force_stance_foot[1] += (*cfs)[i].f;
                }
            }
            else{
                if (tmpP.x > 0){
                    _force_stance_foot[2] += (*cfs)[i].f;
                }
                else{
                    _force_stance_foot[3] += (*cfs)[i].f;
                }
            }
        }
        else if (((*cfs)[i].rb1 == stance_toe) || ((*cfs)[i].rb2 == stance_toe)){
            _force_stance_toes += (*cfs)[i].f;
        }
        else if(((*cfs)[i].rb1 == swing_body) || ((*cfs)[i].rb2 == swing_body)){
            if((*cfs)[i].rb1 == swing_body) {
                if ((*cfs)[i].rb2->name()!="ground"){
                    //std::cout<<(*cfs)[i].rb2->name();
                    continue;
                }
            }
            if ((*cfs)[i].rb2 == swing_body){
                if ((*cfs)[i].rb1->name()!="ground"){
                    //std::cout<<(*cfs)[i].rb1->name();
                    continue;
                }
            }



            tmpP = swing_body->getLocalCoordinates((*cfs)[i].cp);
            if (tmpP.z < 0){
                if (tmpP.x > 0){
                    _force_swing_foot[0] += (*cfs)[i].f;
                }
                else{
                    _force_swing_foot[1] += (*cfs)[i].f;
                }
            }
            else{
                if (tmpP.x > 0){
                    _force_swing_foot[2] += (*cfs)[i].f;
                }
                else{
                    _force_swing_foot[3] += (*cfs)[i].f;
                }
            }
        }
        else if (((*cfs)[i].rb1 == swing_toe) || ((*cfs)[i].rb2 == swing_toe)){
            _force_swing_toes += (*cfs)[i].f;
        }
    }
    //*/
    static bool first_time=true;
    if (first_time){
        std::remove("contact_forces.txt");
        first_time=false;
    }

    Vector3d sum_forces_left=Vector3d(0,0,0);
    Vector3d sum_forces_right=Vector3d(0,0,0);

    for (int i=0;i<4;++i){
        sum_forces_left+=_force_stance_foot[i];
        sum_forces_right+=_force_swing_foot[i];
    }
    sum_forces_left+=_force_stance_toes;
    sum_forces_right+=_force_swing_toes;

    if (!is_left_stance()){
        Vector3d temp=sum_forces_left;
        sum_forces_left=sum_forces_right;
        sum_forces_right=temp;
    }

    std::ostringstream oss;
    oss<<sum_forces_left.x<<"  "<<sum_forces_left.y<<"  "<<sum_forces_left.z<<"  "<<
         sum_forces_right.x<<"  "<<sum_forces_right.y<<"  "<<sum_forces_right.z<<"  "<<
         left_contacts->getSumForces().x<<"  "<<left_contacts->getSumForces().y<<"  "<<left_contacts->getSumForces().z<<"  "<<
         right_contacts->getSumForces().x<<"  "<<right_contacts->getSumForces().y<<"  "<<right_contacts->getSumForces().z<<"  "<<
         std::endl;
    std::ofstream myfile ("contact_forces.txt", std::iostream::app);
    if (myfile.is_open())
    {
        myfile<<oss.str();
        myfile.close();
    }
    else std::cout << "Unable to open file";

}

void Character::organize_foot_contact_forces(std::vector<ContactPoint> *cfs, ArticulatedRigidBody *foot, std::vector<Vector3d>& f_foot)
{
    if (foot==NULL){
        throw("Character::organize_foot_contact_forces fu u give me correct values");
    }

    if (f_foot.size()!=5){
        f_foot.resize(5);
    }

    //so I start by resetting the values
    for (int i = 0; i < 5; i++){
        f_foot[i] = Vector3d(0,0,0);
    }
    ArticulatedRigidBody* toes=foot->child_joints()[0]->child();

    Point3d tmpP;
    for (uint i = 0; i<cfs->size(); i++){
        if (((*cfs)[i].rb1 == foot) || ((*cfs)[i].rb2 == foot)){
            tmpP = foot->getLocalCoordinates((*cfs)[i].cp);
            if (tmpP.z < 0){
                if (tmpP.x > 0){
                    f_foot[0] = (*cfs)[i].f;
                }
                else{
                    f_foot[1] = (*cfs)[i].f;
                }
            }
            else{
                if (tmpP.x > 0){
                    f_foot[2] = (*cfs)[i].f;
                }
                else{
                    f_foot[3] = (*cfs)[i].f;
                }
            }
        }
        //*

        else if (((*cfs)[i].rb1 == toes) || ((*cfs)[i].rb2 == toes)){
            f_foot[4] += (*cfs)[i].f;
        }
        //*/
    }
    //*/

}

void Character::get_leg_contact_influence(std::vector<Joint *> &vect_joints, std::vector<double> &vect_influence)
{
    if (vect_joints.size()<1){
        vect_joints.resize(2);
        vect_joints[0]=getJointByName("rHip");
        vect_joints[1]=getJointByName("lHip");
    }

    std::vector<Vector3d> vect_force;
    vect_force.resize(2);
    vect_force[0]=get_force_on_foot(getJointByName("rAnkle")->child(), true);
    vect_force[1]=get_force_on_foot(getJointByName("lAnkle")->child(), true);

    double total_force=0;
    for (int i=0;i<vect_force.size();++i){
        total_force+=vect_force[i].y;
    }

    //now we get the influence
    vect_influence.resize(2,0.0);
    if (total_force>0){
        for (int i=0;i<vect_force.size();++i){
            vect_influence[i]=vect_force[i].y/total_force;
        }
    }
}



Vector3d Character::get_force_on_foot(RigidBody* foot, bool consider_toes){
    Vector3d fNet;

    if (foot == _swing_foot){
        for (int i = 0; i < 4; ++i){
            fNet.y += std::abs(_force_swing_foot[i].y);
        }
        if (consider_toes){
            fNet.y += std::abs(_force_swing_toes.y);
        }
    }
    else{
        for (int i = 0; i < 4; ++i){
            fNet.y += std::abs(_force_stance_foot[i].y);
        }
        if (consider_toes){
            fNet.y += std::abs(_force_stance_toes.y);
        }
    }

    return fNet;
}

Vector3d Character::get_force_on_foot(bool is_swing_foot, bool consider_toes)
{
    if (is_swing_foot){
        return get_force_on_foot(_swing_foot,consider_toes);
    }else{
        return get_force_on_foot(_stance_foot,consider_toes);
    }
}

void Character::get_force_info_on_foot(RigidBody *rb, ForceStruct &heelForce, ForceStruct &frontFeetForce, ForceStruct &toeForce)
{
    //*
    //reset everything
    heelForce.F=Vector3d(0,0,0);
    frontFeetForce.F = Vector3d(0, 0, 0);
    toeForce.F = Vector3d(0, 0, 0);

    RigidBody* swing_foot = getJointByName("lAnkle")->child();
    if (_stance==0){
        swing_foot = getJointByName("rAnkle")->child();
    }

    if (rb == swing_foot){
        heelForce.F.y += std::abs(_force_swing_foot[0].y);
        heelForce.F.y += std::abs(_force_swing_foot[1].y);
        frontFeetForce.F.y += std::abs(_force_swing_foot[2].y);
        frontFeetForce.F.y += std::abs(_force_swing_foot[3].y);
        toeForce.F.y = std::abs(_force_swing_toes.y);
    }
    else{
        heelForce.F.y += std::abs(_force_stance_foot[0].y);
        heelForce.F.y += std::abs(_force_stance_foot[1].y);
        frontFeetForce.F.y += std::abs(_force_stance_foot[2].y);
        frontFeetForce.F.y += std::abs(_force_stance_foot[3].y);
        toeForce.F.y = std::abs(_force_stance_toes.y);
    }
    //*/
}

/**
    This method is used to populate the relative orientation of the parent and child bodies of joint i.
*/
void Character::getRelativeOrientation(int i, Quaternion* qRel){
    //rotation from child frame to world, and then from world to parent == rotation from child to parent
    joints[i]->compute_relative_orientation(*qRel);
}

void Character::getRelativeOrientationFuture(int i, Quaternion* qRel){
    //rotation from child frame to world, and then from world to parent == rotation from child to parent
    joints[i]->compute_relative_orientation_Future(*qRel);
}

/**
    This method is used to get the relative angular velocities of the parent and child bodies of joint i,
    expressed in parent's local coordinates.
    We'll assume that i is in the range 0 - joints.size()-1!!!
*/
void Character::getRelativeAngularVelocity(int i, Vector3d* wRel){
    *wRel = joints[i]->child()->getAngularVelocity() - joints[i]->parent()->getAngularVelocity();
    //we will store wRel in the parent's coordinates, to get an orientation invariant expression for it
    *wRel = joints[i]->parent()->getLocalCoordinates(*wRel);
}

void Character::getRelativeAngularVelocityFuture(int i, Vector3d* wRel){
    *wRel = joints[i]->child()->getAngularVelocityFuture() - joints[i]->parent()->getAngularVelocityFuture();
    //we will store wRel in the parent's coordinates, to get an orientation invariant expression for it
    *wRel = joints[i]->parent()->getLocalCoordinates(*wRel);
}

/**
    This method is used to get the relative angular velocities of the parent and child bodies of joint i,
    expressed in parent's local coordinates.
    We'll assume that i is in the range 0 - joints.size()-1!!!
*/
void Character::getRelativeAngularVelocityAvg(int i, Vector3d* wRel){
    *wRel = joints[i]->child()->get_angular_velocity_avg(); - joints[i]->parent()->get_angular_velocity_avg();
    //we will store wRel in the parent's coordinates, to get an orientation invariant expression for it
    *wRel = joints[i]->parent()->getLocalCoordinates(*wRel);
}


/**
    This method populates the dynamic array passed in with the state of the character.
    For the state, we consider the 13-dimensional state of the root, and then only
    the relative orientation and angular velocity (as measured from the parent) for
    every other link. The velocities of the CM are derived from this information,
    using the velocity propagation technique (if that's what it is called).
    The only thing we will not be storing explicitly is the positions of the CMs of the rigid bodies.
    The order in which the bodies appear is given by the array of joints.
    This works under the assumption that in the joint
    sequence, the parent of any rigid body appears before its children (assuming that for each joint
    we read the parent first and then the child).
*/
void Character::getState(DynamicArray<double>* state){
    //we'll push the root's state information - ugly code....
    state->push_back(af->root->state.position.x);
    state->push_back(af->root->state.position.y);
    state->push_back(af->root->state.position.z);

    state->push_back(af->root->state.orientation.s);
    state->push_back(af->root->state.orientation.v.x);
    state->push_back(af->root->state.orientation.v.y);
    state->push_back(af->root->state.orientation.v.z);

    state->push_back(af->root->state.velocity.x);
    state->push_back(af->root->state.velocity.y);
    state->push_back(af->root->state.velocity.z);

    state->push_back(af->root->state.angular_velocity.x);
    state->push_back(af->root->state.angular_velocity.y);
    state->push_back(af->root->state.angular_velocity.z);

    //now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
    //we are assuming here that each joint is revolute!!!
    Quaternion qRel;
    Vector3d wRel;

    for (uint i=0;i<joints.size();i++){
        getRelativeOrientation(i, &qRel);

        state->push_back(qRel.s);
        state->push_back(qRel.v.x);
        state->push_back(qRel.v.y);
        state->push_back(qRel.v.z);

        getRelativeAngularVelocity(i, &wRel);
        state->push_back(wRel.x);
        state->push_back(wRel.y);
        state->push_back(wRel.z);
    }
}

/**
    This method populates the state of the current character with the values that are passed
    in the dynamic array. The same conventions as for the getState() method are assumed.
*/
void Character::setState(DynamicArray<double>* state, int start){
    ReducedCharacterState rs(state, start);

    //kinda ugly code....
    af->root->state.position = rs.getPosition();
    af->root->state.orientation = rs.getOrientation();
    af->root->state.velocity = rs.getVelocity();
    af->root->state.angular_velocity = rs.getAngularVelocity();

    //now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
    //we are assuming here that each joint is revolute!!!
    Quaternion qRel;
    Vector3d wRel;

    Vector3d r;
    Vector3d d;
    Vector3d vRel;

//    Point3d sf_p1, sf_p2;
//    double sh_a,sh_a1,sh_a2;
    //	af->root->updateToWorldTransformation();
    for (uint j=0;j<joints.size();j++){

        qRel = rs.getJointRelativeOrientation(j);
        wRel = rs.getJointRelativeAngVelocity(j);
        //transform the relative angular velocity to world coordinates
        wRel = joints[j]->parent()->getWorldCoordinates(wRel);

        //now that we have this information, we need to restore the state of the rigid body.

        //set the proper orientation
        joints[j]->child()->state.orientation = joints[j]->parent()->state.orientation * qRel;
        //and the proper angular velocity
        joints[j]->child()->state.angular_velocity = joints[j]->parent()->state.angular_velocity + wRel;
        //and now set the linear position and velocity
//        if (joints[j]->child()==swing_foot()){
//            sf_p1=swing_foot()->getCMPosition();;
//        }

        joints[j]->fix_joint_constraints(false, true, false);
//        		joints[j]->child->updateToWorldTransformation();

        /*
        if (joints[j]->child()==swing_foot()){
            sf_p2=swing_foot()->getCMPosition();
        }
        if (joints[j]==swing_hip()){
            sh_a=(swing_hip()->child()->getOrientation()*
                  swing_hip()->parent()->getOrientation().getComplexConjugate()).getRotationAngle(Vector3d(0,0,0));
            sh_a1=(swing_hip()->child()->getOrientation()).getRotationAngle(Vector3d(0,0,0));
            sh_a2=(swing_hip()->parent()->getOrientation().getComplexConjugate()).getRotationAngle(Vector3d(0,0,0));

        }
        //*/
    }

}

/**
    this method is used to return the current heading of the character, specified as an angle measured in radians
*/
double Character::getHeadingAngle(){
    //first we need to get the current heading of the character. Also, note that q and -q represent the same rotation
    Quaternion q = getHeading();
    if (q.s<0){
        q.s = -q.s;
        q.v = -q.v;
    }
    double currentHeading = 2 * safeACOS(q.s);
    if (q.v.dotProductWith(SimGlobals::up) < 0)
        currentHeading = -currentHeading;
    return currentHeading;
}


/**
    This method returns the dimension of the state. Note that we will consider
    each joint as having 3-DOFs (represented by the 4 values of the quaternion)
    without taking into account what type of joint it is (i.e. a hinge joint
    has only one degree of freedom, but we will still consider the relative orientation
    of the child relative to the parent as a quaternion.
*/
int Character::getStateDimension(){
    //13 for the root, and 7 for every other body (and each body is introduced by a joint).
    return 13 + 7 * joints.size();
}


/**
    This method is used to mirror the given orientation. It is assumed that rotations in the sagittal plane
    (about parent frame x-axis) are to stay the same, while the rotations about the other axes need to be reversed.
*/
Quaternion mirrorOrientation(const Quaternion& q){
    //get the rotation about the parent's x-axis
    Quaternion qSagittal = q.getComplexConjugate().decomposeRotation(Vector3d(1, 0, 0)).getComplexConjugate();
    //this is what is left, if we removed that component of the rotation
    Quaternion qOther = q * qSagittal.getComplexConjugate();
    //and now negate the non-sagittal part of the rotation, but keep the rotation in the sagittal plane
    return qOther.getComplexConjugate() * qSagittal;
}

/**
    This method is used to multiply, element-wise, the two vectors that are passed in
*/
Vector3d elemWiseMultiply(const Vector3d& a, const Vector3d& b){
    return Vector3d(a.x * b.x, a.y * b.y, a.z * b.z);
}

/**
    this method takes the state of the character, passed in as an array of doubles, and reverses it inplace
*/
void Character::reverseStanceOfStateArray(DynamicArray<double>* state, int start){
    ReducedCharacterState chS(state, start);

    char name[100];

    //now we're ready to start swapping left and right body parts
    for (uint i=0;i<joints.size();i++){
        //get the name of the joint, and if it's a right joint, and can be matched with one on the left, swap them
        strcpy(name, joints[i]->name().c_str());
        if (name[0] == 'r'){
            int rJointIndex = i;
            name[0] = 'l';
            int lJointIndex = this->getJointIndex(name);
            if (lJointIndex > 0){
                //ok, swap the angular velocities and the orientations
                Quaternion qr = chS.getJointRelativeOrientation(rJointIndex);
                Quaternion ql = chS.getJointRelativeOrientation(lJointIndex);
                chS.setJointRelativeOrientation(ql, rJointIndex);
                chS.setJointRelativeOrientation(qr, lJointIndex);
                Vector3d wr = chS.getJointRelativeAngVelocity(rJointIndex);
                Vector3d wl = chS.getJointRelativeAngVelocity(lJointIndex);
                chS.setJointRelativeAngVelocity(wl, rJointIndex);
                chS.setJointRelativeAngVelocity(wr, lJointIndex);
            }
        }
    }

    //now reflect/mirror all the orientations and angular velocities,
    //now we're ready to start swapping left and right body parts
    for (uint i=0;i<joints.size();i++){
        //orientations
        chS.setJointRelativeOrientation(mirrorOrientation(chS.getJointRelativeOrientation(i)), i);
        //angular velocities - assume that ang velocity about the x axis results in rotation in the sagittal plane, so it shouldn't be changed
        chS.setJointRelativeAngVelocity(elemWiseMultiply(chS.getJointRelativeAngVelocity(i), Vector3d(1, -1, -1)), i);
    }

    //and now take care of the root information
    chS.setOrientation(mirrorOrientation(chS.getOrientation()));
    chS.setAngularVelocity(elemWiseMultiply(chS.getAngularVelocity(), Vector3d(1, -1, -1)));
    //for the velocity, we only want the x-component reversed (i.e. movement in the sagittal plane should stay the same)
    chS.setVelocity(elemWiseMultiply(chS.getVelocity(), Vector3d(-1, 1, 1)));
}

/**
    this method is used to retrieve the reduced state of the character, but with the stance reversed.
*/
void Character::getReverseStanceState(DynamicArray<double>* state){
    //this is the index where we will start populating the data in the state array
    int start = state->size();
    getState(state);
    reverseStanceOfStateArray(state, start);
}


/**
This method is used to compute the center of mass of the articulated figure.
*/
Vector3d Character::getCOM(bool keep_weighted){
    Vector3d COM = Vector3d(af->root->getCMPosition()) * af->root->getMass();
    double curMass = af->root->getMass();
    double totalMass = curMass;
    for (uint i = 0; i <joints.size(); i++){
        curMass = joints[i]->child()->getMass();
        totalMass += curMass;
        COM.addScaledVector(joints[i]->child()->getCMPosition(), curMass);
    }

    if (!keep_weighted){
        COM /= totalMass;
    }

    return COM;
}





/**
    This method is used to compute the velocity of the center of mass of the articulated figure.
*/
Vector3d Character::getCOMVelocity(){
    Vector3d COMVel = Vector3d(af->root->getCMVelocity()) * af->root->getMass();
    double curMass = af->root->getMass();
    double totalMass = curMass;
    for (uint i=0; i <joints.size(); i++){
        curMass = joints[i]->child()->getMass();
        totalMass += curMass;
        COMVel.addScaledVector(joints[i]->child()->getCMVelocity() , curMass);
    }

    COMVel /= totalMass;

    return COMVel;
}

/**
This method is used to compute the center of mass of the articulated figure.
I supose the only the torso can lead to the top of the body.
*/
Vector3d Character::getTopCOM(){

    //this container will help me not to do a recursive funtion
    std::vector<ArticulatedRigidBody*> body_buffer;

    //first I need to find the torso
    for (uint i = 0; i < joints.size(); ++i){
        if (strcmp(joints[i]->child()->name().c_str(), "torso") == 0){
            //we store it and add it to the COM
            body_buffer.push_back(joints[i]->child());
            break;
        }
    }

    //now that we have our basis we will iterate on all the top of the caracter
    for (uint i = 0; i < (int)body_buffer.size(); ++i){
        std::vector<Joint*> vec_child_joints= body_buffer[i]->child_joints();
        for (uint j = 0; j < vec_child_joints.size(); ++j){
            body_buffer.push_back(vec_child_joints[j]->child());
        }
    }

    //now that we have all the bodies I can simply calculate the COM
    Vector3d COM;
    double total_mass = 0;
    double cur_mass = 0;
    for (uint i = 0; i < (int)body_buffer.size(); ++i){
        cur_mass = body_buffer[i]->getMass();
        total_mass += cur_mass;
        COM.addScaledVector(body_buffer[i]->getCMPosition(), cur_mass);

    }

    COM /= total_mass;

    return COM;
}


/**
This method is used to compute the center of mass of the articulated figure.
I supose anything but the torso can lead to the bottom of the body.
*/
Vector3d Character::getBottomCOM(){

    //this container will help me not to do a recursive funtion
    std::vector<ArticulatedRigidBody*> body_buffer;

    //first I need to find the torso
    for (uint i = 0; i < joints.size(); ++i){
        if (strcmp(joints[i]->child()->name().c_str(), "torso") != 0){
            //we store it and add it to the COM
            body_buffer.push_back(joints[i]->child());
        }
    }

    //now that we have our basis we will iterate on all the top of the caracter
    for (uint i = 0; i < (int)body_buffer.size(); ++i){
        std::vector<Joint*> vec_child_joints = body_buffer[i]->child_joints();
        for (uint j = 0; j < vec_child_joints.size(); ++j){
            body_buffer.push_back(vec_child_joints[j]->child());
        }
    }

    //now that we have all the bodies I can simply calculate the COM
    Vector3d COM;
    double total_mass = 0;
    double cur_mass = 0;
    for (uint i = 0; i < (int)body_buffer.size(); ++i){
        cur_mass = body_buffer[i]->getMass();
        total_mass += cur_mass;
        COM.addScaledVector(body_buffer[i]->getCMPosition(), cur_mass);

    }

    COM /= total_mass;

    return COM;
}

/**
This method is used to get the part of the sqeleton on whihch we have to have an effect to control the speed
following Coros 2010 paper. I is conposed of the standing leg (except the toes), the torso and the head.
*/
void Character::getSpeedControlSqueleton(int cur_stance, std::vector<Joint*>& vect_squeleton){
    //first I need to find the torso
    for (uint i = 0; i < joints.size(); ++i){
        if (joints[i]->parent() == getRoot()){
            if (strcmp(joints[i]->child()->name().c_str(), "torso") == 0){
                //we store it and add it to the COM
                vect_squeleton.push_back(joints[i]);

            }
            if (cur_stance<=0){
                if (strcmp(joints[i]->child()->name().c_str(), "lUpperLeg") == 0){
                    //we store it and add it to the COM
                    vect_squeleton.push_back(joints[i]);
                    Joint* cur_joint = joints[i];
                    for (;;){
                        //here I use the fact that i know there is only 1 child after each joint on the leg
                        std::vector<Joint*> child_joints= cur_joint->child()->child_joints();
                        cur_joint = child_joints[0];
                        if (strcmp(cur_joint->child()->name().c_str(), "lToes") == 0){
                            break;
                        }
                        vect_squeleton.push_back(cur_joint);
                    }
                }
            }
            if (cur_stance >= 0){
                if (strcmp(joints[i]->child()->name().c_str(), "rUpperLeg") == 0){
                    //we store it and add it to the COM
                    vect_squeleton.push_back(joints[i]);
                    Joint* cur_joint = joints[i];
                    for (;;){
                        //here I use the fact that i know there is only 1 child after each joint on the leg
                        std::vector<Joint*> child_joints = cur_joint->child()->child_joints();
                        cur_joint = child_joints[0];
                        if (strcmp(cur_joint->child()->name().c_str(), "rToes") == 0){
                            break;
                        }
                        vect_squeleton.push_back(cur_joint);
                    }
                }
            }
        }
    }
}

/**
this function can be used to have an easy access to the top of the body (meaning above the root)
*/
void Character::getCharacterTop(std::vector<Joint*>& body_top){
    //this container will help me not to do a recursive funtion
    std::vector<ArticulatedRigidBody*> body_buffer;

    for (uint i = 0; i < joints.size(); ++i){
        if (joints[i]->parent()== getRoot()){
            //we store it and add it to the COM
            if (strcmp(joints[i]->child()->name().c_str(), "torso") != 0){
                body_top.push_back(joints[i]);
            }
            //			body_buffer.push_back(joints[i]->child);
        }
    }

    //first I need to find the torso
    /*for (uint i = 0; i < joints.size(); ++i){
        if (strcmp(joints[i]->child->name, "torso") == 0){
            //we store it and add it to the COM
            body_top.push_back(joints[i]);
            body_buffer.push_back(joints[i]->child);
        }
    }*/

    //now that we have our basis we will iterate on all the top of the caracter
    //for (uint i = 0; i < (int)body_buffer.size(); ++i){
    /*uint i = 0;

    std::vector<Joint*> vec_child_joints = body_buffer[i]->child_joints();
        for (uint j = 0; j < vec_child_joints.size(); ++j){
            body_top.push_back(vec_child_joints[j]);
            body_buffer.push_back(vec_child_joints[j]->child());
        }*/
    //}
}

/**
this function can be used to have an easy access to the bottom of the body (meaning under the root)
*/
void Character::getCharacterBottom(std::vector<Joint*>& body_bottom){
    //this container will help me not to do a recursive funtion
    std::vector<ArticulatedRigidBody*> body_buffer;

    //first I need to find the torso
    for (uint i = 0; i < joints.size(); ++i){
        if (joints[i]->parent() == getRoot()){
            if (strcmp(joints[i]->child()->name().c_str(), "torso") != 0){
                //we store it and add it to the COM
                body_bottom.push_back(joints[i]);
                body_buffer.push_back(joints[i]->child());
            }
        }
    }

    //now that we have our basis we will iterate on all the top of the caracter
    for (uint i = 0; i < (int)body_buffer.size(); ++i){
        std::vector<Joint*> vec_child_joints = body_buffer[i]->child_joints();
        for (uint j = 0; j < vec_child_joints.size(); ++j){
            body_bottom.push_back(vec_child_joints[j]);
            body_buffer.push_back(vec_child_joints[j]->child());
        }
    }
}


/**
    This method is used to return the number of joints of the character.
*/
int Character::getJointCount(){
    return joints.size();
}

/**
    This method is used to return the articulated figure of the character.
*/
ArticulatedFigure* Character::getAF(){
    return af;
}

/**
    this method is used to rotate the character about the vertical axis, so that it's default heading has the value that is given as a parameter
*/
void Character::setHeading(Quaternion heading){
    DynamicArray<double> state;
    getState(&state);
    setHeading(heading, &state);
    setState(&state);
}

/**
    this method is used to rotate the character (well, the character whose state is passed in as a parameter)
    about the vertical axis, so that it's default heading has the value that is given as a parameter
*/
void Character::setHeading(Quaternion heading, DynamicArray<double>* state, int start){
    ReducedCharacterState chS(state, start);
    Quaternion oldHeading, newHeading, qRoot;
    //get the current root orientation, that contains information regarding the current heading
    qRoot = chS.getOrientation();
    //get the twist about the vertical axis...
    oldHeading = computeHeading(qRoot);
    //now we cancel the initial twist and add a new one of our own choosing
    newHeading = heading * oldHeading.getComplexConjugate();
    //add this component to the root.
    chS.setOrientation(newHeading * qRoot);
    //and also update the root velocity and angular velocity
    chS.setVelocity(newHeading.rotate(chS.getVelocity()));
    chS.setAngularVelocity(newHeading.rotate(chS.getAngularVelocity()));
}

/**
    this method is used to rotate the character (well, the character whose state is passed in as a parameter)
    about the vertical axis, so that it's default heading has the value that is given as a parameter
*/
void Character::setHeading(double val, DynamicArray<double>* state, int start){
    setHeading(Quaternion::getRotationQuaternion(val, Vector3d(0,1,0)), state, start);
}

/**
    this method is used to rotate the character about the vertical axis, so that it's default heading has the value that is given as a parameter
*/
void Character::setHeading(double val){
    DynamicArray<double> state;
    getState(&state);
    setHeading(Quaternion::getRotationQuaternion(val, Vector3d(0,1,0)), &state);
    setState(&state);
}

/**
    this method is used to return the current heading of the character
*/
Quaternion Character::getHeading(){
    //get the current root orientation, that contains information regarding the current heading and retrieve the twist about the vertical axis
    return computeHeading(af->root->getOrientation());
}


Quaternion Character::getHeadingHorizontal()
{
    return getHeading();
}

/**
    this method is used to read the reduced state of the character from the file
*/
void Character::loadReducedStateFromFile(char* fName){
    std::vector<double> state;
    readReducedStateFromFile(fName, &state);
    setState(&state);

    Globals::current_starting_pos_file = std::string(fName);
}

/**
    this method is used to read the reduced state of the character from the file, into the array passed in as a parameter. The
    state of the character is not modified
*/
void Character::readReducedStateFromFile(char* fName, DynamicArray<double> *state){
    if (fName == NULL)
        throwError("cannot write to a file whose name is NULL!");

    int start = state->size();

    FILE* fp = fopen(fName, "r");
    if (fp == NULL)
        throwError("cannot open the file \'%s\' for reading... error: %i", fName, errno);

    double temp1, temp2, temp3, temp4;

    char line[100];

    //read the heading first...
    double heading;
    readValidLine(line, fp);
    sscanf(line, "%lf", &heading);

    readValidLine(line, fp);
    sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
    state->push_back(temp1);
    state->push_back(temp2);
    state->push_back(temp3);
    readValidLine(line, fp);
    sscanf(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
    state->push_back(temp1);
    state->push_back(temp2);
    state->push_back(temp3);
    state->push_back(temp4);
    readValidLine(line, fp);
    sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
    state->push_back(temp1);
    state->push_back(temp2);
    state->push_back(temp3);
    readValidLine(line, fp);
    sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
    state->push_back(temp1);
    state->push_back(temp2);
    state->push_back(temp3);

    for (uint i=0;i<joints.size();i++){
        readValidLine(line, fp);
        sscanf(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
        state->push_back(temp1);
        state->push_back(temp2);
        state->push_back(temp3);
        state->push_back(temp4);
        readValidLine(line, fp);
        sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
        state->push_back(temp1);
        state->push_back(temp2);
        state->push_back(temp3);
    }

    //now set the heading...
    setHeading(Quaternion::getRotationQuaternion(heading, SimGlobals::up), state, start);

    fclose(fp);
}

/**
    this method is used to write the reduced state of the character to a file
*/
void Character::saveReducedStateToFile(char* fName, DynamicArray<double>& state){
    if (fName == NULL)
        throwError("cannot write to a file whose name is NULL!");

    FILE* fp = fopen(fName, "w");
    if (fp == NULL)
        throwError("cannot open the file \'%s\' for writing...", fName);

    //retrieve the current heading, write it to file, and then set a zero heading for the state
    double heading = getHeadingAngle();
    setHeading(Quaternion::getRotationQuaternion(0, SimGlobals::up), &state);

    fprintf(fp, "# order is:\n# Heading\n# Position\n# Orientation\n# Velocity\n# AngularVelocity\n\n# Relative Orientation\n# Relative Angular Velocity\n#----------------\n\n# Heading\n%lf\n\n# Root(%s)\n", heading, af->root->name());
    fprintf(fp, "%lf %lf %lf\n", state[0], state[1], state[2]);
    fprintf(fp, "%lf %lf %lf %lf\n", state[3], state[4], state[5],state[6]);
    fprintf(fp, "%lf %lf %lf\n", state[7], state[8], state[9]);
    fprintf(fp, "%lf %lf %lf\n\n", state[10], state[11], state[12]);

    for (uint i=0;i<joints.size();i++){
        fprintf(fp, "# %s\n", joints[i]->name().c_str());
        fprintf(fp, "%lf %lf %lf %lf\n", state[13+7*i+0], state[13+7*i+1], state[13+7*i+2],state[13+7*i+3]);
        fprintf(fp, "%lf %lf %lf\n", state[13+7*i+4], state[13+7*i+5], state[13+7*i+6]);
        fprintf(fp, "\n");
    }
    fclose(fp);
}

/**
    this method is used to write the reduced state of the character to the file
*/
void Character::saveReducedStateToFile(char* fName){
    DynamicArray<double> state;
    getState(&state);

    //those line will simply position the caracter at the origin
    state[0] = 0;
    state[2] = 0;

    saveReducedStateToFile(fName, state);
}


double Character::get_stance_foot_weight_ratio(){
    Vector3d stanceFootForce = get_force_on_foot(false, true);
    Vector3d swingFootForce = get_force_on_foot(true, true);
    double totalYForce = (stanceFootForce + swingFootForce).dotProductWith(SimGlobals::up);

    if (IS_ZERO(totalYForce))
        return -1;
    else
        return stanceFootForce.dotProductWith(SimGlobals::up) / totalYForce;
}
