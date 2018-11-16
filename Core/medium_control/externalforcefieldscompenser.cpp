#include "externalforcefieldscompenser.h"

#include "Core/Character.h"
#include <algorithm>
#include <iostream>
#include <sstream>

ExternalForceFieldsCompenser::ExternalForceFieldsCompenser(Character* c)
{
    character=c;

    _torques.resize(character->getJointCount());
    _torques2.resize(character->getJointCount());
}

ExternalForceFieldsCompenser::~ExternalForceFieldsCompenser()
{
    _torques.clear();
    _torques2.clear();
    stance_leg_idxs.clear();
}



void ExternalForceFieldsCompenser::compute_compensation(WaterImpact &resulting_impact){
    //reset values
    for (int i=0;i<_torques.size();++i){
        _torques[i]=Vector3d(0,0,0);
    }

    //check if the stance leg changed
    if (stance_leg_idxs.empty()||stance_leg_idxs.front()!=character->stance_hip()->idx()){
        stance_leg_idxs.clear();
        Joint* cur_joint=character->stance_hip();
        stance_leg_idxs.push_back(cur_joint->idx());
        while(cur_joint->child()!=NULL&&!cur_joint->child()->child_joints().empty()){
            cur_joint=cur_joint->child()->child_joints().front();
            stance_leg_idxs.push_back(cur_joint->idx());
        }
    }


    //now we compute the actual compensation
    for (uint i = 0; i<character->getJointCount(); i++){
        if (std::find(stance_leg_idxs.begin(),stance_leg_idxs.end(),i)==stance_leg_idxs.end()){
            std::string joint_name=character->getJoint(i)->name();
            if (joint_name=="lElbow"||joint_name=="rElbow"||joint_name=="rShoulder"||joint_name=="lShoulder"){
                continue;
            }


            Point3d pt1=character->getJoint(i)->child()->getCMPosition();

            //compute the gravity compensation
            Vector3d F1 = Vector3d(0, character->getJoint(i)->child()->getMass()*9.8, 0);

            throw("the boyancy compensation need to be repaired first");
            ///TODO redo that computation with the new model
            /*
            //compute the boyancy compensation
            if (resulting_impact.find(character->getJoint(i)->idx())!=resulting_impact.end()){
                ForceStruct  boyancy = resulting_impact[character->getJoint(i)->idx()].boyancy;
                if (!boyancy.F.isZeroVector()){
                    Vector3d vect_support = pt1 - character->getJoint(i)->child()->getWorldCoordinates(boyancy.pt);
                    double L2 = vect_support.length()*boyancy.F.length() / (F1.length() - boyancy.F.length());
                    pt1 = pt1 + vect_support / vect_support.length()*L2;
                    F1 -= boyancy.F;
                }
            }
            //*/

            pt1 = character->getJoint(i)->child()->getLocalCoordinates(pt1);

            //compute the equiv torque
            compute_joint_torques_equivalent_to_force(character->getJoint(i), pt1, F1, NULL);

        }
    }


    //I'll ad a compensation of the whoel body mass on the stance hip
    //because as for now computing the necessary torque on the stance hip is too hard
    Point3d pt1=character->getCOM();
    //compute the gravity compensation
    Vector3d F1 = Vector3d(0, character->get_mass(false)*9.8, 0);

    compute_joint_torques_equivalent_to_force(character->stance_hip(), pt1, F1, NULL);
    Vector3d tmpV = Vector3d(character->stance_hip()->parent()->getWorldCoordinates(
                                 character->stance_hip()->parent_joint_position()), pt1);
    Vector3d tmpT = tmpV.crossProductWith(F1);
    _torques[character->stance_hip()->idx()] += tmpT;

    std::ostringstream oss4;
    Vector3d print_vect=tmpT;
    oss4<<"sht "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
    //    std::cout<<oss4.str();


}



void ExternalForceFieldsCompenser::compute_joint_torques_equivalent_to_force(Joint *start, const Point3d &pLocal,
                                                                             const Vector3d &fGlobal, Joint *end)
{
    //starting from the start joint, going towards the end joint, get the origin of each link, in world coordinates,
    //and compute the vector to the global coordinates of pLocal.

    Joint* currentJoint = start;
    Vector3d tmpV;
    Point3d pGlobal = start->child()->getWorldCoordinates(pLocal);

    while (currentJoint != end){
        if (currentJoint == NULL){
            throwError("VirtualModelController::computeJointTorquesEquivalentToForce --> end was not a parent of start...");
        }
        if (currentJoint->controlled()){
            tmpV = Vector3d(currentJoint->parent()->getWorldCoordinates(currentJoint->parent_joint_position()), pGlobal);
            Vector3d tmpT = tmpV.crossProductWith(fGlobal);
            _torques[currentJoint->idx()] -= tmpT;
        }
        currentJoint = currentJoint->parent()->parent_joint();
    }

    //and we just have to do it once more for the end joint, if it's not NULL
    if (end != NULL){
        if (currentJoint->controlled()){
            tmpV = Vector3d(currentJoint->parent()->getWorldCoordinates(currentJoint->parent_joint_position()), pGlobal);
            _torques[currentJoint->idx()] -= tmpV.crossProductWith(fGlobal);
        }
    }
}


void ExternalForceFieldsCompenser::compute_compensation_v2(WaterImpact &resulting_impact)
{

    //*
    std::vector<Vector3d> vect_forces;
    std::vector<Point3d> vect_pt_appli;
    std::vector<int> vect_action_needed;
    std::vector<int> vect_torque_sign;
    vect_forces.resize(_torques.size());
    vect_pt_appli.resize(_torques.size());
    vect_action_needed.resize(_torques.size());
    vect_torque_sign.resize(_torques.size());

    //reset values
    for (int i=0;i<_torques.size();++i){
        _torques2[i]=Vector3d(0,0,0);
        vect_action_needed[i]=false;
        vect_torque_sign[i]=1;
    }

    vect_action_needed[0]=true;

    //we won't do any compensation on the ankles in contact with the ground because it will cause more noise than anything.
    //the reason is that the foot itself lay on the ground so we don't need to compensate it's weight
    //and we can't have the ankle trying to hold the whole body because it may create huge torques which may
    //lead to worse contacts...
    //we do the check for both f the feet because in the case of a flight phase none of the foots are in contact with the ground
    bool swing_foot_contact=false;
    bool stance_foot_contact=false;
    for (int i=0;i<4;++i){
        if ((character->force_stance_foot()[i]).length()>0){
            stance_foot_contact=true;
        }

        if ((character->force_swing_foot()[i]).length()>0){
            swing_foot_contact=true;
        }
    }


    //First I need the leafs of the character
    //as explaned before we do not count the foots if they are in contact with the ground
    std::vector<int> leaf_joints;
    for (int i=0;i<character->getJointCount();++i){
        ArticulatedRigidBody* arb=character->getJoint(i)->child();
        if (arb->child_joints().empty()){
            if (!((arb==character->stance_foot()->child_joints()[0]->child())&&stance_foot_contact)&&
                    !((arb==character->swing_foot()->child_joints()[0]->child())&&swing_foot_contact)){
                leaf_joints.push_back(i);
            }
        }
//        vect_action_needed[arb->parent_joint()->idx()]=true;
    }

    //Now from each leaf up to the root we compute the visible weight
    //the vector store the last joints before the root
    std::vector<int> leafs_result;
    for (int i=0;i<leaf_joints.size();++i){
        Joint* j_prev=NULL;
        Joint* j=character->getJoint(leaf_joints[i]);
        ArticulatedRigidBody* arb=j->child();

        Vector3d previous_force;
        Point3d previous_pt;


        while(j!=NULL){
            vect_action_needed[j->idx()]=true;


            Point3d pt1;
            Vector3d F1;

            if (vect_forces[j->idx()].isZeroVector()){
                pt1=arb->getCMPosition();

                //compute the gravity compensation
                F1 = Vector3d(0, arb->getMass()*9.8, 0);


                throw("the boyancy compensation need to be repaired first");
                ///TODO redo that computation with the new model
                /*
                //compute the boyancy compensation
                if (resulting_impact.find(j->idx())!=resulting_impact.end()){
                    ForceStruct  boyancy = resulting_impact[j->idx()].boyancy;
                    if (!boyancy.F.isZeroVector()){
                        Point3d bpt_w=arb->getWorldCoordinates(boyancy.pt);
                        pt1 = (pt1*F1.length() + bpt_w*boyancy.F.length())/(F1.length()+boyancy.F.length());
                        F1 -= boyancy.F;
                    }
                }
                //*/

                //add the compensation due to the subtree
                pt1 = (pt1*F1.length() + previous_pt*previous_force.length())/(F1.length()+previous_force.length());
                F1 += previous_force;

                //store the new subtree
                previous_force=F1;
                previous_pt= pt1;
            }else{
                //read the values computed by others leaf(s)
                pt1=vect_pt_appli[j->idx()];
                F1 = vect_forces[j->idx()];

                //add the compensation due to the subtree
                pt1 = (pt1*F1.length() + previous_pt*previous_force.length())/(F1.length()+previous_force.length());
                F1 += previous_force;
            }

            vect_forces[j->idx()]=F1;
            vect_pt_appli[j->idx()]=pt1;

            //next joint and arb
            j_prev=j;
            arb=j->parent();
            j=arb->parent_joint();
        }

        if (std::find(leafs_result.begin(),leafs_result.end(),j_prev->idx())==leafs_result.end()){
            leafs_result.push_back(j_prev->idx());
        }
    }

    /*
    for (int i=0; i<leafs_result.size();++i){
        vect_action_needed[leafs_result[i]]=true;
    }
    //*/

    //now we need to compute the forces seen by the subtrees linked to the feets
    double stanceHipToSwingHipRatio;
    if (stance_foot_contact||swing_foot_contact){
        if (stance_foot_contact&&swing_foot_contact){
            stanceHipToSwingHipRatio=character->get_stance_foot_weight_ratio();
        }else{
            if (stance_foot_contact){
                stanceHipToSwingHipRatio=1;
            }else{
                stanceHipToSwingHipRatio=0;
            }
        }


        //we compute the force seen at the root (with the root weight counted)
        Vector3d F_all;
        Point3d pt_all;

        for (int i=0; i<leafs_result.size();++i){

            Vector3d F=vect_forces[leafs_result[i]];
            Point3d pt=vect_pt_appli[leafs_result[i]];

            pt_all = (pt_all*F_all.length() + pt*F.length())/(F_all.length()+F.length());
            F_all += F;
        }
        //add the root weight
        {
            Vector3d F= Vector3d(0, character->getRoot()->getMass()*9.8, 0);
            Point3d pt=character->getRoot()->getCMPosition();

            pt_all = (pt_all*F_all.length() + pt*F.length())/(F_all.length()+F.length());
            F_all += F;
        }



        if (stance_foot_contact){
            //we desactivate the compensation on the ankle
            vect_action_needed[character->stance_foot()->parent_joint()->idx()]=false;

            //and activate the one on the hip and the knee
            vect_action_needed[character->stance_foot()->parent_joint()->parent()->parent_joint()->idx()]=true;
            vect_action_needed[character->stance_hip()->idx()]=true;

            Vector3d F=F_all*stanceHipToSwingHipRatio;
            Point3d pt=pt_all;

            Joint* j=character->stance_hip();
            vect_forces[j->idx()]=F;
            vect_pt_appli[j->idx()]=pt;
            vect_torque_sign[j->idx()]=-1;


            //now compute the compensation on the knee
            ArticulatedRigidBody* arb=j->child();
            Point3d pt1=arb->getCMPosition();

            //compute the gravity compensation
            Vector3d F1 = Vector3d(0, arb->getMass()*9.8, 0);


            throw("the boyancy compensation need to be repaired first");
            ///TODO redo that computation with the new model
            /*
            //compute the boyancy compensation
            //we keep j as the hip since we want the water impact on the upper leg
            //to be compensated by the knee
            if (resulting_impact.find(j->idx())!=resulting_impact.end()){
                ForceStruct  boyancy = resulting_impact[j->idx()].boyancy;
                if (!boyancy.F.isZeroVector()){
                    Point3d bpt_w=arb->getWorldCoordinates(boyancy.pt);
                    pt1 = (pt1*F1.length() + bpt_w*boyancy.F.length())/(F1.length()+boyancy.F.length());
                    F1 -= boyancy.F;
                }
            }
            //*/

            //add the compensation due to the rest of the body
            pt1 = (pt1*F1.length() + pt*F.length())/(F1.length()+F.length());
            F1 += F;

            j=arb->child_joints()[0];

            vect_forces[j->idx()]=F1;
            vect_pt_appli[j->idx()]=pt1;
            vect_torque_sign[j->idx()]=-1;
        }

        if (swing_foot_contact){
            //we desactivate the compensation on the ankle
            vect_action_needed[character->swing_foot()->parent_joint()->idx()]=false;

            //and activate the one on the hip and the knee
            vect_action_needed[character->swing_foot()->parent_joint()->parent()->parent_joint()->idx()]=true;
            vect_action_needed[character->swing_hip()->idx()]=true;

            Vector3d F=F_all*(1-stanceHipToSwingHipRatio);
            Point3d pt=pt_all;

            Joint* j=character->swing_hip();
            vect_forces[j->idx()]=F;
            vect_pt_appli[j->idx()]=pt;
            vect_torque_sign[j->idx()]=-1;


            //now compute the compensation on the knee
            ArticulatedRigidBody* arb=j->child();
            Point3d pt1=arb->getCMPosition();

            //compute the gravity compensation
            Vector3d F1 = Vector3d(0, arb->getMass()*9.8, 0);


            throw("the boyancy compensation need to be repaired first");
            ///TODO redo that computation with the new model
            /*
            //compute the boyancy compensation
            //we keep j as the hip since we want the water impact on the upper leg
            //to be compensated by the knee
            if (resulting_impact.find(j->idx())!=resulting_impact.end()){
                ForceStruct  boyancy = resulting_impact[j->idx()].boyancy;
                if (!boyancy.F.isZeroVector()){
                    Point3d bpt_w=arb->getWorldCoordinates(boyancy.pt);
                    pt1 = (pt1*F1.length() + bpt_w*boyancy.F.length())/(F1.length()+boyancy.F.length());
                    F1 -= boyancy.F;
                }
            }
            //*/

            //add the compensation due to the rest of the body
            pt1 = (pt1*F1.length() + pt*F.length())/(F1.length()+F.length());
            F1 += F;

            j=arb->child_joints()[0];

            vect_forces[j->idx()]=F1;
            vect_pt_appli[j->idx()]=pt1;
            vect_torque_sign[j->idx()]=-1;
        }
    }


    //now that all the forces have been computed we can finaly evaluate the compensation torques
    for (int i=0;i<character->getJointCount();++i){
        Joint* j=character->getJoint(i);
        if (vect_action_needed[j->idx()]){
            Point3d pGlobal=vect_pt_appli[j->idx()];
            Vector3d fGlobal=vect_forces[j->idx()]*1;

            Vector3d tmpV = Vector3d(j->parent()->getWorldCoordinates(j->parent_joint_position()), pGlobal);
            Vector3d tmpT = tmpV.crossProductWith(fGlobal)*-1;
            _torques2[j->idx()] = tmpT*vect_torque_sign[j->idx()];
        }else{
//            std::cout<<j->name();
        }
    }
//*/
}
