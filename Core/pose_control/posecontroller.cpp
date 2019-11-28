#include "posecontroller.h"

#include <algorithm>

#include "Core/Character.h"
#include "Core/pose_control/posecontrolglobals.h"
#include "Core/pose_control/swingfootcontroller.h"
#include "Core/velocity_control/velocitycontroller.h"
#include "Core/ConUtils.h"

#include "Core/SimBiConState.h"
#include "Globals.h"

#include <iostream>
#include <sstream>
#include <iomanip>

PoseController::PoseController(Character *c)
{
    //old static members
    old_phase=2.0;

    //all those members are specials
    //they are old static variables in functions but I could not find what was the problem with them
    //so I put them here so I'm ensured they are reinitialized between each evaluations ...
    //PoseController::init_character_step
    velDsag_old=-125;
    liquid_lvl_old=-215;

    //PoseController::preprocess_simulation_step
    old_heading=0;
    target_heading=0;
    desired_heading_pelvis=0;
    desired_heading_swing_foot=0;
    start_phi_change_heading=0.0f;
    target_heading_mitigation_coefficient=1.0;
    first_pass_this_char_step=true;

    //PoseController::simulation_step
    old_swing_foot_is_relative=false;
    target_heading_stance_foot=Quaternion::getRotationQuaternion(0,SimGlobals::up);

    //PoseController::simulation_step
    start_phi_change_swing_leg_heading=0.0f;

    //PoseController::set_swing_leg_rotation_angle
    old_val=-102;
    trigerred_once=false;

    //other  operations

    character=c;

    _torques.resize(character->getJointCount());
    controlParams.resize(character->getJointCount());

    set_fsm_state(-1);

    stanceHipDamping = -1;
    stanceHipMaxVelocity = 4;
    rootPredictiveTorqueScale = 0;

    startingState = -1;
    startingStance = LEFT_STANCE;
    initialBipState[0] = '\0';

    set_stance(LEFT_STANCE);

    //copy the current state of the character into the desired pose - makes sure that it's the correct size
    character->getState(&PoseControlGlobals::desired_pose);

    //set initial values
    ReducedCharacterState rs(&PoseControlGlobals::desired_pose);
    rs.setPosition(Vector3d());
    rs.setVelocity(Vector3d());
    rs.setOrientation(Quaternion(1, 0, 0, 0));
    rs.setAngularVelocity(Vector3d());

    for (int i=0;i<character->getJointCount();i++){
        rs.setJointRelativeAngVelocity(Vector3d(), i);
        rs.setJointRelativeOrientation(Quaternion(), i);
    }
    //modules
    swing_foot_controller= new SwingFootController(character);
}

PoseController::~PoseController()
{
    delete swing_foot_controller;

    _torques.clear();
    controlParams.clear();

    for (int i=0;i<(int)_states.size();++i){
        delete _states[i];
        _states[i]=NULL;
    }
    _states.clear();

    PoseControlGlobals::desired_pose.clear();
    
    
}

void PoseController::restart()
{
    swing_foot_controller->restart();
}

void PoseController::init_character_step(VelocityController *vel_control)
{
    target_heading_mitigation_coefficient=1.0;

    if(velDsag_old!=SimGlobals::velDSagittal&&liquid_lvl_old!=SimGlobals::water_level){
        state(_state_idx)->update_joints_trajectory();
    }
    velDsag_old=SimGlobals::velDSagittal;
    liquid_lvl_old=SimGlobals::water_level;




    //init the swing foot controller
    if (vel_control!=NULL){
        swing_foot_controller->init_new_character_step(vel_control->need_recovery_steps(),state_time(),vel_control);
    }else{
        swing_foot_controller->init_new_character_step(false,state_time(),NULL);
    }

    //now we need to update the joints that are controlled
    //to make it simple let's say everything is controlled except the stance hip
    //(which is true actually fot the biped)
    for (int i = 0; i<character->getJointCount(); i++){
        controlParams[i].controlled = true;
        controlParams[character->stance_hip()->idx()].controlled=false;


        Joint* j=character->getJoint(i);
        j->control_params=controlParams[i];
    }

    //I store the orientation of the stance foot on contact because it will be my
    //target orientation as ;long as the only contact points are inside the heel.
    target_heading_stance_foot=computeHeading(character->stance_foot()->getOrientation());
    //this is used as a static var for the stance foot orientation
    //2.0 is to make sure it's higher than the previous value
    old_phase=2.0;
    /*
    for (int i = 0; i<character->getJointCount(); i++){
        controlParams[i].controlled = true;
        controlParams[i].relative_to_char_frame = false;
        std::string j_name=character->getJoint(i)->name();
        if (j_name == "rElbow" || j_name == "lElbow" || j_name == "rShoulder" || j_name == "lShoulder"){
            //controlParams[i].controlled = false;
            //controlParams[i].relative_to_char_frame = true;
        }
        character->getJoint(i)->controlled(controlParams[i].controlled);
    }
    //*/

    //at the start of the step is the character has not reached the target yet we fix the target to the current orientation
    if (desired_heading_pelvis!=SimGlobals::desiredHeading){
        desired_heading_pelvis=character->getHeadingAngle();
    }
    first_pass_this_char_step=true;
}

void PoseController::preprocess_simulation_step(double phase, Vector3d v_com)
{
    for (int i=0;i<character->getJointCount();++i){
        character->getJoint(i)->controlled(is_controlled(i));
    }


    swing_foot_controller->preprocess_simulation_step(phase,v_com);

    //hande the possible modifications of the desired heading
    control_desired_heading(phase);

}


void PoseController::simulation_step(double phase)
{
    //*
    SimGlobals::desiredHeading_active=desired_heading_pelvis;
    Quaternion desired_swing_foot_heading_quat=Quaternion::getRotationQuaternion(desired_heading_swing_foot, SimGlobals::up);
    Quaternion desired_pelvis_heading_quat=Quaternion::getRotationQuaternion(desired_heading_pelvis, SimGlobals::up);
    SimBiConState* curState=_states[_state_idx];
    //set the relatives to char frame
    for (int i = 0; i<curState->getTrajectoryCount(); i++){
        int jIndex = curState->getTrajectory(i)->getJointIndex(character->stance());

        //anything loer than -1 is just a trajectory stored here for a simple usage
        //but it's not a real tajectory so I have to ignore them
        if (jIndex < -1){
            continue;
        }

        if (curState->getTrajectory(i)->relative_to_char_frame){
            controlParams[jIndex].relative_to_char_frame = true;
            //            controlParams[jIndex].char_frame = character->getHeading();
            controlParams[jIndex].char_frame = desired_pelvis_heading_quat;
        }
    }

    ///TODO check impact of this
    ///Normaly should not be done since there is no y axis on the ankle...
    if (false){
        if (swing_foot_controller->ipm_needed()){
            controlParams[character->swing_foot()->parent_joint()->idx()].relative_to_char_frame = true;
            controlParams[character->swing_foot()->parent_joint()->idx()].char_frame=desired_swing_foot_heading_quat;
            controlParams[character->swing_foot()->child_joints().front()->idx()].char_frame=desired_swing_foot_heading_quat;
        }else{
            controlParams[character->swing_foot()->parent_joint()->idx()].relative_to_char_frame = false;
        }
    }

    //stance foot orientation specification
    controlParams[character->stance_foot()->parent_joint()->idx()].relative_to_char_frame = true;
    if (SimGlobals::foot_flat_on_ground){
        //when the foot touch the ground we store the orientation of the foot to maintain it after
        if (old_phase>phase){
            target_heading_stance_foot=computeHeading(character->stance_foot()->getOrientation());
        }
        old_phase=phase;
    }else{
        //the orientation is noted at the start of the new character step
    }
    controlParams[character->stance_foot()->parent_joint()->idx()].char_frame=target_heading_stance_foot;
    controlParams[character->stance_foot()->child_joints().front()->idx()].char_frame=target_heading_stance_foot;




    //reset the desired pose
    //always start from a neutral desired pose, and build from there...
    ReducedCharacterState rs(&PoseControlGlobals::desired_pose);
    for (int i = 0; i<character->getJointCount(); i++){
        rs.setJointRelativeOrientation(Quaternion(1, 0, 0, 0), i);
        rs.setJointRelativeAngVelocity(Vector3d(), i);
    }
    //reset the torques result pose
    for (int i=0;i<_torques.size();++i){
        _torques[i]=Vector3d(0,0,0);
    }

    //we compute the basic target
    evaluate_joints_target(phase);

    for (int i=0; i<(int)PoseControlGlobals::desired_pose.size();++i){
        if (PoseControlGlobals::desired_pose[i]!=PoseControlGlobals::desired_pose[i]){
            std::cout<<"PoseController::simulation_step  undefinned target befre ipm ";
        }
    }




    //compute the swing foot target
    swing_foot_controller->simulation_step(desired_pelvis_heading_quat);

    for (int i=0; i<(int)PoseControlGlobals::desired_pose.size();++i){
        if (PoseControlGlobals::desired_pose[i]!=PoseControlGlobals::desired_pose[i]){
            std::cout<<"PoseController::simulation_step  undefinned target after ipm ";
        }
    }

    if (Globals::use_hand_position_tracking){
        track_hand_target();
    }

    //now we have to modify the swing hip target so that we can obtain de desired direction on the swing foot
    if (controlParams[character->swing_foot()->parent_joint()->idx()].relative_to_char_frame){
        //since there is no y axis on the ankle
        //the whole roation must be supported by the hip.
        Quaternion q=Quaternion::getRotationQuaternion(desired_heading_swing_foot,SimGlobals::up);
        set_leg_forward_to_orientation(q,true,1.0);
    }

    //*
    if (false){
        //I now control the orientation directly by looking at the fot orientation
        //so I don't need that anymore
        if (Globals::use_stance_leg_orientation_controller){
            if (!SimGlobals::foot_flat_on_ground)
            {
                controlParams[character->stance_hip()->idx()].relative_to_char_frame=false;


                double root_angle=computeHeading(rs.getOrientation()).getRotationAngle(SimGlobals::up);
                double foot_angle=computeHeading(controlParams[character->stance_foot()->parent_joint()->idx()].char_frame).getRotationAngle(SimGlobals::up);

                Quaternion q=Quaternion::getRotationQuaternion(root_angle+(foot_angle-root_angle)*0.5,SimGlobals::up);


                set_leg_forward_to_orientation(q,false,1.0);
            }
        }
    }
    //*/

    /*
    Quaternion quat=computeHeading(controlParams[character->stance_foot()->parent_joint()->idx()].char_frame*
            rs.getJointRelativeOrientation(character->stance_foot()->parent_joint()->idx()));
    double angle=quat.getRotationAngle(Vector3d(0,1,0));
    angle=0;
    //*/

    for (int i=0; i<(int)PoseControlGlobals::desired_pose.size();++i){
        if (PoseControlGlobals::desired_pose[i]!=PoseControlGlobals::desired_pose[i]){
            std::cout<<"PoseController::simulation_step  after reorienting leg  ";
        }
    }

    //compute the torques now, using the desired pose information
    compute_torques();


    //bubble-up the torques computed from the PD controllers
    bubbleup_torques();
}

int PoseController::end_simulation_step(double phi)
{

    Vector3d swing_foot_force=character->get_force_on_foot(true, false);
    Vector3d stance_foot_force=character->get_force_on_foot(false, false);

    if (_states[_state_idx]->needTransition(phi, fabs(swing_foot_force.y), fabs(stance_foot_force.y))){
        if (SimGlobals::state_duration_modified){
            _states[_state_idx]->setStateTime(SimGlobals::requested_state_duration);
            SimGlobals::state_duration_modified=false;
        }


        int newStateIndex = _states[_state_idx]->getNextStateIndex();
        transition_to_state(newStateIndex);
        //here _state_idx is already equals to the new state idx so I should use the member attribute
        SimGlobals::requested_state_duration=_states[_state_idx]->getStateTime();
        return newStateIndex;
    }
    return -1;
}

#include "Core/pose_control/twolinkik.h"
void PoseController::track_hand_target()
{
    Point3d target_pt;
    target_pt=character->getRoot()->getCMPosition();
    target_pt.y=0;
    Vector3d target_offset=Vector3d(0,1.4,0.18);
    double coronal_offset=0.17;

    Quaternion desired_heading_quat=Quaternion::getRotationQuaternion(desired_heading_pelvis,SimGlobals::up);
    //for the left shoulder
    {
        Vector3d left_target_offset=target_offset;
        left_target_offset.x+=coronal_offset;
        left_target_offset=desired_heading_quat.rotate(left_target_offset);
        Point3d left_target_pt=target_pt+left_target_offset;
        /*
        std::cout<<"left_target_pt: "<<left_target_pt.x<<" "<<left_target_pt.y<<" "<<left_target_pt.z<<std::endl;
        ArticulatedRigidBody* arb=character->getARBByName("lLowerarm");
        Point3d hand_pos=arb->getCMPosition()+arb->getWorldCoordinates(Vector3d(0.15,0,0));
        std::cout<<"left_target_pt: "<<hand_pos.x<<" "<<hand_pos.y<<" "<<hand_pos.z<<std::endl;
        //*/
        //this is the joint between the grandparent RB and the parent
        Joint* parentJoint = character->getJointByName("lShoulder");
        Joint* interJoint = parentJoint->child()->child_joints().front();
        Point3d end_point =Point3d(0.15,0,0);
        //this is the grandparent - most calculations will be done in its coordinate frame
        ArticulatedRigidBody* gParent = parentJoint->parent();
        //this is the reduced character space where we will be setting the desired orientations and ang vels.
        ReducedCharacterState rs(&PoseControlGlobals::desired_pose);
        //the desired relative orientation between parent and grandparent
        Quaternion qParent;
        //and the desired relative orientation between child and parent
        Quaternion qChild;


        //*
        Vector3d parentAxis(parentJoint->child_joint_position(), interJoint->parent_joint_position());
        Vector3d childAxis(interJoint->child_joint_position(), end_point);

        Vector3d parent_rotation_axis= Vector3d(1,1,0);
        parent_rotation_axis=desired_heading_quat.rotate(parent_rotation_axis);

        TwoLinkIK::getIKOrientations(parentJoint->parent_joint_position(), gParent->getLocalCoordinates(left_target_pt),
                                     gParent->getLocalCoordinates(parent_rotation_axis),
                                     parentAxis,Vector3d(0, 1, 0), childAxis, &qParent, &qChild);


        rs.setJointRelativeOrientation(qChild, interJoint->idx());
        rs.setJointRelativeOrientation(qParent, parentJoint->idx());


        Vector3d wParentD(0, 0, 0);
        Vector3d wChildD(0, 0, 0);
        rs.setJointRelativeAngVelocity(wChildD, interJoint->idx());
        rs.setJointRelativeAngVelocity(wParentD, parentJoint->idx());
        /*
        Quaternion q=Quaternion::getRotationQuaternion(0,Vector3d(0,1,0));
        controlParams[interJoint->idx()].relative_to_char_frame = true;
        controlParams[interJoint->idx()].char_frame = q;
        controlParams[parentJoint->idx()].relative_to_char_frame = true;
        controlParams[parentJoint->idx()].char_frame = q;
        //*/
    }

    //for the right shoulder
    {
        Vector3d right_target_offset=target_offset;
        right_target_offset.x+=-coronal_offset;
        right_target_offset=Quaternion::getRotationQuaternion(desired_heading_pelvis,SimGlobals::up).rotate(right_target_offset);
        Point3d right_target_pt=target_pt+right_target_offset;

        //this is the joint between the grandparent RB and the parent
        Joint* parentJoint = character->getJointByName("rShoulder");
        Joint* interJoint = parentJoint->child()->child_joints().front();
        Point3d end_point =Point3d(-0.15,0,0);
        //this is the grandparent - most calculations will be done in its coordinate frame
        ArticulatedRigidBody* gParent = parentJoint->parent();
        //this is the reduced character space where we will be setting the desired orientations and ang vels.
        ReducedCharacterState rs(&PoseControlGlobals::desired_pose);
        //the desired relative orientation between parent and grandparent
        Quaternion qParent;
        //and the desired relative orientation between child and parent
        Quaternion qChild;


        //*
        Vector3d parentAxis(parentJoint->child_joint_position(), interJoint->parent_joint_position());
        Vector3d childAxis(interJoint->child_joint_position(), end_point);

        Vector3d parent_rotation_axis= Vector3d(1,-1,0);
        parent_rotation_axis=desired_heading_quat.rotate(parent_rotation_axis);

        TwoLinkIK::getIKOrientations(parentJoint->parent_joint_position(), gParent->getLocalCoordinates(right_target_pt),
                                     gParent->getLocalCoordinates(parent_rotation_axis),
                                     parentAxis,Vector3d(0, -1, 0), childAxis, &qParent, &qChild);



        rs.setJointRelativeOrientation(qChild, interJoint->idx());
        rs.setJointRelativeOrientation(qParent, parentJoint->idx());


        Vector3d wParentD(0, 0, 0);
        Vector3d wChildD(0, 0, 0);
        rs.setJointRelativeAngVelocity(wChildD, interJoint->idx());
        rs.setJointRelativeAngVelocity(wParentD, parentJoint->idx());
        /*
        Quaternion q=Quaternion::getRotationQuaternion(0,Vector3d(0,1,0));
        controlParams[interJoint->idx()].relative_to_char_frame = true;
        controlParams[interJoint->idx()].char_frame = q;
        controlParams[parentJoint->idx()].relative_to_char_frame = true;
        controlParams[parentJoint->idx()].char_frame = q;
        //*/
    }
}

void PoseController::load_from_file(char *fName)
{
    Globals::current_controller_file = std::string(fName);


    if (fName == NULL)
        throwError("NULL file name provided.");
    FILE *f = fopen(fName, "r");
    if (f == NULL)
        throwError("PoseController::load_from_file:Could not open file: %s", fName);

    //to be able to load multiple controllers from multiple files,
    //we will use this offset to make sure that the state numbers
    //mentioned in each input file are updated correctly
    int stateOffset = _states.size();
    SimBiConState* tempState;
    int tempStateNr = -1;

    //have a temporary buffer used to read the file line by line...
    char buffer[200];
    //this is where it happens.

    while (!feof(f)){
        //get a line from the file...
        fgets(buffer, 200, f);
        if (feof(f))
            break;
        if (strlen(buffer)>195)
            throwError("The input file contains a line that is longer than ~200 characters - not allowed");
        char *line = lTrim(buffer);
        int lineType = getConLineType(line);

        std::string path;
        char effective_path[256];
        switch (lineType) {
            case CON_PD_GAINS_START:
                read_gains(f);
                //save a backup of the gains
                controlParams_initial=controlParams;

                break;
            case CON_STATE_START:
                tempState = new SimBiConState();
                sscanf(line, "%d", &tempStateNr);
                if (tempStateNr != stateOffset + _states.size())
                    throwError("Inccorect state offset specified: %d", tempStateNr);
                _states.push_back(tempState);
                tempState->readState(f, stateOffset);
                //now we have to resolve all the joint names (i.e. figure out which joints they apply to).
                resolve_joints(tempState);

                //*
                //now we can fill the implicit components for the trajectories
                for (int i=0;i<tempState->getTrajectoryCount();++i){
                    for(int stance=0;stance<2;++stance){
                        int j_id=tempState->getTrajectory(i)->getJointIndex(stance);
                        Joint* joint=character->getJoint(j_id);

                        //generate the list of possible components
                        std::vector<Vector3d> list_possib_components;
                        if (joint!=NULL){
                            switch (joint->get_joint_type()){
                                case  HINGE_JOINT :{
                                    HingeJoint* temp_j=dynamic_cast<HingeJoint*>(joint);
                                    list_possib_components.push_back(temp_j->axis());
                                    break;
                                }
                                case  BALL_IN_SOCKET_JOINT :{
                                    BallInSocketJoint* temp_j=dynamic_cast<BallInSocketJoint*>(joint);
                                    list_possib_components.push_back(temp_j->get_swingAxis1());
                                    list_possib_components.push_back(temp_j->get_swingAxis2());
                                    list_possib_components.push_back(temp_j->get_twistAxis());
                                    break;
                                }
                                case  UNIVERSAL_JOINT :{
                                    UniversalJoint* temp_j=dynamic_cast<UniversalJoint*>(joint);
                                    list_possib_components.push_back(temp_j->axis_a());
                                    list_possib_components.push_back(temp_j->axis_b());
                                    break;
                                }
                                default:
                                    break;
                            }
                        }else{
                            //I have to handle the root by itself
                            if (j_id=-1){
                                list_possib_components.push_back(Vector3d(1,0,0));
                                list_possib_components.push_back(Vector3d(0,1,0));
                                list_possib_components.push_back(Vector3d(0,0,1));
                            }
                        }

                        //and now create the implicit components
                        tempState->getTrajectory(i)->create_implicit_components(list_possib_components);
                    }
                }
                //*/
                break;
            case CON_STANCE_HIP_DAMPING:
                sscanf(line, "%lf", &stanceHipDamping);
                break;
            case CON_STANCE_HIP_MAX_VELOCITY:
                sscanf(line, "%lf", &stanceHipMaxVelocity);
                break;
            case CON_ROOT_PRED_TORQUE_SCALE:
                sscanf(line, "%lf", &rootPredictiveTorqueScale);
                break;
            case CON_CHARACTER_STATE:
                //first we must correct the path
                path = std::string(line);
                path = interpret_path(path);
                //and now we can use it
                strcpy(effective_path, path.c_str());

                character->loadReducedStateFromFile(effective_path);
                strcpy(initialBipState, trim(line));
                break;
            case CON_START_AT_STATE:
                if (sscanf(line, "%d", &tempStateNr) != 1)
                    throwError("A starting state must be specified!");
                transition_to_state(tempStateNr);
                startingState = tempStateNr;
                break;
            case CON_COMMENT:
                break;
            case CON_STARTING_STANCE:
                if (strncmp(trim(line), "left", 4) == 0){
                    set_stance(LEFT_STANCE);
                    startingStance = LEFT_STANCE;
                }
                else if (strncmp(trim(line), "right", 5) == 0){
                    set_stance(RIGHT_STANCE);
                    startingStance = RIGHT_STANCE;
                }
                else
                    throwError("When using the \'reverseTargetOnStance\' keyword, \'left\' or \'right\' must be specified!");
                break;
            case CON_NOT_IMPORTANT:
                std::cerr<<"Ignoring input line: "<<line<<std::endl;
                break;
            case CON_IPM_ALTERATION_EFFECTIVENESS:{
                double buff;
                sscanf(line, "%lf", &buff);
                SimGlobals::ipm_alteration_effectiveness = buff;
            }
                break;
            case CON_VIRTUAL_FORCE_EFFECTIVENESS:{
                double buff;
                sscanf(line, "%lf", &buff);
                SimGlobals::virtual_force_effectiveness= buff;
            }
                break;
            default:
                throwError("Incorrect SIMBICON input file: \'%s\' - unexpected line.", buffer);
        }
    }
    fclose(f);

    init_character_step(NULL);

    //now init the sub componennts
    //PoseControlGlobals::desired_pose= desiredPose;
}

void PoseController::read_gains(char* fName){
    FILE* f = fopen(fName, "r");
    if (f == NULL)
        return;

    read_gains(f);
    fclose(f);
}

void PoseController::read_gains(std::string input_file){
    read_gains(const_cast<char*>(input_file.c_str()));
}

void PoseController::read_gains(FILE* f){
    if (f == NULL)
        throwError("File pointer is NULL - cannot read gain coefficients!!");

    //have a temporary buffer used to read the file line by line...
    char buffer[200];

    //this is where it happens.
    while (!feof(f)){
        //get a line from the file...
        fgets(buffer, 200, f);
        if (strlen(buffer)>195)
            throwError("The input file contains a line that is longer than ~200 characters - not allowed");
        char *line = lTrim(buffer);
        int lineType = getConLineType(line);
        switch (lineType) {
            case CON_PD_GAINS_END:
                return;
                break;
            case CON_PD_GAINS_START:
                break;
            case CON_COMMENT:
                break;
            default:
                parse_gain_line(line);
                break;
        }
    }
    throwError("Incorrect controller input file: No \'/KpKdMaxT\' found", buffer);
}

void PoseController::write_gains(FILE* f){

    for( uint jIndex=0; jIndex < controlParams.size(); ++jIndex ) {

        Joint* joint = character->getJoint(jIndex);
        if( !joint ) continue;

        fprintf( f, "    %s\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
                 joint->name().c_str(),
                 controlParams[jIndex].kp,
                 controlParams[jIndex].kd,
                 controlParams[jIndex].max_abs_torque,
                 controlParams[jIndex].scale.x,
                 controlParams[jIndex].scale.y,
                 controlParams[jIndex].scale.z );

    }

}

void PoseController::writeToFile(FILE *f, char *stateFileName)
{
    fprintf( f, "%s\n", getConLineString(CON_PD_GAINS_START) );
    fprintf( f, "#        joint name              Kp      Kd      MaxTorque    ScaleX        ScaleY        ScaleZ\n" );
    fprintf( f, "    root\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
             rootControlParams.kp,
             rootControlParams.kd,
             rootControlParams.max_abs_torque,
             rootControlParams.scale.x,
             rootControlParams.scale.y,
             rootControlParams.scale.z );
    write_gains(f);

    fprintf( f, "%s\n", getConLineString(CON_PD_GAINS_END) );

    fprintf( f, "\n" );

    if( stanceHipDamping > 0 ) {
        fprintf( f, "%s %lf\n", getConLineString(CON_STANCE_HIP_DAMPING), stanceHipDamping );
        fprintf( f, "%s %lf\n", getConLineString(CON_STANCE_HIP_MAX_VELOCITY), stanceHipMaxVelocity );
    }

    fprintf( f, "\n" );


    for( uint i=0; i<_states.size(); ++i ) {

        fprintf( f, "\n\n" );
        _states[i]->writeState( f, i );

    }

    fprintf( f, "\n\n" );

    fprintf( f, "%s %d\n", getConLineString(CON_START_AT_STATE), startingState );
    fprintf( f, "%s %s\n", getConLineString(CON_STARTING_STANCE),(character->stance() == 0)?"left":"right" );

    if (stateFileName == NULL){
        fprintf(f, "%s %s\n", getConLineString(CON_CHARACTER_STATE), initialBipState);
    }
    else{
        fprintf(f, "%s %s\n", getConLineString(CON_CHARACTER_STATE), stateFileName);
    }

    //at the end I now add the speed control system alteration values
    fprintf(f, "%s %lf\n", getConLineString(CON_IPM_ALTERATION_EFFECTIVENESS), SimGlobals::ipm_alteration_effectiveness);
    fprintf(f, "%s %lf\n", getConLineString(CON_VIRTUAL_FORCE_EFFECTIVENESS), SimGlobals::virtual_force_effectiveness);

}

void PoseController::parse_gain_line(char *line)
{
    double kp, kd, tMax, scX, scY, scZ;
    char jName[100];
    int jIndex;
    int nrParams = 0;
    nrParams = sscanf(line, "%s %lf %lf %lf %lf %lf %lf\n", jName, &kp, &kd, &tMax, &scX, &scY, &scZ);
    if (nrParams == 2){
        Vector3d tmp=character->getJointByName(jName)->child()->getPMI();
        double maxM = std::max(tmp.x, tmp.y);
        maxM = std::max(maxM, tmp.z);
        kd = kp/10;
        tMax = 10000;
        scX = tmp.x/maxM;
        scY = tmp.y/maxM;
        scZ = tmp.z/maxM;
    }else
        if (nrParams!=7){
            throwError("To specify the gains, you need: 'joint name Kp Kd Tmax scaleX scaleY scaleZ'! --> \'%s\'", line);
        }

    if (strcmp(jName, "root") == 0){
        rootControlParams.kp = kp;
        rootControlParams.kd = kd;
        rootControlParams.max_abs_torque= tMax;
        rootControlParams.scale = Vector3d(scX, scY, scZ);
    }else{
        jIndex = character->getJointIndex(jName);
        if (jIndex < 0)
            throwError("Cannot find joint: \'%s\'", jName);
        controlParams[jIndex].kp = kp;
        controlParams[jIndex].kd = kd;
        controlParams[jIndex].max_abs_torque = tMax;
        controlParams[jIndex].scale = Vector3d(scX, scY, scZ);
    }
}

void PoseController::resolve_joints(SimBiConState *state)
{
    char tmpName[100];
    for (uint i=0;i<state->getTrajectoryCount();i++){
        Trajectory* jt = state->getTrajectory(i);
        //deal with the 'root' special case
        if (strcmp(jt->jName, "root") == 0){
            jt->leftStanceIndex = jt->rightStanceIndex = -1;
            continue;
        }
        //deal with the new swing foot trajectory specification
        if (strcmp(jt->jName, "swing_foot") == 0){
            jt->leftStanceIndex = jt->rightStanceIndex = -2;

            //we save a pointer for easy access
            swing_foot_controller->user_swing_foot_traj(jt);

            continue;
        }

        //deal with the velD trajectory specification
        if (strcmp(jt->jName, "velD") == 0){
            jt->leftStanceIndex = jt->rightStanceIndex = -3;
            velD_traj=jt;

            //we have some work to do.
            //indeed it is possible that the points do not correspond to real phi that will be reached
            //so we need to modify them to make surre that they are.
            double d_phi=SimGlobals::dt/state->getStateTime();


            for (int k=0;k<velD_traj->components.size();++k){
                TrajectoryComponent* c=velD_traj->components[k];

                if (c->is_implicit()){
                    continue;
                }

                if (c->baseTraj.getKnotPosition(0)>-0.5){

                    // first phi=0 will be sometime not reached so I'll set the first position to phi=dt/_states[_state_idx]->getStateTime()
                    if (c->baseTraj.getKnotPosition(0)<0){
                        c->baseTraj.setKnotPosition(0,0);
                    }
                    //also we remove any other point that would be lower than this value
                    for (int i=1;i<c->baseTraj.getKnotCount();++i){
                        if (c->baseTraj.getKnotPosition(i)<(d_phi*0.5)){
                            c->baseTraj.removeKnot(i);
                            --i;
                        }else{
                            break;
                        }
                    }

                    //now we ajust avery point in the trajectory to the nearest reached phi value
                    for (int i=0;i<c->baseTraj.getKnotCount();++i){
                        int nb_phi_iter=static_cast<int>(c->baseTraj.getKnotPosition(i)/d_phi);
                        double phi_error=c->baseTraj.getKnotPosition(i)-nb_phi_iter*d_phi;
                        if (std::abs(phi_error)>d_phi/2.0){
                            nb_phi_iter+=1;
                        }
                        c->baseTraj.setKnotPosition(i,nb_phi_iter*d_phi);
                    }
                }else{
                    //in that case we need to generate it
                    double zero_val=0;
                    if (c->rotationAxis.z!=0){
                        zero_val=1;
                    }
                    int nb_points=c->baseTraj.getKnotValue(0);
                    c->baseTraj.clear();

                    //ad the start and the end
                    double start_value=0;
                    c->baseTraj.addKnot(start_value,zero_val);
                    double end_value=(static_cast<int>(1.0/d_phi))*d_phi;
                    c->baseTraj.addKnot(end_value,zero_val);

                    double range=c->baseTraj.getKnotPosition(1)-c->baseTraj.getKnotPosition(0);

                    int timesteps_betwen_values=(range/nb_points)/d_phi;
                    if (timesteps_betwen_values<1){
                        timesteps_betwen_values=1;
                    }

                    double current_value=start_value+d_phi*timesteps_betwen_values;
                    while (current_value<end_value){
                        c->baseTraj.addKnot(current_value,zero_val);
                        current_value+=d_phi*timesteps_betwen_values;
                    }
                }
            }

            continue;
        }

        //deal with the SWING_XXX' case
        if (strncmp(jt->jName, "SWING_", strlen("SWING_"))==0){
            strcpy(tmpName+1, jt->jName + strlen("SWING_"));
            tmpName[0] = 'r';
            jt->leftStanceIndex = character->getJointIndex(tmpName);
            if (jt->leftStanceIndex<0)
                throwError("Cannot find joint %s\n", tmpName);
            tmpName[0] = 'l';
            jt->rightStanceIndex = character->getJointIndex(tmpName);
            if (jt->rightStanceIndex<0)
                throwError("Cannot find joint %s\n", tmpName);
            continue;
        }
        //deal with the STANCE_XXX' case
        if (strncmp(jt->jName, "STANCE_", strlen("STANCE_"))==0){
            strcpy(tmpName+1, jt->jName + strlen("STANCE_"));
            tmpName[0] = 'l';
            jt->leftStanceIndex = character->getJointIndex(tmpName);
            if (jt->leftStanceIndex<0)
                throwError("Cannot find joint %s\n", tmpName);
            tmpName[0] = 'r';
            jt->rightStanceIndex = character->getJointIndex(tmpName);
            if (jt->rightStanceIndex<0)
                throwError("Cannot find joint %s\n", tmpName);
            continue;
        }
        //if we get here, it means it is just the name...
        jt->leftStanceIndex = character->getJointIndex(jt->jName);
        if (jt->leftStanceIndex<0)
            throwError("Cannot find joint %s\n", jt->jName);
        jt->rightStanceIndex = jt->leftStanceIndex;
    }



    ///TODO think about that (where it should be done and else)
    //at the end of this function I'm sure that all the character is created.
    //so I'll be able to create the finites elements

    //I'll start dividing the foots
    //RigidBody* body= character->getARBByName("lFoot");
    //CollisionDetectionPrimitive* cdp = body->cdps.front();
    //BoxCDP* box = static_cast<BoxCDP*>(cdp);
    //box->compute_finites_elements();
}





void PoseController::set_fsm_state(int index){
    if (index<0 || (uint)index>=_states.size()){
        _state_idx = 0;
        return;
    }
    _state_idx = index;
}

void PoseController::transition_to_state(int idx)
{
    set_fsm_state(idx);
    int new_stance=_states[_state_idx]->getStateStance(character->stance());

    ///TODO think about that line (should it be done here or not)
    set_stance(new_stance);

}

void PoseController::evaluate_joints_target(double phi)
{
    ReducedCharacterState rs(&PoseControlGlobals::desired_pose);



    double cur_phi = MIN(phi, 1);
    double future_phi = MIN(cur_phi + SimGlobals::dt / swing_foot_controller->get_cur_state_time(), 1);

    //d and v are specified in the rotation (heading) invariant coordinate frame
    //already done before

    Vector3d comPosition = character->getCOM();

    Vector3d d = Vector3d(character->stance_foot()->getCMPosition(), comPosition);
    //d is now in world coord frame, so we'll represent it in the 'character frame'
    d = character->getHeadingHorizontal().getComplexConjugate().rotate(d);
    //compute v in the 'character frame' as well
    Vector3d v = character->getHeadingHorizontal().getComplexConjugate().rotate(character->getCOMVelocity());



    //there are two stages here. First we will compute the pose (i.e. relative orientations),
    //using the joint trajectories for the current state
    //and then we will compute the PD torques that are needed to drive the links
    //towards their orientations - here the special case of the
    //swing and stance hips will need to be considered



    SimBiConState* curState = _states[_state_idx];
    Quaternion newOrientation,newOrientationF;

    //get the estimated global trajectory
    //I noticed that those values are always equal to 0 (a least on the forward walk...)
    //Vector3d d0, v0;
    //computeD0(phiToUse, &d0);
    //computeV0(phiToUse, &v0);

    //just precompute the real D and v (couting the base trajectory)
    Vector3d d_d = d;// - d0;
    Vector3d d_v = v;// - v0;


    for (int i = 0; i<curState->getTrajectoryCount(); i++){
        //now we have the desired rotation angle and axis, so we need to see which joint this is intended for
        int jIndex = curState->getTrajectory(i)->getJointIndex(character->stance());

        //anything loer than -1 is just a trajectory stored here for a simple usage
        //but it's not a real tajectory so I have to ignore them
        if (jIndex < -1){
            continue;
        }


        //get the desired joint orientation to track - include the feedback if necessary/applicable
        newOrientation = curState->getTrajectory(i)->evaluateTrajectory(character->stance(), cur_phi, d_d, d_v);
        newOrientationF = curState->getTrajectory(i)->evaluateTrajectory(character->stance(), future_phi, d_d, d_v);

        //compute the target velocity
        Quaternion qDiff = newOrientationF * newOrientation.getComplexConjugate();
        Vector3d ang_vel = Vector3d(0,0,0);
        if (newOrientationF!=newOrientation){
            ang_vel =qDiff.v/(qDiff.v.length())*(safeACOS(qDiff.s)*2/SimGlobals::dt);
        }

        if (jIndex==character->stance_foot()->parent_joint()->idx()&&SimGlobals::foot_flat_on_ground){
            ang_vel=Vector3d(0,0,0);
        }

        //if the index is -1, it must mean it's the root's trajectory. Otherwise we should give an error
        if (jIndex == -1){
            qRootD = Quaternion::getRotationQuaternion(desired_heading_pelvis, SimGlobals::up) * newOrientation;
            qVelD = Quaternion::getRotationQuaternion(desired_heading_pelvis, SimGlobals::up).rotate(ang_vel);
            //            rs.setOrientation(character->getRoot()->getOrientation());
            rs.setAngularVelocity(qVelD);
            rs.setOrientation(qRootD);
            rootControlParams.strength = curState->getTrajectory(i)->evaluateStrength(cur_phi);
        }
        else{
            rs.setJointRelativeOrientation(newOrientation, jIndex);
            rs.setJointRelativeAngVelocity(ang_vel, jIndex);
            controlParams[jIndex].strength = curState->getTrajectory(i)->evaluateStrength(cur_phi);
        }
    }
}



void PoseController::control_desired_heading(double phase)
{
    if (!Globals::use_fluid_heading){
        desired_heading_pelvis=SimGlobals::desiredHeading;
        desired_heading_swing_foot=desired_heading_pelvis;
        return;
    }

    //all this is for the smooth transition of the heading
    //the turning strategy is to only detecte modification during the use of the IPM
    //also after detecting a modification. the target will only be half the total modification
    //until the end of the step. the rest of the modification will be effectuated during the next step
    //once we detect that the foot is fully anchored in the ground

    //if it's the same heading, do nothing
    if (desired_heading_pelvis==SimGlobals::desiredHeading){
        old_heading=desired_heading_pelvis;
        start_phi_change_heading=phase;
        return;
    }

    //we do not touche the desired orientation of the pelvis as long as the foot is not anchored
    if ((!SimGlobals::foot_flat_on_ground)){
        old_heading=desired_heading_pelvis;
        start_phi_change_heading=phase;
        return;
    }

    //also I don't try anything as long as we are in the early step phase
    if (swing_foot_controller->is_early_step()){
        old_heading=desired_heading_pelvis;
        start_phi_change_heading=phase;
        return;
    }

    if(first_pass_this_char_step){
        old_heading=character->getHeadingAngle();//in case the heading moved during the wait time
        first_pass_this_char_step=false;
        std::cout<<"trig"<<std::endl;
    }

    if (target_heading!=SimGlobals::desiredHeading){
        //this if allow to only register orientations changes after the start of the us of the ipm
        if (!swing_foot_controller->is_early_step(0.1)){
            old_heading=desired_heading_pelvis;
            start_phi_change_heading=phase;
            target_heading=SimGlobals::desiredHeading;
            target_heading_mitigation_coefficient=0.75;
        }else{
            old_heading=desired_heading_pelvis;
            start_phi_change_heading=phase;
            return;
        }
    }

/*
    double current_mult_swing_foot=std::min(1.0,(phase-start_phi_change_heading)/0.6);
    desired_heading_swing_foot=old_heading+ ((target_heading-old_heading)*
                                             target_heading_mitigation_coefficient)*current_mult_swing_foot;//*/

    double current_mult_pelvis=std::min(1.0,(phase-start_phi_change_heading)/0.8);
    desired_heading_pelvis=old_heading+ ((target_heading-old_heading)*
                                         target_heading_mitigation_coefficient)*current_mult_pelvis;

    if (std::abs(desired_heading_pelvis-target_heading)/std::abs(target_heading)<0.01){
        //std::cout<<"completed change"<< desired_heading_pelvis<<"  "<< target_heading<<std::endl;
        desired_heading_pelvis=target_heading;
    }


    desired_heading_swing_foot=desired_heading_pelvis;



}


void PoseController::compute_torques(){
    Quaternion qRelD;
    Vector3d relAngVelD;

    Quaternion qRel;
    Vector3d wRel;

    bool use_spd=false;

    ReducedCharacterState rs(&PoseControlGlobals::desired_pose);


    //    static std::vector<Vector3d> vect_speeds_p[50];
    //    static std::vector<Vector3d> vect_speeds_c[50];

    for (int i=0;i<character->getJointCount();i++){
        if (controlParams[i].controlled == true ){
            ControlParams params = controlParams[i];
            std::vector<Vector3d>& vect_speed=vect_speeds[i];

            //*
            //uint idx = character->getJoint(i)->idx();

            /*
            Vector3d test=rs.getJointRelativeAngVelocity(i);
            Vector3d test2=Vector3d(0,0,0);
            std::ostringstream oss;
            oss<<"roottorque "<<test.x<<" "<<test.y<<" "<<test.z<<" "<<test2.x<<" "<<test2.y<<" "<<test2.z<<"   "<<character->getJoint(i)->name();
            std::cout<<oss.str();
            //*/


            /*if (resulting_impact.find(idx) != resulting_impact.end()){
                double length = resulting_impact[idx].drag_torque.length();
                if (!IS_ZERO(length)){
                params.kp *= 1 + length*SimGlobals::time_factor;
                params.kd =std::sqrt(params.kd*params.kd/4* 1 + length*SimGlobals::time_factor)*2;
                }//*/
            //if (std::find(idx_vect.begin(), idx_vect.end(), idx) != idx_vect.end()){
            //params.kp *= SimGlobals::time_factor;
            //params.kd = std::sqrt(params.kd*params.kd / 4 * (SimGlobals::time_factor)) * 2;
            //}//*/
            if (controlParams[i].relative_to_char_frame == false){

                if (use_spd){
                    //get the current relative orientation between the child and parent
                    character->getRelativeOrientationFuture(i, &qRel);
                    //and the relative angular velocity, computed in parent coordinates
                    character->getRelativeAngularVelocityFuture(i, &wRel);
                }else{
                    //get the current relative orientation between the child and parent
                    character->getRelativeOrientation(i, &qRel);
                    //and the relative angular velocity, computed in parent coordinates
                    character->getRelativeAngularVelocity(i, &wRel);
                }

                RigidBody* childRB = character->getJoint(i)->child();
                RigidBody* parentRB = character->getJoint(i)->parent();
                /*
                int buffer_size=4;
                vect_speed.push_back(wRel);
                Vector3d wRel_avg=Vector3d(0,0,0);
                for (int j=0;j<vect_speed.size();++j){
                    wRel_avg+=vect_speed[j];
                }
                wRel_avg=wRel_avg/vect_speed.size();
                if (vect_speed.size()>=buffer_size){
                    vect_speed.erase(vect_speed.begin());
                }



                std::vector<Vector3d>& vect_speed_c=vect_speeds_c[i];
                wRel=childRB->getAngularVelocity();
                //do the avg on the max 4 last steps for the speed
                vect_speed_c.push_back(wRel);
                Vector3d wRel_avg_c=Vector3d(0,0,0);
                for (int j=0;j<vect_speed_c.size();++j){
                    wRel_avg_c+=vect_speed_c[j];
                }
                wRel_avg_c=wRel_avg_c/vect_speed_c.size();
                if (vect_speed_c.size()>=buffer_size){
                    vect_speed_c.erase(vect_speed_c.begin());
                }


                std::vector<Vector3d>& vect_speed_p=vect_speeds_p[i];
                wRel=parentRB->getAngularVelocity();
                //do the avg on the max 4 last steps for the speed
                vect_speed_p.push_back(wRel);
                Vector3d wRel_avg_p=Vector3d(0,0,0);
                for (int j=0;j<vect_speed_p.size();++j){
                    wRel_avg_p+=vect_speed_p[j];
                }
                wRel_avg_p=wRel_avg_p/vect_speed_p.size();
                if (vect_speed_p.size()>=buffer_size){
                    vect_speed_p.erase(vect_speed_p.begin());
                }

                Vector3d wRel_result=wRel_avg_c-wRel_avg_p;
                wRel_result=parentRB->getLocalCoordinates(wRel_result);
                //*/

                //this is the system that give the avg angular vaocity but I don't currently uses it
                if(false){
                    Vector3d wRel_result2=childRB->get_angular_velocity_avg()-parentRB->get_angular_velocity_avg();
                    wRel_result2=parentRB->getLocalCoordinates(wRel_result2);
                }


                /*
                bool to_check=false;
                if (idx==character->swing_hip()->idx()){
                    character->getRelativeAngularVelocity(i, &wRel);

                    Vector3d test=wRel_result2;
                    Vector3d test2=rs.getJointRelativeAngVelocity(i);
                    std::ostringstream oss;
                    oss<<"roottorque "<<std::fixed<<test.x<<" "<<test.y<<" "<<test.z<<" "<<test2.x<<" "<<test2.y<<" "<<test2.z;
                    std::cout<<oss.str();
//                    to_check=true;
                }
                //*/

                //now compute the torque
                _torques[i] = compute_pd_torque(qRel, rs.getJointRelativeOrientation(i), wRel, rs.getJointRelativeAngVelocity(i), &params);
                //the torque is expressed in parent coordinates, so we need to convert it to world coords now
                _torques[i] = character->getJoint(i)->parent()->getWorldCoordinates(_torques[i]);


                Vector3d t=_torques[i];
                if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
                    std::cout<<"PoseController::compute_torques  undefinned torques   ";
                }
            }else{
                RigidBody* childRB = character->getJoint(i)->child();
                if (use_spd){
                    qRel=childRB->getOrientationFuture();
                    wRel=childRB->getAngularVelocityFuture();
                }else{
                    qRel=childRB->getOrientation();
                    wRel=childRB->getAngularVelocity();
                }

                //this use an avg system for the angular velocity but I don't currently uses it
                if (false)
                {
                    //do the avg on the max 4 last steps for the speed
                    Vector3d wRel_avg=Vector3d(0,0,0);
                    vect_speed.push_back(wRel);
                    for (int j=0;j<vect_speed.size();++j){
                        wRel_avg+=vect_speed[j];
                    }
                    wRel_avg=wRel_avg/vect_speed.size();
                    if (vect_speed.size()>3){
                        vect_speed.erase(vect_speed.begin());
                    }
                }
                /*
                if (childRB->angular_velocity_avg_is_valid()){
                    wRel_avg=childRB->get_angular_velocity_avg();
                }else{
//                    wRel_avg= rs.getJointRelativeAngVelocity(i);
                }
                //*/

                /*
                if (idx==character->root_top()->idx()){
                    if (childRB->angular_velocity_avg_is_valid()){
                        std::cout<<"valid";
                    }
                    Vector3d test=childRB->get_angular_velocity_avg();
                    Vector3d test2=rs.getJointRelativeAngVelocity(i);
                    std::ostringstream oss;
                    oss<<"roottorque "<<test.x<<" "<<test.y<<" "<<test.z<<" "<<test2.x<<" "<<test2.y<<" "<<test2.z;
                    std::cout<<oss.str();
                }
                //*/


                _torques[i] = compute_pd_torque(qRel, controlParams[i].char_frame * rs.getJointRelativeOrientation(i),
                                                wRel, rs.getJointRelativeAngVelocity(i), &params);


                Vector3d t=_torques[i];
                if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
                    std::cout<<"PoseController::compute_torques  undefinned torques   ";
                }

            }
        }else{
            _torques[i].setValues(0,0,0);
        }
    }
}

void PoseController::compute_torque_for_joint(int jIdx, bool relative_to_char_frame)
{

}

Vector3d PoseController::compute_torque_for_joint_from_angular_displacement(int jIdx, Vector3d angular_displacement)
{
    ControlParams* cParams=&controlParams[jIdx];
    Vector3d torque=angular_displacement*cParams->kp;

    return torque;
}

Vector3d PoseController::compute_pd_torque(const Quaternion &qRel, const Quaternion &qRelD, const Vector3d &wRel,
                                           const Vector3d &wRelD, ControlParams *cParams, bool to_test)
{
    Vector3d torque;
    //the torque will have the form:
    // T = kp*D(qRelD, qRel) + kd * (wRelD - wRel)

    //Note: There can be problems computing the proper torque from the quaternion part, because q and -q
    //represent the same orientation. To make sure that we get the correct answer, we'll take into account
    //the sign of the scalar part of qErr - both this and the v part will change signs in the same way if either
    //or both of qRel and qRelD are negative
    //	Quaternion qErr = qRel.getComplexConjugate() * qRelD;
    Quaternion qErr = qRel.getComplexConjugate();
    qErr *= qRelD;

    //qErr.v also contains information regarding the axis of rotation and the angle (sin(theta)), but I want to scale it by theta instead
    double sinTheta = qErr.v.length();
    if (sinTheta>1)
        sinTheta = 1;
    if (IS_ZERO(sinTheta)){
        //avoid the divide by close-to-zero. The orientations match, so the proportional component of the torque should be 0
    }else{
        double absAngle = 2 * safeASIN(sinTheta);
        torque = qErr.v;
        torque *= 1/sinTheta * absAngle * (-cParams->kp) * SGN(qErr.s);
        //		torque = qErr.v/sinTheta * absAngle * (-cParams->kp) * SGN(qErr.s);
    }

    //qErr represents the rotation from the desired child frame to the actual child frame, which
    //means that the torque is now expressed in child coordinates. We need to express it in parent coordinates!
    torque = qRel.rotate(torque);

    //the angular velocities are stored in parent coordinates, so it is ok to add this term now
    Vector3d vel_torque=(wRelD - wRel) * (-cParams->kd);
    if (to_test){
        //*
        Vector3d test=torque;
        Vector3d test2=vel_torque;
        std::ostringstream oss;
        oss<<"roottorque "<<std::showpos<<std::fixed<<test.x<<" "<<test.y<<" "<<test.z<<"   "<<test2.x<<" "<<test2.y<<" "<<test2.z;
        std::cout<<oss.str();
        //*/
    }



    torque += vel_torque;
    torque *= cParams->strength;


    //now the torque is stored in parent coordinates - we need to scale it and apply torque limits
    scale_and_limit_torque(&torque, cParams, qRel.getComplexConjugate());

    Vector3d t=torque;
    if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
        std::cout<<"PoseController::compute_pd_torque  undefinned torques   ";
    }

    //and we're done...
    return torque;
}

void PoseController::scale_and_limit_torque(Vector3d *torque, ControlParams *cParams, const Quaternion &qToChild)
{
    //now change the torque to child coordinates
    *torque = qToChild.rotate(*torque);

    //and scale it differently along the main axis...
    torque->x *= cParams->scale.x;
    torque->y *= cParams->scale.y;
    torque->z *= cParams->scale.z;

    limit_torque(torque, cParams);

    // and now change it back to the original coordinates
    *torque = qToChild.getComplexConjugate().rotate(*torque);
}

void PoseController::limit_torque(Vector3d *torque, ControlParams *cParams)
{
    if (torque->x < -cParams->scale.x * cParams->max_abs_torque){
        torque->x = -cParams->scale.x * cParams->max_abs_torque;
    }
    if (torque->x > cParams->scale.x * cParams->max_abs_torque){
        torque->x = cParams->scale.x * cParams->max_abs_torque;
    }
    if (torque->y < -cParams->scale.y * cParams->max_abs_torque){
        torque->y = -cParams->scale.y * cParams->max_abs_torque;
    }
    if (torque->y > cParams->scale.y * cParams->max_abs_torque){
        torque->y = cParams->scale.y * cParams->max_abs_torque;
    }
    if (torque->z < -cParams->scale.z * cParams->max_abs_torque){
        torque->z = -cParams->scale.z * cParams->max_abs_torque;
    }
    if (torque->z > cParams->scale.z * cParams->max_abs_torque){
        torque->z = cParams->scale.z * cParams->max_abs_torque;
    }
}

void PoseController::limit_torque(Vector3d *torque, int joint_idx)
{
    limit_torque(torque, &controlParams[joint_idx]);
}


void PoseController::bubbleup_torques(){
    int stance_hip_idx= character->stance_hip()->idx();
    int stance_knee_idx= character->stance_hip()->child()->child_joints().front()->idx();


    for (int i = character->getJointCount() - 1; i >= 0; i--){
        if (i != stance_hip_idx && i != stance_knee_idx)
            if (character->getJoint(i)->parent() != character->getRoot())
                _torques[character->getJoint(i)->parent()->parent_joint()->idx()] += _torques[i];
    }
}


double PoseController::state_time(){
    return _states[_state_idx]->getStateTime();
}



void PoseController::target_pose(std::vector<double> &pose){
    pose=PoseControlGlobals::desired_pose;
}

void PoseController::set_character_to_desired_pose(Character *c)
{
    std::vector<double> tmp=PoseControlGlobals::desired_pose;
    ReducedCharacterState rs(&tmp);
    ReducedCharacterState rs_original(&PoseControlGlobals::desired_pose);

    //now when we do that it considers that the joints orientation are exprimed in relative coordinate
    //so we need to convert the ones that are actually expressed in the character frame.
    //the proble is that everything is stored in global cooordinate in the character
    for (int jIndex = 0; jIndex<controlParams.size(); ++jIndex){
        if (controlParams[jIndex].relative_to_char_frame){
            Joint* joint=c->getJoint(jIndex);

            Quaternion local_frame;
            Joint* p_j=joint->parent()->parent_joint();
            while (p_j!=NULL){
                Quaternion p_ori=rs.getJointRelativeOrientation(p_j->idx());
                local_frame=p_ori*local_frame;
                p_j=p_j->parent()->parent_joint();
            }
            //add the orientation of the root
            local_frame= rs.getOrientation()*local_frame;

            Quaternion ori=controlParams[jIndex].char_frame*rs.getJointRelativeOrientation(jIndex);
            //compute the orientation in the local_frame
            ori=local_frame.getComplexConjugate()*ori;

            //and set it
            rs.setJointRelativeOrientation(ori,jIndex);
        }
    }

    //and now that everything is in relative coordinate we can set the state of the character
    c->setState(&tmp);


    /*
    Point3d hip_loca=c->swing_hip()->child()->getWorldCoordinates(c->swing_hip()->child_joint_position());
    Point3d ankle_loca=c->swing_foot()->parent_joint()->parent()->getWorldCoordinates(c->swing_foot()->parent_joint()->parent_joint_position());


    Vector3d print_vect=hip_loca-ankle_loca;
    std::ostringstream oss4;
    oss4<<"test2_normal"<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
    std::cout<<oss4.str();
    //*/
}

void PoseController::set_swing_leg_rotation_angle(double angle)
{

    ReducedCharacterState rs(&PoseControlGlobals::desired_pose);


    Joint* joint=character->swing_foot()->parent_joint();
    Quaternion local_frame;
    Joint* p_j=joint->parent()->parent_joint();
    while (p_j!=NULL){
        Quaternion p_ori=rs.getJointRelativeOrientation(p_j->idx());
        local_frame=p_ori*local_frame;
        p_j=p_j->parent()->parent_joint();
    }
    Quaternion tibias_frame= rs.getOrientation()*local_frame;
    tibias_frame.toUnit();

    p_j=character->swing_hip();
    local_frame=Quaternion();
    while (p_j!=NULL){
        Quaternion p_ori=rs.getJointRelativeOrientation(p_j->idx());
        local_frame=p_ori*local_frame;
        p_j=p_j->parent()->parent_joint();
    }
    Quaternion femur_frame= rs.getOrientation()*local_frame;
    femur_frame.toUnit();

    Quaternion rotation_frame=local_frame;
    //add the orientation of the root
    rotation_frame= rs.getOrientation()*local_frame;

    //*
    //now we need to compute the quaternion that goes for the global coordinate to
    //the coordiante corresponding to tthe direction swing_hip-swing_foot.
    //and the swing leg will most likely point toward the ground
    //it is not realy likely that the hip is lower thant the foot...
    //s the direction to was we will fit (+y) is  swing_foot-swing_hip
    Point3d start=character->swing_foot()->parent_joint()->parent_joint_position();
    Point3d end=character->swing_foot()->parent_joint()->parent()->parent_joint()->child_joint_position();
    Vector3d tibias_vect_local=end-start;

    Point3d start2=character->swing_foot()->parent_joint()->parent()->parent_joint()->parent_joint_position();
    Point3d end2=character->swing_hip()->child_joint_position();
    Vector3d femur_vect_local=end2-start2;


    Vector3d tibias_vect=tibias_frame.rotate(tibias_vect_local);
    Vector3d femur_vect=femur_frame.rotate(femur_vect_local);
    Vector3d rot_axis=tibias_vect+femur_vect;

    //compute the local referencial
    Vector3d rot_axis_unit=rot_axis.unit();
    Vector3d normal=(rot_axis_unit).crossProductWith(tibias_vect);
    normal.toUnit();
    Vector3d normal2=normal.crossProductWith(rot_axis_unit);
    normal2.toUnit();


    if (old_val==-angle){
        if (!trigerred_once){
            trigerred_once=true;
            Vector3d print_vect=normal2;
            std::ostringstream oss4;
            oss4<<"angle: "<<old_val<<" // "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
            std::cout<<oss4.str();
        }
    }else{
        if (old_val!=angle){
            old_val=angle;
            trigerred_once=false;
            std::cout<<"reseted";
        }
    }

    /*
    print_vect=(normal);
    std::ostringstream oss4;
    oss4<<"normal1 "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
    std::cout<<oss4.str();
//*/
    /*
    print_vect=(normal2);
    std::ostringstream oss5;
    oss5<<"normal2 "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
    std::cout<<oss5.str();
//*/
    /*
    print_vect=(rot_axis_unit);
    std::ostringstream oss6;
    oss6<<"normal3 "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
    std::cout<<oss6.str();
//*/


    Quaternion desired_rotation_quat=Quaternion::getRotationQuaternion(angle,rot_axis_unit);


    //affect the rotation in the desired poses
    Quaternion p_ori=rs.getOrientation()*rs.getJointRelativeOrientation(character->swing_hip()->idx());
    p_ori=rs.getOrientation().getComplexConjugate()*desired_rotation_quat*p_ori;
    rs.setJointRelativeOrientation(p_ori,character->swing_hip()->idx());

}

void PoseController::set_leg_forward_to_orientation(Quaternion orientation, bool is_swing, float influence)
{
    Joint* ankle;
    Joint* hip;

    if (is_swing){
        ankle=character->swing_foot()->parent_joint();
        hip=character->swing_hip();
    }else{
        ankle=character->stance_foot()->parent_joint();
        hip=character->stance_hip();
    }

    ReducedCharacterState rs(&PoseControlGlobals::desired_pose);

    Vector3d print_vect;
    //*
    Joint* joint=ankle;
    Quaternion local_frame;
    Joint* p_j=joint->parent()->parent_joint();
    while (p_j!=NULL){
        Quaternion p_ori=rs.getJointRelativeOrientation(p_j->idx());
        local_frame=p_ori*local_frame;
        p_j=p_j->parent()->parent_joint();
    }
    Quaternion tibias_frame= rs.getOrientation()*local_frame;
    tibias_frame.toUnit();

    p_j=hip;
    local_frame=Quaternion();
    while (p_j!=NULL){
        Quaternion p_ori=rs.getJointRelativeOrientation(p_j->idx());
        local_frame=p_ori*local_frame;
        p_j=p_j->parent()->parent_joint();
    }
    Quaternion femur_frame= rs.getOrientation()*local_frame;
    femur_frame.toUnit();

    Quaternion rotation_frame=local_frame;
    //add the orientation of the root
    rotation_frame= rs.getOrientation()*local_frame;

    //*
    //now we need to compute the quaternion that goes for the global coordinate to
    //the coordiante corresponding to tthe direction hip-ankle.
    //and the leg will most likely point toward the ground
    //it is not realy likely that the hip is lower thant the foot...
    //s the direction to was we will fit (+y) is  ankle-hip
    Point3d start=ankle->parent_joint_position();
    Point3d end=ankle->parent()->parent_joint()->child_joint_position();
    Vector3d tibias_vect_local=end-start;

    Point3d start2=ankle->parent()->parent_joint()->parent_joint_position();
    Point3d end2=hip->child_joint_position();
    Vector3d femur_vect_local=end2-start2;


    Vector3d tibias_vect=tibias_frame.to_world_coordinate(tibias_vect_local);
    Vector3d femur_vect=femur_frame.to_world_coordinate(femur_vect_local);
    Vector3d rot_axis=tibias_vect+femur_vect;

    //compute the local referencial
    Vector3d rot_axis_unit=rot_axis.unit();
    Vector3d normal=(rot_axis_unit).crossProductWith(tibias_vect);
    normal.toUnit();
    Vector3d normal2=normal.crossProductWith(rot_axis_unit);
    normal2.toUnit();



    /*
    print_vect=(normal);
    std::ostringstream oss4;
    oss4<<"normal1 "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
    std::cout<<oss4.str();
//*/
    /*
    print_vect=(normal2);
    std::ostringstream oss5;
    oss5<<"normal2 "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
    std::cout<<oss5.str();
//*/
    /*
    print_vect=(rot_axis_unit);
    std::ostringstream oss66;
    oss66<<"rot_axis "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
    std::cout<<oss66.str();
//*/

    //so to obtain the target we have to:
    // take global z axis
    // apply desired character frame
    // look at the x component
    // find the direction on the rotation plane with the same x component

    Vector3d local_forward(0,0,1);
    Vector3d world_forward=orientation.to_world_coordinate(local_forward);
    world_forward.toUnit();
    double x_target,y_target,z_target;
    x_target=world_forward.x;

    Vector3d proj_target;
    /*
    //I can't seem to rememebre why there is a degree 2 polynom so I use another way...
    //anyway there are some nan in near 90° rotations
    double a,b,c;
    if (std::abs(x_target)<1.0){
        //we solve a degree 2 polinom here to find the z_target
        a=1+(rot_axis.z/rot_axis.y)*(rot_axis.z/rot_axis.y);
        b=2*rot_axis.x*x_target*rot_axis.z/(rot_axis.y*rot_axis.y);
        c=(-1+x_target*x_target+((x_target*rot_axis.x)/rot_axis.y)*((x_target*rot_axis.x)/rot_axis.y));

        //for now we limit to the case where z_target>=0;
        z_target=-(b-std::sqrt(b*b-4*a*c))/(2*a);
        y_target=-(rot_axis.x*x_target+rot_axis.z*z_target)/rot_axis.y;

        proj_target=Vector3d(x_target,y_target,z_target);
    }else{
        proj_target=Vector3d(x_target,0,0);
    }
    //*/
    ///so the solution is to find the intersection of (0,1,0) passing by world_forward
    /// with the rotation plane knowing the rotation plane rotate around O(0,0,0)
    ///http://geomalgorithms.com/a05-_intersect-1.html
    Vector3d W(world_forward.x,world_forward.y,world_forward.z);
    Vector3d n=rot_axis_unit;
    Vector3d u(0,1,0);

    double S1=-(n.dotProductWith(W))/(n.dotProductWith(u));

    proj_target=Vector3d(world_forward.x,world_forward.y+S1*u.y,world_forward.z);
    proj_target.toUnit();



    /*
    print_vect=normal2-proj_target;
    std::ostringstream oss66;
    oss66<<"error befor "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
    std::cout<<oss66.str();


    print_vect=proj_target;
    std::ostringstream oss250;
    oss250<<"length: "<<proj_target.length()<<" dot_prot: "<<rot_axis_unit.dotProductWith(proj_target)
         <<" targetp: "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
        std::cout<<oss250.str();
//*/

    //get the angle of rotation
    double cos_value=(normal.dotProductWith(proj_target));
    double desired_rotation_angle=safeASIN(cos_value);
    //the influence is here to do a smooth transition for the rotation
    {
        std::ostringstream oss;
        oss<<"influence: "<<influence<<"    desired rotation angle: "<<desired_rotation_angle;
        //        std::cout<<oss.str();
    }
    desired_rotation_angle*=influence;
    Quaternion desired_rotation_quat=Quaternion::getRotationQuaternion(desired_rotation_angle,rot_axis_unit);

    {
        Vector3d t=desired_rotation_quat.v;
        if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
            std::cout<<"PoseController::compute_torques  undefinned orientation 3 ";

            t=rot_axis_unit;
            if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
                std::cout<<"PoseController::compute_torques  undefinned orientation 3.1 ";
            }

            if (desired_rotation_angle!=desired_rotation_angle){
                std::cout<<"PoseController::compute_torques  undefinned orientation 3.2 ";
            }

            if (cos_value!=cos_value){
                std::cout<<"PoseController::compute_torques  undefinned orientation 3.3 ";

                t=normal;
                if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
                    std::cout<<"PoseController::compute_torques  undefinned orientation 3.3.1 ";
                }

                t=proj_target;
                if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
                    std::cout<<"PoseController::compute_torques  undefinned orientation 3.3.2 ";
                }
            }
        }
    }

    //    std::ostringstream oss6;
    //    double desired_cos=std::cos(desired_rotation_angle);
    //    oss6<<"angle "<<desired_rotation_angle<< "  cos: "<<desired_cos<< "  current: "<<cos_value;
    //    std::cout<<oss6.str();



    //affect the rotation in the desired poses
    Quaternion p_ori=rs.getOrientation()*rs.getJointRelativeOrientation(hip->idx());

    Vector3d t=p_ori.v;
    if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
        std::cout<<"PoseController::compute_torques  undefinned orientation 1  ";
    }
    p_ori=rs.getOrientation().getComplexConjugate()*desired_rotation_quat*p_ori;

    rs.setJointRelativeOrientation(p_ori,hip->idx());


    t=p_ori.v;
    if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
        std::cout<<"PoseController::compute_torques  undefinned orientation 2 ";

        t=desired_rotation_quat.v;
        if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
            std::cout<<"PoseController::compute_torques  undefinned orientation 2.1 ";
        }

        t=rs.getOrientation().v;
        if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
            std::cout<<"PoseController::compute_torques  undefinned orientation 2.2 ";
        }

        t=rs.getOrientation().getComplexConjugate().v;
        if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
            std::cout<<"PoseController::compute_torques  undefinned orientation 2.3 ";
        }
    }

    //test result

    /*
    Quaternion tibias_frame_new=desired_rotation_quat*tibias_frame;
    Quaternion femur_frame_new=desired_rotation_quat*femur_frame;

    Vector3d tibias_vect_new=tibias_frame_new.rotate(tibias_vect_local);
    Vector3d femur_vect_new=femur_frame_new.rotate(femur_vect_local);

    print_vect=tibias_vect+femur_vect-(tibias_vect_new+femur_vect_new);
    std::ostringstream oss7;
    oss7<<"last3 "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
//        std::cout<<oss7.str();
//*



    Vector3d rot_axis_unit_new=(tibias_vect_new+femur_vect_new).unit();
    Vector3d normal_new=(rot_axis_unit_new).crossProductWith(tibias_vect_new);
    normal_new.toUnit();
    Vector3d normal2_new=normal_new.crossProductWith(rot_axis_unit_new);
    normal2_new.toUnit();
//*
    print_vect=normal2_new-proj_target;
    std::ostringstream oss8;
    oss8<<"error after "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
    std::cout<<oss8.str();
//*/
}



void PoseController::tracking_pose(std::vector<double> &trackingPose, double phiToUse)
{
    if( phiToUse < 0 || phiToUse>1 ){
        throw "PoseController::tracking_pose   incorect phi required";
    }

    ///TODO finish this function (the only thing to do is to compute d and v
    /*

    trackingPose.clear();
    character->getState(&trackingPose);

    ReducedCharacterState debugRS(&trackingPose);

    //always start from a neutral desired pose, and build from there...
    for (int i=0;i<character->getJointCount();i++){
        debugRS.setJointRelativeOrientation(Quaternion(1, 0, 0, 0), i);
        debugRS.setJointRelativeAngVelocity(Vector3d(), i);
        controlParams[i].relative_to_char_frame = false;
    }

    //and this is the desired orientation for the root
    Quaternion qRootD(1, 0, 0, 0);

    SimBiConState* curState = _states[state_idx];

    for (int i=0;i<curState->getTrajectoryCount();i++){
        //now we have the desired rotation angle and axis, so we need to see which joint this is intended for
        int jIndex = curState->getTrajectory(i)->getJointIndex(character->stance());

        //if the index is -1, it must mean it's the root's trajectory. Otherwise we should give an error
        if (curState->getTrajectory(i)->relative_to_char_frame == true || jIndex == character->swing_hip()->idx())
            controlParams[jIndex].relative_to_char_frame = true;

        Quaternion newOrientation = curState->getTrajectory(i)->evaluateTrajectory(character->stance(), phiToUse, d, v);
        if (jIndex == -1){
            qRootD = newOrientation;
        }else{
            debugRS.setJointRelativeOrientation(newOrientation, jIndex);
        }
    }

    debugRS.setOrientation(qRootD);

    //now, we'll make one more pass, and make sure that the orientations that are relative to the character frame are drawn that way
    for (int i=0;i<jointCount;i++){
        if (controlParams[i].relative_to_char_frame){
            Quaternion temp;
            Joint* j = character->getJoint(i);
            ArticulatedRigidBody* parent = j->parent();
            while (parent != root){
                j = parent->parent_joint();
                parent = j->parent();
                temp = debugRS.getJointRelativeOrientation(character->getJointIndex(j->name().c_str())) * temp;
            }

            temp = qRootD * temp;
            temp = temp.getComplexConjugate() * debugRS.getJointRelativeOrientation(i);
            debugRS.setJointRelativeOrientation(temp, i);
        }
    }
    //*/
}


void PoseController::set_stance(int nstance){
    character->update_stance(nstance);

    //Now I need to handle the case of the asymetric gains for the legs
    //just a note if the gain are symetric that system does nothing
    //so I don't have to consider the case where that system is not used
    if (check_fsm_state()){
        //we will say that the right side contains the values for the stance
        std::vector<std::string> target={"rHip","rKnee","rAnkle","rToeJoint","lHip","lKnee","lAnkle","lToeJoint"};
        std::vector<std::string> input={"rHip","rKnee","rAnkle","rToeJoint","lHip","lKnee","lAnkle","lToeJoint"};
        if (character->is_left_stance()){
            input={"lHip","lKnee","lAnkle","lToeJoint","rHip","rKnee","rAnkle","rToeJoint"};
        }

        for (int i=0;i<target.size();++i){
            controlParams[character->getJointByName(target[i])->idx()]=
                    controlParams_initial[character->getJointByName(input[i])->idx()];
        }

    }

    ///PROTOCOL: activate those lines if you want color on stance foot
    //    character->stance_foot()->set_mesh_color(0, 0, 1, 1);
    //    character->swing_foot()->set_mesh_color(1, 1, 1, 1);
}

bool PoseController::check_fsm_state()
{
    if (state_idx()<0 || state_idx() >= (int)_states.size()){
        std::cerr<<"Warning: no FSM state was selected in the controller!"<<std::endl;
        return false;
    }
    return true;
}

void PoseController::compute_root_orientation_torques(Vector3d& stance_hip_torque, Vector3d& swing_hip_torque,
                                                      const Vector3d& pelvis_torso_torque)
{

    RigidBody* root=character->getRoot();

    //compute the total torques that should be applied to the root and swing hip, keeping in mind that
    //the desired orientations are expressed in the character frame
    Vector3d rootTorque;

    //this is the desired orientation and angular velocity in world coordinates
    Quaternion qRootDW;
    Vector3d qVelDW;

    //if (SimGlobals::forceHeadingControl == false){
    //qRootD is specified in the character frame, so just maintain the current heading
    //qRootDW = qRootD;
    //qRootDW = characterFrame * qRootD;
    //}else{
    //qRootDW needs to also take into account the desired heading
    qRootDW =  qRootD;
    qVelDW = qVelD;

    //}

    double rootStrength = rootControlParams.strength;
    if (rootStrength < 0)
        rootStrength = 0;
    if (rootStrength > 1)
        rootStrength = 1;
    double stanceHipToSwingHipRatio=character->get_stance_foot_weight_ratio();
    if (stanceHipToSwingHipRatio< 0)
        rootStrength = 0;



    Vector3d wRel=root->getAngularVelocity();
    root->set_angular_vel_avg_max_size(2);
    Vector3d wRel_avg=root->get_angular_velocity_avg();
    if (!root->angular_velocity_avg_is_valid()){
        wRel_avg=Vector3d(0,0,0);
    }

    //so this is the net torque that the root wants to see, in world coordinates
    rootTorque = compute_pd_torque(root->getOrientation(), qRootDW, wRel_avg, qVelDW,
                                   &rootControlParams,false);

    ///TODO do so that ffroottorque is counted
    /// or maybe not. not sure ...
    //rootTorque += ffRootTorque;


    //we need to compute the total torque that needs to be applied at the hips so that the final net torque on the root is rootTorque
    Vector3d rootMakeupTorque;
    rootMakeupTorque -= swing_hip_torque  + pelvis_torso_torque;// + stance_hip_torque;
    rootMakeupTorque -= rootTorque;

    /*
    static Vector3d old_value=Vector3d(0,0,0);
    Vector3d test=qVelD;
    Vector3d test2=qVelDW;
    old_value=root->getOrientation().v;
    std::ostringstream oss;
    oss<<"roottorque "<<std::setprecision(3)<<std::fixed<<std::setw(10)<<test.x<<" "<<std::setw(10)<<test.y<<" "<<std::setw(10)<<test.z
      <<"  //  "<<std::setw(10)<<test2.x<<" "<<std::setw(10)<<test2.y<<" "<<std::setw(10)<<test2.z;
    //    Vector3d test3=pelvis_torso_torque;
    //    Vector3d test4=rootTorque;
    //    oss<<std::setprecision(3)<<"  //  "<<std::fixed<<"   "<<std::setw(10)<<test3.x<<" "<<std::setw(10)<<test3.y<<" "<<std::setw(10)<<test3.z
    //      <<"  //  "<<std::setw(10)<<test4.x<<" "<<std::setw(10)<<test4.y<<" "<<std::setw(10)<<test4.z;
    //        oss<<"roottorque "<<rootMakeupTorque.y<<" "<<rootTorque.y<<" "<<test.z;
    std::cout<<oss.str();
    //*/

    //add to the root makeup torque the predictive torque as well (only consider the effect of the torque in the lateral plane).
    //Vector3d rootPredictiveTorque(0, 0, pose_controller->rootPredictiveTorqueScale*9.8*d.x);///WARNING seems strange that there is an x here but well...
    //rootMakeupTorque += getCharacterFrame().rotate(rootPredictiveTorque);

    Vector3d swingHipTorque= swing_hip_torque;
    Vector3d stanceHipTorque = stance_hip_torque;

    //now, based on the ratio of the forces the feet exert on the ground, and the ratio of the forces between the two feet, we will compute the forces that need to be applied
    //to the two hips, to make up the root makeup torque - which means the final torque the root sees is rootTorque!
    stanceHipTorque += rootMakeupTorque * stanceHipToSwingHipRatio * rootStrength;
    swingHipTorque += rootMakeupTorque * (1-stanceHipToSwingHipRatio) * rootStrength;


    //now transform the torque to child coordinates, apply torque limits and then change it back to world coordinates
    Quaternion qStanceHip = character->stance_hip()->child()->getOrientation();
    stanceHipTorque = qStanceHip.getComplexConjugate().rotate(stanceHipTorque);
    limit_torque(&stanceHipTorque, character->stance_hip()->idx());
    stanceHipTorque = qStanceHip.rotate(stanceHipTorque);
    //if (!SimGlobals::foot_flat_on_ground){
    if (!SimGlobals::foot_flat_on_ground){
        stanceHipTorque.y=0;
        if (Globals::use_stance_leg_orientation_controller){
            /*
            //this one works on the leg orientation
            ReducedCharacterState rs(&PoseControlGlobals::desired_pose);
            int hip_idx=character->stance_hip()->idx();
            Quaternion qRel;
            Vector3d vRel;
            ControlParams params = controlParams[hip_idx];
            //get the current relative orientation between the child and parent
            character->getRelativeOrientation(hip_idx, &qRel);
            character->getRelativeAngularVelocity(hip_idx, &vRel);
            Vector3d tq=compute_pd_torque(qRel, rs.getJointRelativeOrientation(hip_idx), vRel,
                                          Vector3d(0,0,0), &params);
            //*/
            //this version works directly with the foot orientation
            Quaternion foot_quat=computeHeading(character->stance_foot()->getOrientation());
            Quaternion target_quat=computeHeading(controlParams[character->stance_foot()->parent_joint()->idx()].char_frame);
            Vector3d foot_ang_speed=character->stance_foot()->getAngularVelocity();
            foot_ang_speed.x=0;
            foot_ang_speed.z=0;
            ControlParams params = controlParams[character->stance_hip()->idx()];
            Vector3d tq=compute_pd_torque(foot_quat, target_quat, foot_ang_speed, Vector3d(0,0,0), &params);

            //            std::cout<<stanceHipTorque.y<<"    "<<tq.y<<std::endl;
            stanceHipTorque.y+=tq.y;
        }
    }

    stanceHipDamping=1;
    stanceHipMaxVelocity=1.0;
    if( stanceHipDamping > 0 ) {
        Vector3d wRel = root->getAngularVelocity() - character->stance_hip()->child()->getAngularVelocity();
        double wRelLen = wRel.length();
        if (wRelLen > stanceHipMaxVelocity ) wRel = wRel * (stanceHipMaxVelocity/wRelLen);
        Vector3d old_tq=stanceHipTorque;
        stanceHipTorque -=  wRel * (stanceHipDamping * wRelLen);
        //        std::cout<<old_tq.y<<" "<<stanceHipTorque.y<<std::endl;
        //        Quaternion foot_quat=computeHeading(character->stance_foot()->getOrientation());
        //        std::cout<<"stance foot angle: "<<foot_quat.getRotationAngle(SimGlobals::up)<< std::endl;
        if (!SimGlobals::foot_flat_on_ground){
            //            std::cout<<stanceHipTorque.y<<std::endl;

        }
    }

    Quaternion qSwingHip = character->swing_hip()->child()->getOrientation();
    swingHipTorque = qSwingHip.getComplexConjugate().rotate(swingHipTorque);
    limit_torque(&swingHipTorque, character->swing_hip()->idx());
    swingHipTorque = qSwingHip.rotate(swingHipTorque);

    //and done...
    stance_hip_torque = stanceHipTorque;
    swing_hip_torque = swingHipTorque;
    /*
    std::ostringstream oss4;
    Vector3d print_vect=stance_hip_torque;
    oss4<<"sht "<<print_vect.x<<"  "<<print_vect.y<<"  "<<print_vect.z;
    std::cout<<oss4.str();
//*/
}


double PoseController::ipm_alt_sagittal(){return swing_foot_controller->get_ipm_alt_sagittal();}
