#include "ODEWorld.h"

#include <Utils/utils.h>
#include <Physics/joints/Joint.h>
#include <Physics/joints/HingeJoint.h>
#include <Physics/joints/UniversalJoint.h>
#include <Physics/joints/BallInSocketJoint.h>
#include <Core/SimGlobals.h>
#include <Globals.h>
#include <Utils/Timer.h>
#include "Core/Character.h"
#include <iostream>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <math.h>
#include "Physics/softbodypointmass.h"
#include "SPlisHSPlasH/Interface.h"

/**
    Default constructor
*/
ODEWorld::ODEWorld(bool i_is_parallel_world) : AbstractRBEngine(){
    int maxCont = 10;
    m_is_parallel_world=i_is_parallel_world;
    //Initialize the world, simulation space and joint groups
    worldID = dWorldCreate();
    spaceID = dSimpleSpaceCreate(0);//dHashSpaceCreate(0);
    contactGroupID = dJointGroupCreate(0);

    //make sure that when we destroy the space group, we destroy all the geoms inside it
    dSpaceSetCleanup(spaceID, 1);

    //set a few of the constants that ODE needs to be aware of
    dWorldSetContactSurfaceLayer(worldID,0.001);							// the ammount of interpenetration allowed between objects
    dWorldSetContactMaxCorrectingVel(worldID, 1.0);							// maximum velocity that contacts are allowed to generate

    //set the gravity...
    Vector3d gravity = SimGlobals::up * SimGlobals::gravity;
    dWorldSetGravity(worldID, gravity.x, gravity.y, gravity.z);

    //allocate the space for the contacts;
    maxContactCount = maxCont;
    cps = new dContact[maxContactCount];

    pcQuery = NULL;

    pcQuery = new PreCollisionQuery();


    //now we initialize ODE
    //    int init_result=dInitODE2(0);
    int init_result=dInitODE2(0);
    int init_2=dAllocateODEDataForThread(dAllocateMaskAll);
    if (init_result == false || init_2 ==false){
        std::cout << "ODE initialisation failed" << std::endl;
    }
    if (m_is_parallel_world){
        init_result=dAllocateODEDataForThread(dAllocateMaskAll);
        if (init_result == false){
            std::cout << "ODE initialisation failed 2" << std::endl;
        }
        m_ode_threading_id=dThreadingAllocateSelfThreadedImplementation();
        dWorldSetStepThreadingImplementation(worldID,dThreadingImplementationGetFunctions(m_ode_threading_id),m_ode_threading_id);
    }


    nb_collisions_cumul=0;
    time_counter=0;

}

/**
    destructor
*/
ODEWorld::~ODEWorld(void){
    odeToRbs.clear();

    if (m_is_parallel_world){
        dWorldSetStepThreadingImplementation(worldID,NULL,NULL);
        dThreadingFreeImplementation(m_ode_threading_id);
    }
    //destroy the ODE physical world, simulation space and joint group
    delete[] cps;
    delete pcQuery;
    dJointGroupDestroy(contactGroupID);
    dSpaceDestroy(spaceID);
    dWorldDestroy(worldID);
    dCloseODE();

}

/**
    This method is used to draw all the rigid bodies in the world
*/
void ODEWorld::drawRBs(int flags){
    for (uint i=0;i<objects.size();i++){
        if (objects[i]->name().find("empty") != std::string::npos ||
                objects[i]->name().find("fillerObj") != std::string::npos) {
            objects[i]->draw(SHOW_CD_PRIMITIVES);
        }else{
            objects[i]->draw(flags);
        }


    }
}


void ODEWorld::loadRBsFromList(std::vector<RigidBody *> vect_rbs)
{
    int cur_nbr_rb=(int)objects.size();
    for (int i=0; i<vect_rbs.size();++i){
        objects.push_back(vect_rbs[i]->cpy());
    }

    //and we init ODE
    initODEStruct(cur_nbr_rb);
}

void ODEWorld::loadARBsFromList(std::vector<ArticulatedRigidBody *> vect_arbs)
{

    //first we create the bodies inside a temporaty buffer
    std::vector<ArticulatedRigidBody*> temp_arbs;
    for (int i=0; i<vect_arbs.size();++i){
        ArticulatedRigidBody* new_arb=vect_arbs[i]->cpy();
        temp_arbs.push_back(new_arb);
    }

    //we create the figure
    ArticulatedFigure* fig=new  ArticulatedFigure(temp_arbs.front());

    //and fill it with the joints
    std::vector<ArticulatedRigidBody*>::iterator it;
    for (int i=0; i<vect_arbs.size();++i){
        Joint* j=vect_arbs[i]->parent_joint();

        //we need to be carefull to the root^^
        if (j==NULL){
            continue;
        }

        //we only create joints that are between 2 elements of the arbs compising the new figure
        it = std::find(vect_arbs.begin(), vect_arbs.end(), j->parent());
        if (it != vect_arbs.end()){
            Joint* j_new=NULL;
            switch (j->get_joint_type()){
                case HINGE_JOINT:{
                    j_new= new HingeJoint(static_cast<HingeJoint*>(j));
                    break;
                }
                case BALL_IN_SOCKET_JOINT:{
                    j_new=new BallInSocketJoint(static_cast<BallInSocketJoint*>(j));
                    break;
                }
                case UNIVERSAL_JOINT:{
                    j_new=new UniversalJoint(static_cast<UniversalJoint*>(j));
                    break;
                }
                default:{
                    throwError("Ooops.... Only BallAndSocket, Hinge and Universal joints are currently supported.\n");
                }
            }
            //and link it all together
            temp_arbs[std::distance(vect_arbs.begin(),it)]->add_link(j_new,temp_arbs[i]);

        }
    }

    //now I only have 1 problem, I'm not sure of the value of the af pointeur inside the
    //new arbs. So I just set it
    for (int i=0; i<temp_arbs.size();++i){
        temp_arbs[i]->af(fig);
    }

    int cur_nbr_rb=(int)objects.size();
    int cur_nbr_af=(int)AFs.size();

    //now just add the objects in the world
    for (int i=0; i<temp_arbs.size();++i){
        objects.push_back(temp_arbs[i]);
        ABs.push_back(temp_arbs[i]);
    }
    AFs.push_back(fig);
    fig->add_joints_to_list(&jts);

    //and we init ODE
    initODEStruct(cur_nbr_rb,cur_nbr_af);
}

void ODEWorld::initODEStruct(int start_RB, int start_AF)
{
    //now we'll make sure that the joint constraints are satisfied
    for (uint i=start_RB;i<objects.size();i++){

        //CREATE AND LINK THE ODE BODY WITH OUR RIGID BODY

        //if the body is fixed, we'll only create the colission detection primitives
        if (objects[i]->isLocked() == true){
            //push in a dummy - never to be used - mapping!!!
            odeToRbs.push_back(ODE_RB_Map(0, objects[i]));
        }else{
            odeToRbs.push_back(ODE_RB_Map(dBodyCreate(worldID), objects[i]));

            ///TODO change all that code to remove the use of that fucking setter
            //the ID of this rigid body will be its index in the
            objects[i]->idx(i);

            //we will use the user data of the object to store the index in this mapping as well, for easy retrieval
            dBodySetData(odeToRbs[i].id, (void*)i);
        }

        //if this is a planar object, make sure we constrain it to always stay planar
        if (objects[i]->props.isPlanar){
            dJointID j = dJointCreatePlane2D(worldID, 0);
            dJointAttach(j, odeToRbs[(int)(objects[i]->idx())].id, 0);
        }

        //PROCESS THE COLLISION PRIMITIVES OF THE BODY
        createODECollisionPrimitives(objects[i]);

        //SET THE INERTIAL PARAMETERS

        if (objects[i]->isLocked() == false){
            dMass m;

            //set the mass and principal moments of inertia for this object
            m.setZero();
            Vector3d principalMoments = odeToRbs[i].rb->getPMI();
            m.setParameters(odeToRbs[i].rb->getMass(), 0, 0, 0,
                            principalMoments.x,
                            principalMoments.y,
                            principalMoments.z,
                            0, 0, 0);

            dBodySetMass(odeToRbs[i].id, &m);

            setODEStateFromRB(i);
        }

    }

    DynamicArray<Joint*> joints;

    //now we will go through all the new joints, and create and link their ode equivalents
    for (uint i=start_AF;i<AFs.size();i++){
        joints.clear();
        AFs[i]->add_joints_to_list(&joints);
        for (uint j=0;j<joints.size();j++){
            //connect the joint to the two bodies
            int jointType = joints[j]->get_joint_type();
            switch (jointType){
                case BALL_IN_SOCKET_JOINT:
                    setupODEBallAndSocketJoint((BallInSocketJoint*)joints[j]);
                    break;
                case HINGE_JOINT:
                    setupODEHingeJoint((HingeJoint*)joints[j]);
                    break;
                case UNIVERSAL_JOINT:
                    setupODEUniversalJoint((UniversalJoint*)joints[j]);
                    break;
                default:
                    throwError("Ooops.... Only BallAndSocket, Hinge and Universal joints are currently supported.\n");
            }
        }
    }
}

void ODEWorld::initODEMassProperties()
{
    for (uint i=0;i<objects.size();i++){


        //SET THE INERTIAL PARAMETERS
        if (objects[i]->isLocked() == false){
            dMass m;

            //set the mass and principal moments of inertia for this object
            m.setZero();
            Vector3d principalMoments = odeToRbs[i].rb->getPMI();
            m.setParameters(odeToRbs[i].rb->getMass(), 0, 0, 0,
                            principalMoments.x,
                            principalMoments.y,
                            principalMoments.z,
                            0, 0, 0);

            dBodySetMass(odeToRbs[i].id, &m);

            setODEStateFromRB(i);
        }

    }
}


/**
    this method is used to copy the state of the ith rigid body to its ode counterpart.
*/
void ODEWorld::setODEStateFromRB(int i){
    if (i<0 || (uint)i>=odeToRbs.size()){
        return;
        std::cout<<"ODEWorld::setODEStateFromRB fail acces to id";
    }

    //if it is a locked object, we won't do anything about it
    if (odeToRbs[i].rb->isLocked() == true)
        return;

    dQuaternion tempQ;
    tempQ[0] = odeToRbs[i].rb->state.orientation.s;
    tempQ[1] = odeToRbs[i].rb->state.orientation.v.x;
    tempQ[2] = odeToRbs[i].rb->state.orientation.v.y;
    tempQ[3] = odeToRbs[i].rb->state.orientation.v.z;

    dBodySetPosition(odeToRbs[i].id, odeToRbs[i].rb->state.position.x, odeToRbs[i].rb->state.position.y, odeToRbs[i].rb->state.position.z);
    dBodySetQuaternion(odeToRbs[i].id, tempQ);
    dBodySetLinearVel(odeToRbs[i].id, odeToRbs[i].rb->state.velocity.x, odeToRbs[i].rb->state.velocity.y, odeToRbs[i].rb->state.velocity.z);
    dBodySetAngularVel(odeToRbs[i].id, odeToRbs[i].rb->state.angular_velocity.x, odeToRbs[i].rb->state.angular_velocity.y, odeToRbs[i].rb->state.angular_velocity.z);
}

/**
    this method is used to copy the state of the ith rigid body, from the ode object to its rigid body counterpart
*/
void ODEWorld::setRBStateFromODE(int i){
    const double *tempData;

    //if it is a locked object, we won't do anything about it
    if (odeToRbs[i].rb->isLocked() == true)
        return;

    //if the objects is supposed to be planar, make sure we don't let drift accumulate
    if (odeToRbs[i].rb->props.isPlanar){
        const dReal *rot = dBodyGetAngularVel(odeToRbs[i].id);
        const dReal *quat_ptr;
        dReal quat[4], quat_len;
        quat_ptr = dBodyGetQuaternion( odeToRbs[i].id );
        quat[0] = quat_ptr[0];
        quat[1] = quat_ptr[1];
        quat[2] = 0;
        quat[3] = 0;
        quat_len = sqrt( quat[0] * quat[0] + quat[1] * quat[1] );
        quat[0] /= quat_len;
        quat[1] /= quat_len;
        dBodySetQuaternion( odeToRbs[i].id, quat );
        dBodySetAngularVel( odeToRbs[i].id, rot[0], 0, 0);
    }

    RigidBody* body=odeToRbs[i].rb;
    Point3d position;
    Quaternion orientation;

    tempData = dBodyGetPosition(odeToRbs[i].id);
    position.x = tempData[0];
    position.y = tempData[1];
    position.z = tempData[2];

    tempData = dBodyGetQuaternion(odeToRbs[i].id);
    orientation.s = tempData[0];
    orientation.v.x = tempData[1];
    orientation.v.y = tempData[2];
    orientation.v.z = tempData[3];



    body->setCMPosition(position);
    body->setOrientation(orientation);

    //backup the d velocties to ba able to compute the accelerations
    Vector3d linear_vel_old=odeToRbs[i].rb->state.velocity;
    Vector3d angular_vel_old=odeToRbs[i].rb->state.angular_velocity;

    //get the velocities from the engine
    tempData = dBodyGetLinearVel(odeToRbs[i].id);
    odeToRbs[i].rb->state.velocity.x = tempData[0];
    odeToRbs[i].rb->state.velocity.y = tempData[1];
    odeToRbs[i].rb->state.velocity.z = tempData[2];

    tempData = dBodyGetAngularVel(odeToRbs[i].id);
    odeToRbs[i].rb->state.angular_velocity.x = tempData[0];
    odeToRbs[i].rb->state.angular_velocity.y = tempData[1];
    odeToRbs[i].rb->state.angular_velocity.z = tempData[2];

    //compute the acceleration
    odeToRbs[i].rb->state.acc= (odeToRbs[i].rb->state.velocity-linear_vel_old)/SimGlobals::dt;
    odeToRbs[i].rb->state.angular_acc= (odeToRbs[i].rb->state.angular_velocity-angular_vel_old)/SimGlobals::dt;

    /*
    if (odeToRbs[i].rb->name()=="pelvis"){
        static Quaternion old_quat;
        Quaternion relative=body->getOrientation()*old_quat.getComplexConjugate();
        Vector3d ang_vel=relative.v/(relative.v.length())*safeACOS(relative.s)*2/SimGlobals::dt;

        static Vector3d old_value=Vector3d(0,0,0);
        Vector3d test=ang_vel;
        Vector3d test2=odeToRbs[i].rb->getAngularVelocity();
        old_quat=body->getOrientation();
        std::ostringstream oss;
        oss<<"roottorque"<<std::setprecision(3)<<std::fixed<<std::setw(10)<<test.x<<" "<<std::setw(10)<<test.y<<" "<<std::setw(10)<<test.z
          <<"  //  "<<std::setw(10)<<test2.x<<" "<<std::setw(10)<<test2.y<<" "<<std::setw(10)<<test2.z;
        //    Vector3d test3=pelvis_torso_torque;
        //    Vector3d test4=rootTorque;
        //    oss<<std::setprecision(3)<<"  //  "<<std::fixed<<"   "<<std::setw(10)<<test3.x<<" "<<std::setw(10)<<test3.y<<" "<<std::setw(10)<<test3.z
        //      <<"  //  "<<std::setw(10)<<test4.x<<" "<<std::setw(10)<<test4.y<<" "<<std::setw(10)<<test4.z;
        //        oss<<"roottorque "<<rootMakeupTorque.y<<" "<<rootTorque.y<<" "<<test.z;
        std::cout<<oss.str();
    }
    //*/
}


/**
    this method is used to set up an ODE sphere geom. NOTE: ODE only allows planes to
    be specified in world coordinates, not attached to a body, so we need to fix it once and
    for all.
*/
dGeomID ODEWorld::getPlaneGeom(PlaneCDP* p, RigidBody* parent){
    //and create the ground plane
    Vector3d n = parent->getWorldCoordinates(p->normal());
    Vector3d o = Vector3d(parent->getWorldCoordinates(p->point()));
    dGeomID g = dCreatePlane(spaceID, n.x, n.y, n.z, o.dotProductWith(n));
    return g;
}

/**
    this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
*/
dGeomID ODEWorld::getSphereGeom(SphereCDP* s){
    dGeomID g = dCreateSphere(0, s->getRadius());
    Point3d c = s->getCenter();
    dGeomSetPosition(g, c.x, c.y, c.z);
    return g;
}


/**
    this method is used to set up an ODE box geom. It is properly placed in body coordinates.
*/
dGeomID ODEWorld::getBoxGeom(BoxCDP* b){
    dGeomID g = dCreateBox(0, b->X_length(), b->Y_length(), b->Z_length());
    Point3d c = b->center();
    dGeomSetPosition(g, c.x, c.y, c.z);
    return g;
}

/**
    this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
*/
dGeomID ODEWorld::getCapsuleGeom(CapsuleCDP* c){
    Point3d a = c->getA();
    Point3d b = c->getB();
    Vector3d ab(a, b);
    dGeomID g = dCreateCCylinder(0, c->radius(), ab.length());

    Point3d cen = a + ab/2.0;
    dGeomSetPosition(g, cen.x, cen.y, cen.z);


    //now, the default orientation for this is along the z-axis. We need to rotate this to make it match the direction
    //of ab, so we need an angle and an axis...
    Vector3d defA(0, 0, 1);

    Vector3d axis = defA.crossProductWith(ab);
    axis.toUnit();
    double rotAngle = defA.angleWith(ab);

    Quaternion relOrientation = Quaternion::getRotationQuaternion(rotAngle, axis);

    dQuaternion q;
    q[0] = relOrientation.s;
    q[1] = relOrientation.v.x;
    q[2] = relOrientation.v.y;
    q[3] = relOrientation.v.z;

    dGeomSetQuaternion(g, q);

    return g;
}

/**
    This method is used to set up an ode hinge joint, based on the information in the hinge joint passed in as a parameter
*/
dJointID ODEWorld::setupODEFixedJoint(RigidBody* bd1,RigidBody* bd2){
    dJointID j = dJointCreateFixed(worldID, 0);

    dJointAttach(j, odeToRbs[(int)(bd1->idx())].id, odeToRbs[(int)(bd2->idx())].id);
    dJointSetFixed(j);

    return j;
}

void ODEWorld::destroyODEFixedJoint(dJointID joint_id)
{
    dJointDestroy(joint_id);
}


/**
    This method is used to set up an ode hinge joint, based on the information in the hinge joint passed in as a parameter
*/
void ODEWorld::setupODEHingeJoint(HingeJoint* hj){
    dJointID j = dJointCreateHinge(worldID, 0);
    dJointAttach(j, odeToRbs[(int)(hj->child()->idx())].id, odeToRbs[(int)(hj->parent()->idx())].id);
    Point3d p = hj->child()->getWorldCoordinates(hj->child_joint_position());
    dJointSetHingeAnchor(j, p.x, p.y, p.z);
    Vector3d a = hj->parent()->getWorldCoordinates(hj->axis());
    dJointSetHingeAxis(j, a.x, a.y, a.z);

    //now set the joint limits
    if (hj->use_joint_limits() == false)
        return;

    dJointSetHingeParam(j, dParamLoStop, hj->min_angle());
    dJointSetHingeParam(j, dParamHiStop, hj->max_angle());
}

/**
    This method is used to set up an ode universal joint, based on the information in the universal joint passed in as a parameter
*/
void ODEWorld::setupODEUniversalJoint(UniversalJoint* uj){
    dJointID j = dJointCreateUniversal(worldID, 0);
    dJointAttach(j, odeToRbs[(int)(uj->child()->idx())].id, odeToRbs[(int)(uj->parent()->idx())].id);
    Point3d p = uj->child()->getWorldCoordinates(uj->child_joint_position());
    dJointSetUniversalAnchor(j, p.x, p.y, p.z);

    Vector3d a = uj->parent()->getWorldCoordinates(uj->a);
    Vector3d b = uj->child()->getWorldCoordinates(uj->b);

    dJointSetUniversalAxis1(j, a.x, a.y, a.z);
    dJointSetUniversalAxis2(j, b.x, b.y, b.z);

    //now set the joint limits
    if (uj->use_joint_limits() == false)
        return;

    dJointSetUniversalParam(j, dParamLoStop, uj->minAngleA);
    dJointSetUniversalParam(j, dParamHiStop, uj->maxAngleA);
    dJointSetUniversalParam(j, dParamLoStop2, uj->minAngleB);
    dJointSetUniversalParam(j, dParamHiStop2, uj->maxAngleB);
}

/**
    This method is used to set up an ode ball-and-socket joint, based on the information in the ball in socket joint passed in as a parameter
*/
void ODEWorld::setupODEBallAndSocketJoint(BallInSocketJoint* basj){
    dJointID j = dJointCreateBall(worldID, 0);
    dJointAttach(j, odeToRbs[(int)(basj->child()->idx())].id, odeToRbs[(int)(basj->parent()->idx())].id);
    Point3d p = basj->child()->getWorldCoordinates(basj->child_joint_position());
    //now we'll set the world position of the ball-and-socket joint. It is important that the bodies are placed in the world
    //properly at this point
    dJointSetBallAnchor(j, p.x, p.y, p.z);

    //now deal with the joint limits
    if (basj->use_joint_limits() == false)
        return;

    Vector3d a = basj->parent()->getWorldCoordinates(basj->swingAxis1);
    Vector3d b =  basj->child()->getWorldCoordinates(basj->twistAxis);

    //we'll assume that:
    //b is the twisting axis of the joint, and the joint limits will be (in magnitude) less than 90 degrees, otherwise
    //the simulation will go unstable!!!


    dJointID aMotor = dJointCreateAMotor(worldID, 0);
    dJointAttach(aMotor, odeToRbs[(int)(basj->parent()->idx())].id, odeToRbs[(int)(basj->child()->idx())].id);
    dJointSetAMotorMode(aMotor, dAMotorEuler);

    dJointSetAMotorParam(aMotor, dParamStopCFM, 0.1);
    dJointSetAMotorParam(aMotor, dParamStopCFM2, 0.1);
    dJointSetAMotorParam(aMotor, dParamStopCFM3, 0.1);


    dJointSetAMotorAxis (aMotor, 0, 1, a.x, a.y, a.z);
    dJointSetAMotorAxis (aMotor, 2, 2, b.x, b.y, b.z);

    dJointSetAMotorParam(aMotor, dParamLoStop, basj->minSwingAngle1);
    dJointSetAMotorParam(aMotor, dParamHiStop, basj->maxSwingAngle1);

    dJointSetAMotorParam(aMotor, dParamLoStop2, basj->minSwingAngle2);
    dJointSetAMotorParam(aMotor, dParamHiStop2, basj->maxSwingAngle1);

    dJointSetAMotorParam(aMotor, dParamLoStop3, basj->minTwistAngle);
    dJointSetAMotorParam(aMotor, dParamHiStop3, basj->maxTwistAngle);
}

/**
    this method is used to create ODE geoms for all the collision primitives of the rigid body that is passed in as a paramter
*/
void ODEWorld::createODECollisionPrimitives(RigidBody* body){
    //now we'll set up the body's collision detection primitives
    for (uint j=0;j<body->cdps.size();j++){
        int cdpType = body->cdps[j]->type();

        //depending on the type of collision primitive, we'll now create g.
        dGeomID g;

        switch (cdpType){
            case SPHERE_CDP:
                g = getSphereGeom((SphereCDP*)body->cdps[j]);
                break;
            case CAPSULE_CDP:
                g = getCapsuleGeom((CapsuleCDP*)body->cdps[j]);
                break;
            case BOX_CDP:
                g = getBoxGeom((BoxCDP*)body->cdps[j]);
                break;
            case PLANE_CDP:
                //NOTE: only static objects can have planes as their collision primitives - if this isn't static, force it!!
                g = getPlaneGeom((PlaneCDP*)body->cdps[j], body);
                break;
            default:
                throwError("Ooppps... No collision detection primitive was created rb: %s, cdp: %d", body->name(), j);
        }

        //now associate the geom to the rigid body that it belongs to, so that we can look up the properties we need later...
        dGeomSetData(g, body);

        //if it's a plane, it means it must be static, so we can't attach a transform to it...
        if (cdpType == PLANE_CDP){
            ground_geom_id=g;
            continue;
        }

        //now we've created a geom for the current body. Note: g will be rotated relative to t, so that it is positioned
        //well in body coordinates, and then t will be attached to the body.
        dGeomID t = dCreateGeomTransform(spaceID);
        //make sure that when we destroy the transfromation, we destroy the encapsulated objects as well.
        dGeomTransformSetCleanup(t, 1);

        //associate the transform geom with the body as well
        dGeomSetData(t, body);

        //if the object is fixed, then we want the geometry to take into account the initial position and orientation of the rigid body
        if (body->isLocked() == true){
            dGeomSetPosition(t, body->state.position.x, body->state.position.y, body->state.position.z);
            dQuaternion q;
            q[0] = body->state.orientation.s;
            q[1] = body->state.orientation.v.x;
            q[2] = body->state.orientation.v.y;
            q[3] = body->state.orientation.v.z;
            dGeomSetQuaternion(t, q);
        }

        dGeomTransformSetGeom(t, g);
        //now add t (which contains the correctly positioned geom) to the body, if we do really have an ODE body for it
        if (body->isLocked() == false)
            dGeomSetBody(t, odeToRbs[body->idx()].id);
    }
}


/**
    This method reads a list of rigid bodies from the specified file.
*/
void ODEWorld::loadRBsFromFile(char* fName){
    //make sure we don't go over the old articulated figures in case this method gets called multiple times.
    int index = objects.size();
    int index_afs = AFs.size();

    AbstractRBEngine::loadRBsFromFile(fName);

    initODEStruct(index,index_afs);
}

/**
    this method is used to process the collision between the two objects passed in as parameters. More generally,
    it is used to determine if the collision should take place, and if so, it calls the method that generates the
    contact points.
*/
void ODEWorld::processCollisions(dGeomID o1, dGeomID o2){
    dBodyID b1, b2;
    RigidBody *rb1, *rb2;
    b1 = dGeomGetBody(o1);
    b2 = dGeomGetBody(o2);
    rb1 = (RigidBody*) dGeomGetData(o1);
    rb2 = (RigidBody*) dGeomGetData(o2);

    //bool joined = b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact);

    if (pcQuery){
        if (pcQuery->shouldCheckForCollisions(rb1, rb2) == false)
            return;
    }


    //this mean that the ollisions is between 2 things tha are not the ground...
    //or the ball ...
    //this is used so that the sole have no collision with the foot and that the body parts don't collide with each other
    if (rb1->name()!="ground"&&rb2->name()!="ground"&&
            rb1->name()!="dodgeBall"&&rb2->name()!="dodgeBall"&&
            rb1->name()!="fillerObj"&&rb2->name()!="fillerObj"){

        return;
    }

    //*
    for (int i=0;i<(int)vect_soft_bodies_point_mass.size();++i)
        if (((rb1==vect_soft_bodies_point_mass[i]->rb)&&(rb2==vect_soft_bodies_point_mass[i]->rb_ground))||
                ((rb2==vect_soft_bodies_point_mass[i]->rb)&&(rb1==vect_soft_bodies_point_mass[i]->rb_ground))){
            //those collisions are handled somewhere else
            //std::cout<<"detected";
            return;
        }
    //*/

    //we'll use the minimum of the two coefficients of friction of the two bodies.
    double mu1 = rb1->getFrictionCoefficient();
    double mu2 = rb2->getFrictionCoefficient();
    double mu_to_use = std::min(mu1, mu2);
    double eps1 = rb1->getRestitutionCoefficient();
    double eps2 = rb2->getRestitutionCoefficient();
    double eps_to_use = std::min(eps1, eps2);



    int num_contacts = dCollide(o1,o2,maxContactCount,&(cps[0].geom), sizeof(dContact));

    double groundSoftness = 0, groundPenalty = 0;
    if (rb1){
        groundSoftness = rb1->props.groundSoftness;
        groundPenalty = rb1->props.groundPenalty;
    }else{
        groundSoftness = rb2->props.groundSoftness;
        groundPenalty = rb2->props.groundPenalty;
    }


    //fill in the missing properties for the contact points
    for (int i=0;i<num_contacts;i++){
        cps[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 |dContactRolling;//| dContactBounce;
        cps[i].surface.mu = mu_to_use;
        //        cps[i].surface.mu = 2000.0;
        //        cps[i].surface.bounce = eps_to_use;
        //        cps[i].surface.bounce_vel = 0.00001;

        cps[i].surface.soft_cfm = groundSoftness;
        cps[i].surface.soft_erp = groundPenalty;

        /*
        //this is a tst to have a stronger friction on the feet
        if(rb1->name()=="lFoot"||rb1->name()=="rFoot"){
            Point3d p(cps[i].geom.pos[0], cps[i].geom.pos[1], cps[i].geom.pos[2]);
            p=rb1->getLocalCoordinates(p);
            if (p.y<-0.0335){
                cps[i].surface.bounce =0;
                cps[i].surface.soft_cfm =1;
                cps[i].surface.mu=10000;
                //std::cout<<"using it"<<p.y;
            }
        }


        if(rb2->name()=="lFoot"||rb2->name()=="rFoot"){
            Point3d p(cps[i].geom.pos[0], cps[i].geom.pos[1], cps[i].geom.pos[2]);
            p=rb2->getLocalCoordinates(p);
            if (p.y<-0.0335){
                cps[i].surface.bounce =0;
                cps[i].surface.soft_cfm =1;
                cps[i].surface.mu=10000;
                //std::cout<<"using it"<<p.y;
            }

        }
        //*/
    }

    // and now add them contact points to the simulation
    for (int i=0;i<num_contacts;i++){
        //create a joint, and link the two geometries.
        dJointID c = dJointCreateContact(worldID, contactGroupID, &cps[i]);
        dJointAttach(c, b1, b2);

        if (cps[i].surface.soft_cfm>0.5 ){
            continue;
        }

        if (jointFeedbackCount >= MAX_CONTACT_FEEDBACK)
            std::cerr<<"Warning: too many contacts are established. "
                    <<"Some of them will not be reported.\n"<<std::endl;
        else{
            if (contactPoints.size() != jointFeedbackCount){
                std::cerr<<"Warning: Contact forces need to be cleared after each simulation,"
                        <<" otherwise the results are not predictable.\n"<<std::endl;
            }
            contactPoints.push_back(ContactPoint());
            //now we'll set up the feedback for this contact joint
            contactPoints[jointFeedbackCount].rb1 = rb1;
            contactPoints[jointFeedbackCount].rb2 = rb2;
            contactPoints[jointFeedbackCount].cp = Point3d(cps[i].geom.pos[0], cps[i].geom.pos[1], cps[i].geom.pos[2]);
            dJointSetFeedback(c,&(jointFeedback[jointFeedbackCount]));
            jointFeedbackCount++;
        }

        if (rb1->name()=="fillerObj"||rb2->name()=="fillerObj"){
            nb_collisions_cumul++;
        }
    }
}

/**
    This method is used to set the state of all the rigid body in this collection.
*/
void ODEWorld::setState(DynamicArray<double>* state, int start){
    AbstractRBEngine::setState(state, start);
}

/**
    This method is a simple call back function that passes the message to the world whose objects are being acted upon.
*/
void collisionCallBack(void* odeWorld, dGeomID o1, dGeomID o2){
    ((ODEWorld*)odeWorld)->processCollisions(o1, o2);
}


void ODEWorld::sendDataToEngine(RigidBody *root_bdy, Vector3d sup_root_torque){
    //make sure that the state of the RB's is synchronized with the engine...
    setEngineStateFromRB();

    //restart the counter for the joint feedback terms
    jointFeedbackCount = 0;

    //go through all the joints in the world, and apply their torques to the parent and child rb's
    for (uint j = 0; j<jts.size(); j++){
        Vector3d t = jts[j]->torque();


        if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
            std::cout<<"ODEWorld::advanceInTime::  undefinned torques   "<<jts[j]->name();

        }


        //we will apply to the parent a positive torque, and to the child a negative torque
        dBodyAddTorque(odeToRbs[jts[j]->parent()->idx()].id, t.x, t.y, t.z);
        dBodyAddTorque(odeToRbs[jts[j]->child()->idx()].id, -t.x, -t.y, -t.z);
    }

    if (root_bdy!=NULL){
        Vector3d t=sup_root_torque;
        dBodyAddTorque(odeToRbs[root_bdy->idx()].id, -t.x, -t.y, -t.z);
    }

    //clear the previous list of contact forces
    contactPoints.clear();

    //*
    RigidBody* rb=getRBByName("dodgeBall");
    if (rb!=NULL){
        if (vect_soft_bodies_point_mass.empty()){
            double km=1E4;
            SoftBodyPointMass* sbpm;
            /*
            sbpm=new SoftBodyPointMass(rb,this,km,0.01);
            sbpm->init(3);
//            vect_soft_bodies_point_mass.push_back(sbpm);
            //*/
            /*
            km=1.5E4;
            rb=getRBByName("rFoot");
            sbpm=new SoftBodyPointMass(rb,this,km);
            sbpm->init(3);
            vect_soft_bodies_point_mass.push_back(sbpm);


            rb=getRBByName("lFoot");
            sbpm=new SoftBodyPointMass(rb,this,km,0.01);
            sbpm->init(3);
            vect_soft_bodies_point_mass.push_back(sbpm);
            //*/
        }
    }
    //*/

    //update the target pos
    for (int i=0;i<(int)vect_soft_bodies_point_mass.size();++i){
        vect_soft_bodies_point_mass[i]->update_target_position();
    }
    //*/

    // applyForceTo(objects.back(),Vector3d(0,0,1),Point3d(0,0,0));

    /*
    for (int i=0;i<vect_container_cubes.size();++i){
        TimeCube &tc=vect_container_cubes[i];
        RigidBody* rb=tc.cube;
        if (rb->getAngularVelocity().x<1){
            applyTorqueTo(rb,Vector3d(10,0,0));
        }

        applyForceTo(rb,Vector3d(0,-SimGlobals::gravity,0),Point3d(0,0,0));
    }
    //*/
}

dBodyID ODEWorld::get_body_id(RigidBody *rb)
{
    for (int i=0;i<odeToRbs.size();++i){
        if (odeToRbs[i].rb==rb){
            return odeToRbs[i].id;
        }
    }

    throw("there is no such rigid body existing");

    //just so that it compile
    dBodyID blanc_id;
    return blanc_id;
}

/**
    This method is used to integrate the forward simulation in time.
*/
void ODEWorld::advanceInTime(double deltaT){


    //do the rigib bodies simulation
    //SimGlobals::nb_collisons_event=0;
    SimGlobals::nb_active_objects=vect_container_cubes.size()*(1+SimGlobals::nb_filler);

    if (time_counter>1.0){
        SimGlobals::nb_collisons_event=nb_collisions_cumul/(time_counter/deltaT);
        time_counter=0;
        nb_collisions_cumul=0;
    }
    time_counter+=deltaT;

    //we need to determine the contact points first - delete the previous contacts
    dJointGroupEmpty(contactGroupID);
    //initiate the collision detection
    dSpaceCollide(spaceID, this, &collisionCallBack);
    //*

    /*
    RigidBody* rb=getRBByName("dodgeBall");
    if (rb!=NULL){
        applyForceTo(rb,Vector3d(0,rb->getMass()*SimGlobals::gravity,0),Point3d(0,1,0.5));
    }
    //*/


    for (int i=0;i<(int)vect_soft_bodies_point_mass.size();++i){
        vect_soft_bodies_point_mass[i]->generate_contacts();
    }

    //*/
    //advance the simulation
    try{
        dWorldStep(worldID, deltaT);
    }catch(char* c){
        throw(c);
    }

    //dWorldQuickStep(worldID, deltaT);


}

void ODEWorld::readDataFromEngine(){
    //copy over the state of the ODE bodies to the rigid bodies...
    setRBStateFromEngine();

    //copy over the force information for the contact forces
    for (int i=0;i<jointFeedbackCount;i++){
        contactPoints[i].f = Vector3d(jointFeedback[i].f1[0], jointFeedback[i].f1[1], jointFeedback[i].f1[2]);
        //make sure that the force always points away from the static objects
        if (contactPoints[i].rb1->isLocked() && !contactPoints[i].rb2->isLocked()){
            contactPoints[i].f = contactPoints[i].f * (-1);
            RigidBody* tmpBdy = contactPoints[i].rb1;
            contactPoints[i].rb1 = contactPoints[i].rb2;
            contactPoints[i].rb2 = tmpBdy;
        }
    }

    for (int i=0;i<(int)vect_soft_bodies_point_mass.size();++i){
        vect_soft_bodies_point_mass[i]->forward_mass_points(2);
    }

}

void ODEWorld::sendDataToParticleFluidEngine(){
    //send the rb info to the fluid engine
    //*
    std::vector<SPH::DynamicBody> vect_dynamic_bodies_info;

    for (int i=0;i<vect_objects_fluid_interaction.size();++i){
        RigidBody* body=vect_objects_fluid_interaction[i];
        SPH::DynamicBody arrayInfo;
         arrayInfo.position=Interface::vector3dToSPH3d(body->getCMPosition());
         arrayInfo.velocity=Interface::vector3dToSPH3d(body->getCMVelocity());
         arrayInfo.q=Interface::quaternionToSPHQuaternion(body->getOrientation());
         arrayInfo.angular_vel=Interface::vector3dToSPH3d(body->getAngularVelocity());

        vect_dynamic_bodies_info.push_back(arrayInfo);

        Quaternion q=body->getOrientation();
    }

    Interface::updateDynamicBodies(vect_dynamic_bodies_info);
}
void ODEWorld::advanceInTimeParticleFluidEngine(double deltaT){

    //do the particle fluid simulation
    Interface::updateTimeStepDuration(deltaT);
    Interface::fluidSimulationStep();

    //now Ill get other values that will represent the boyant force
    std::vector<Vector3d> forces_b;
    std::vector<Point3d> pts_appli_b;
    Interface::getFluidBoyancyOnDynamicBodies(forces_b,pts_appli_b);

}

void ODEWorld::readDataFromParticleFluidEngine(WaterImpact &resulting_impact){

    //I'll apply a reduction factor because it is wayyyyyyyyy too high
    double reduction_factor=0.25;

    //resize the data structure
    resulting_impact.init(getRBCount());

    //now Ill get other values that will represent the boyant force
    std::vector<Vector3d> forces_b;
    std::vector<Point3d> pts_appli_b;
    Interface::getFluidBoyancyOnDynamicBodies(forces_b,pts_appli_b);

    for (int i=0;i<vect_objects_fluid_interaction.size();++i){
        RigidBody* body=vect_objects_fluid_interaction[i];

        ForceStruct impact;
        impact.pt=pts_appli_b[i];
        impact.F=forces_b[i]*reduction_factor;
        impact.M=Vector3d(0,0,0);
        resulting_impact.impact_boyancy[body->idx()]=impact;
    }




    std::vector<Vector3d> forces, moments;
    Interface::getFluidImpactOnDynamicBodies(forces, moments);

    for (int i=0;i<vect_objects_fluid_interaction.size();++i){
        RigidBody* body=vect_objects_fluid_interaction[i];

        ForceStruct impact;
        impact.pt=body->getCMPosition();
        impact.F=forces[i]*reduction_factor;
        impact.M=moments[i]*reduction_factor;

        //only apply the forces if the rigid bodies are animated
        if (!Globals::simulateOnlyFluid){
            if (!impact.F.isZeroVector()){
                applyForceTo(body,impact.F,body->getLocalCoordinates(impact.pt));
            }
            if (!impact.M.isZeroVector()){
                applyTorqueTo(body,impact.M);
            }
        }

        resulting_impact.impact_drag[body->idx()]=impact;

    }



    //std::cout<<"sum force automatic : "<<forces[0].x<<" "<<forces[0].y<<" "<<forces[0].z<<std::endl;

}


/**
    this method is used to transfer the state of the rigid bodies, from ODE to the rigid body wrapper
*/
void ODEWorld::setRBStateFromEngine(){
    //now update all the rigid bodies...
    for (uint i=0;i<objects.size();i++){
        setRBStateFromODE(i);
        //		objects[i]->updateToWorldTransformation();
    }
}

/**
    this method is used to transfer the state of the rigid bodies, from the rigid body wrapper to ODE's rigid bodies
*/
void ODEWorld::setEngineStateFromRB(){
    //now update all the rigid bodies...
    RigidBody* lf;
    RigidBody* rf;

    for (uint i=0;i<objects.size();i++){
        RigidBody* rb=objects[i];
        if (rb->name()=="lFoot"){
            lf=rb;
        }
        if (rb->name()=="rFoot"){
            rf=rb;
        }

    }

    for (uint i=0;i<objects.size();i++){
        RigidBody* rb=objects[i];


       // if (dynamic_cast<ArticulatedRigidBody*>(objects[i])!=NULL){
            setODEStateFromRB(i);
       // }
    }
}

void ODEWorld::create_contact(dContact *cps_bd, int contact_idx, dBodyID bd1_id, dBodyID bd2_id, RigidBody *rb1, RigidBody *rb2)
{
    dJointID c = dJointCreateContact(worldID, contactGroupID, &cps_bd[contact_idx]);
    dJointAttach(c, bd1_id, bd2_id);

    contactPoints.push_back(ContactPoint());
    //now we'll set up the feedback for this contact joint
    contactPoints[jointFeedbackCount].rb1 = rb1;
    contactPoints[jointFeedbackCount].rb2 = rb2;
    contactPoints[jointFeedbackCount].cp = Point3d(cps_bd[contact_idx].geom.pos[0],
            cps_bd[contact_idx].geom.pos[1], cps_bd[contact_idx].geom.pos[2]);
    dJointSetFeedback(c,&(jointFeedback[jointFeedbackCount]));
    jointFeedbackCount++;
}

/**
    this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
    and the force is also specified in local coordinates.
*/
void ODEWorld::applyRelForceTo(RigidBody* b, const Vector3d& f, const Point3d& p){
    if (!b)
        return;
    dBodyAddRelForceAtRelPos(odeToRbs[b->idx()].id, f.x, f.y, f.z, p.x, p.y, p.z);
}

/**
    this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
    and the force is specified in world coordinates.
*/
void ODEWorld::applyForceTo(RigidBody* b, const Vector3d& f, const Point3d& p){
    if (!b)
        return;
    dBodyAddForceAtRelPos(odeToRbs[b->idx()].id, f.x, f.y, f.z, p.x, p.y, p.z);
}

void ODEWorld::applyForceToWorldPos(RigidBody* b, const Vector3d& f, const Point3d& p){
    if (!b)
        return;
    dBodyAddForceAtPos(odeToRbs[b->idx()].id, f.x, f.y, f.z, p.x, p.y, p.z);
}


/**
    this method applies a torque to a rigid body. The torque is specified in world coordinates.
*/
void ODEWorld::applyTorqueTo(RigidBody* b, const Vector3d& t){
    if (!b)
        return;
    dBodyAddTorque(odeToRbs[b->idx()].id, t.x, t.y, t.z);
}

/**
    this method is used to compute the effect of water (it convert a level of water into the induced forces
    this version only work for the bipV2 caracter
*/
void ODEWorld::compute_water_impact(Character* character, float water_level, WaterImpact &resulting_impact){

    //resize the data structure
    resulting_impact.init(getRBCount());

    //first I check if the water have any density (if not it's useless to try anything)
    if (IS_ZERO(SimGlobals::liquid_density)||IS_ZERO(SimGlobals::water_level)){
        return;
    }


    //*
    std::vector<Joint*> lower_body;
    if (lower_body.empty()){
        character->getCharacterBottom(lower_body);
    }




    double friction_factor = SimGlobals::liquid_density/(SimGlobals::force_alpha*1E-3);
    double drag_density = SimGlobals::liquid_density*SimGlobals::liquid_viscosity;

    for (uint i = 0; i < lower_body.size(); ++i){
        Joint* joint = lower_body[i];
        RigidBody* body = joint->child();
        std::string bname=body->name();


        ForceStruct impact_drag;
        ForceStruct impact_boyancy;

        //*
        //don't bother with the toes they are not large enougth to have any significative impact
        if (bname=="rToes"){
            //impact_drag = compute_liquid_drag_on_toes(body, water_level, drag_density);
            //impact_boyancy = compute_buoyancy(body, water_level);
        }
        else if (bname=="lToes"){
            //impact_drag = compute_liquid_drag_on_toes(body, water_level, drag_density);
            //impact_boyancy = compute_buoyancy(body, water_level);
        }else if (bname=="rFoot"){
            impact_drag = compute_liquid_drag_on_feet(body, water_level, drag_density, friction_factor);
            impact_boyancy = compute_buoyancy(body, water_level);
        }
        else if (bname=="lFoot"){
            impact_drag = compute_liquid_drag_on_feet(body, water_level, drag_density, friction_factor);
            impact_boyancy = compute_buoyancy(body, water_level);
        }
        else if (bname=="lLowerleg"){
            impact_drag = compute_liquid_drag_on_legs(body, water_level, drag_density, friction_factor);
            impact_boyancy = compute_buoyancy(body, water_level);
        }
        else if (bname=="rLowerleg"){
            impact_drag = compute_liquid_drag_on_legs(body, water_level, drag_density, friction_factor);
            impact_boyancy = compute_buoyancy(body, water_level);
        }
        else if (bname=="lUpperleg"){
            impact_drag = compute_liquid_drag_on_legs(body, water_level, drag_density, friction_factor);
            impact_boyancy = compute_buoyancy(body, water_level);
        }
        else if (bname=="rUpperleg"){
            impact_drag = compute_liquid_drag_on_legs(body, water_level, drag_density, friction_factor);
            impact_boyancy = compute_buoyancy(body, water_level);
        }
        else{
            //should be impossible with the current system
            //I'll just shut down the application if this happens bouahahahahha
            exit(985632);
        }
        //*/

        if (!impact_boyancy.isZero()){

            impact_drag.modifyApplicationPoint(body->getCMPosition());
            impact_boyancy.modifyApplicationPoint(body->getCMPosition());

            //*
            applyForceTo(body, impact_drag.F, body->getLocalCoordinates(impact_drag.pt));
            if (!impact_drag.M.isZeroVector()){
                //applyTorqueTo(body,impact_drag.M);
            }

            applyForceTo(body, impact_boyancy.F, body->getLocalCoordinates(impact_boyancy.pt));
            if (!impact_boyancy.M.isZeroVector()){
                //applyTorqueTo(body,impact_boyancy.M);
            }
            //*/



            resulting_impact.impact_drag[body->idx()]=impact_drag;
            resulting_impact.impact_boyancy[body->idx()]=impact_boyancy;



        }


    }
    //*/

    resulting_impact.check_for_undefined();
}



/**
this function is a children function of the above one (it prevent mass duplication of code for similar body parts
this function handle the toes
*/
ForceStruct ODEWorld::compute_liquid_drag_on_toes(RigidBody *body, float water_level, double eff_density){

    throw("ODEWorld::compute_liquid_drag_on_toes we don't use a sphere for the toes anymore and so this function must be changed");

    CollisionDetectionPrimitive* cdp = body->cdps.front();
    SphereCDP* sphere = dynamic_cast<SphereCDP*>(cdp);

    if (sphere == NULL){
        throw("the toes should only have a sphere primitive...");
        return ForceStruct();
    }

    double dy = water_level - body->getCMPosition().getY();

    ForceStruct drag_impact;

    //we vrify that the water hit the ball before doing anything
    if (dy + sphere->getRadius()>0){
        //now I want to determine the surface that face the speed

        int nbr_interval_r = 3;
        double dr = sphere->getRadius() / nbr_interval_r;

        int nbr_interval_t = 10;
        double dt = 2 * PI / nbr_interval_t;


        //we need the speed in world coordinates
        //since the sphere is realy small for the toes we approxiate the speed as beeing constant through the object
        ///TODO remove that approximation
        Vector3d V = body->getCMVelocity();
        Vector3d v_norm = V / V.length();

        Vector3d u, v;//those will be the 2 unitary vector of the disc plane
        v_norm.getOrthogonalVectors(&u, &v);//get the value of the 2 unitary vector

        //since idk if they are normed I'll norm them to be sure
        u /= u.length();
        v /= v.length();

        Point3d center = body->getWorldCoordinates(sphere->getCenter());

        double cur_r = dr;
        double cur_t = 0;

        double S = 0;

        for (int i = 0; i < nbr_interval_r; ++i){
            cur_t = 0;

            //we now calculate the alpha for the current r
            double alpha = std::sqrt(sphere->getRadius()*sphere->getRadius() - cur_r*cur_r);
            alpha /= (v_norm.x + v_norm.y + v_norm.z);


            for (int j = 0; j < nbr_interval_t; ++j){
                //now we need to test to see if we have to consider this segment for the calculation

                //first we determine the wolrld coordinate of the current point
                Point3d p = center + u*std::cos(cur_t)*cur_r + v*std::sin(cur_t)*cur_r;

                //now I need to know if the corresponding point on the spere is affected by the water
                //since the water level only depnds on the y coordinate I only need to compute it (nvm the x and z)
                double y = p.y + alpha*v_norm.y;

                if (y < water_level){
                    //I need to calculate the area and add count it
                    double ds = cur_r*dr*dt;
                    S += ds;
                }
            }
        }

        //now that we have the surface we can compute the resulting force
        Vector3d F = -V*V.length() * 1 / 2 * eff_density*S;


        drag_impact.pt=body->getWorldCoordinates(sphere->getCenter());
        drag_impact.F=F;

    }

    return drag_impact;
}

/**
this function is a children function of the above one (it prevent mass duplication of code for similar body parts)
this function handle the feet
*/
ForceStruct ODEWorld::compute_liquid_drag_on_feet(RigidBody *body, float water_level, double eff_density, double friction_coef){

    CollisionDetectionPrimitive* cdp = body->cdps.front();
    BoxCDP* box = dynamic_cast<BoxCDP*>(cdp);

    if (box == NULL){
        //throwError("the toes should only have a sphere primitive...");
        return ForceStruct();
    }

    //I want the lower points to find out if the box is in the water
    //I call Z the vertical axis but in this world representation the vertical axis is actualy Y...

    Point3d center = box->center();
    Point3d corners[8];
    corners[0] = center + Point3d(box->X_length() / 2, box->Y_length() / 2, -box->Z_length() / 2);
    corners[1] = center + Point3d(box->X_length() / 2, -box->Y_length() / 2, -box->Z_length() / 2);
    corners[2] = center + Point3d(-box->X_length() / 2, box->Y_length() / 2, -box->Z_length() / 2);
    corners[3] = center + Point3d(-box->X_length() / 2, -box->Y_length() / 2, -box->Z_length() / 2);
    corners[4] = center + Point3d(box->X_length() / 2, box->Y_length() / 2, box->Z_length() / 2);
    corners[5] = center + Point3d(box->X_length() / 2, -box->Y_length() / 2, box->Z_length() / 2);
    corners[6] = center + Point3d(-box->X_length() / 2, box->Y_length() / 2, box->Z_length() / 2);
    corners[7] = center + Point3d(-box->X_length() / 2, -box->Y_length() / 2, box->Z_length() / 2);


    double miny = body->getWorldCoordinates(corners[0]).getY();
    Point3d wcorners[8];

    for (int i = 0; i < 8; ++i){
        wcorners[i] = body->getWorldCoordinates(corners[i]);
        if (wcorners[i].getY() < miny){
            miny = wcorners[i].getY();
        }
    }

    ForceStruct drag_impact(body->getCMPosition());

    //we vrify that the water hit the ball before doing anything
    if (miny<water_level){
        //now I'll subdivide the faces in smaller surfaces and apply the necessary force on each of them

        //I'll have to handle each face in a diffenrent way
        //but i'll use these variable (well 2 of them) in every face so I'll create them here
        int nbr_interval_x = 3;
        int nbr_interval_y = 2;
        int nbr_interval_z = 9;

        double l_x = box->X_length();
        double l_y = box->Y_length();
        double l_z = box->Z_length();
        double d_x = l_x / nbr_interval_x;
        double d_y = l_y / nbr_interval_y;
        double d_z = l_z / nbr_interval_z;
        Point3d cur_pos,cur_normal;

        ///TODO optimize this so we star with the lowest corner
        /*
        //first let's handle the back face
        cur_pos = center + Point3d(-box->X_length() / 2 + d_x/2, -box->Y_length() / 2 + d_y/2, -box->Z_length() / 2);
        cur_normal = Point3d(0, 0, -1);
        drag_torque+=compute_liquid_drag_on_plane(joint, box->X_length(), box->Y_length(), box->Z_length(),
            cur_pos, cur_normal, water_level, nbr_interval_x, nbr_interval_y, 0);

        //now the front face
        cur_pos = center + Point3d(-box->X_length() / 2 + d_x / 2, -box->Y_length() / 2 + d_y / 2, box->Z_length() / 2);
        cur_normal = Point3d(0, 0, 1);
        drag_torque+=compute_liquid_drag_on_plane(joint, box->X_length(), box->Y_length(), box->Z_length(),
            cur_pos, cur_normal, water_level, nbr_interval_x, nbr_interval_y, 0);

        //now the left face
        cur_pos = center + Point3d(box->X_length() / 2, -box->Y_length() / 2 + d_y / 2, -box->Z_length() / 2 + d_z / 2);
        cur_normal = Point3d(1, 0, 0);
        drag_torque+=compute_liquid_drag_on_plane(joint, box->X_length(), box->Y_length(), box->Z_length(),
            cur_pos, cur_normal, water_level, 0, nbr_interval_y, nbr_interval_z);

        //now the right face
        cur_pos = center + Point3d(-box->X_length() / 2, -box->Y_length() / 2 + d_y / 2, -box->Z_length() / 2 + d_z / 2);
        cur_normal = Point3d(-1, 0, 0);
        drag_torque+=compute_liquid_drag_on_plane(joint, box->X_length(), box->Y_length(), box->Z_length(),
            cur_pos, cur_normal, water_level, 0, nbr_interval_y, nbr_interval_z);

        //now the top face
        cur_pos = center + Point3d(-box->X_length() / 2 + d_x / 2, box->Y_length() / 2, -box->Z_length() / 2 + d_z / 2);
        cur_normal = Point3d(0, 1, 0);
        drag_torque+=compute_liquid_drag_on_plane(joint, box->X_length(), box->Y_length(), box->Z_length(),
            cur_pos, cur_normal, water_level, nbr_interval_x, 0, nbr_interval_z);

        //now the bottom face to finish
        cur_pos = center + Point3d(-box->X_length() / 2 + d_x / 2, -box->Y_length() / 2, -box->Z_length() / 2 + d_z / 2);
        cur_normal = Point3d(0, -1, 0);
        drag_torque+=compute_liquid_drag_on_plane(joint, box->X_length(), box->Y_length(), box->Z_length(),
            cur_pos, cur_normal, water_level, nbr_interval_x, 0, nbr_interval_z);


        //*/
        //*
        //I need this to get hthe world coordinates

        Vector3d nx = body->getWorldCoordinates(Vector3d(1, 0, 0));
        Vector3d ny = body->getWorldCoordinates(Vector3d(0, 1, 0));
        Vector3d nz = body->getWorldCoordinates(Vector3d(0, 0, 1));
        Vector3d wvx = nx*d_x;
        Vector3d wvy = ny*d_y;
        Vector3d wvz = nz*d_z;


        //just a note since the larger faces are the top and bottom face (and that it's near impossible that none of them
        //is facing the movement) I'll base miself on tham to compute the friction

        //first let's handle the back face
        cur_pos = body->getWorldCoordinates(center + Point3d(-box->X_length() / 2 + d_x / 2,
                                                             -box->Y_length() / 2 + d_y / 2, -box->Z_length() / 2));
        cur_normal = -nz;
        drag_impact += compute_liquid_drag_on_planev2(body, cur_pos, cur_normal, water_level,
                                                      wvx, wvy, nbr_interval_x, nbr_interval_y, eff_density);

        //now the front face
        cur_pos = body->getWorldCoordinates(center + Point3d(-box->X_length() / 2 + d_x / 2,
                                                             -box->Y_length() / 2 + d_y / 2, box->Z_length() / 2));
        cur_normal = nz;
        drag_impact += compute_liquid_drag_on_planev2(body, cur_pos, cur_normal, water_level
                                                      , wvx, wvy, nbr_interval_x, nbr_interval_y, eff_density);

        //now the left face
        cur_pos = body->getWorldCoordinates(center + Point3d(box->X_length() / 2, -box->Y_length() / 2 + d_y / 2,
                                                             -box->Z_length() / 2 + d_z / 2));
        cur_normal = nx;
        drag_impact += compute_liquid_drag_on_planev2(body, cur_pos, cur_normal, water_level,
                                                      wvy, wvz, nbr_interval_y, nbr_interval_z, eff_density);

        //now the right face
        cur_pos = body->getWorldCoordinates(center + Point3d(-box->X_length() / 2, -box->Y_length() / 2 + d_y / 2
                                                             , -box->Z_length() / 2 + d_z / 2));
        cur_normal = -nx;
        drag_impact += compute_liquid_drag_on_planev2(body, cur_pos, cur_normal, water_level,
                                                      wvy, wvz, nbr_interval_y, nbr_interval_z, eff_density);

        //now the top face
        cur_pos = body->getWorldCoordinates(center + Point3d(-box->X_length() / 2 + d_x / 2, box->Y_length() / 2,
                                                             -box->Z_length() / 2 + d_z / 2));
        cur_normal = ny;
        drag_impact += compute_liquid_drag_on_planev2(body, cur_pos, cur_normal, water_level,
                                                      wvx, wvz, nbr_interval_x, nbr_interval_z, eff_density);

        //now the bottom face to finish
        cur_pos = body->getWorldCoordinates(center + Point3d(-box->X_length() / 2 + d_x / 2, -box->Y_length() / 2,
                                                             -box->Z_length() / 2 + d_z / 2));
        cur_normal = -ny;
        drag_impact += compute_liquid_drag_on_planev2(body, cur_pos, cur_normal, water_level,
                                                      wvx, wvz, nbr_interval_x, nbr_interval_z, eff_density);

        //*/
    }

    return drag_impact;
}

/**
    Compute and affect to force on a face
*/
ForceStruct ODEWorld::compute_liquid_drag_on_plane(RigidBody *body, double l_x, double l_y, double l_z, Point3d pos,
                                                Vector3d normal, float water_level,	int nbr_interval_x, int nbr_interval_y, int nbr_interval_z){




    ForceStruct drag_impact(body->getCMPosition());

    double d_x = 0, d_y = 0, d_z = 0;
    if (nbr_interval_x > 0){
        d_x = l_x / nbr_interval_x;
    }

    if (nbr_interval_y > 0){
        d_y = l_y / nbr_interval_y;
    }

    if (nbr_interval_z > 0){
        d_z = l_z / nbr_interval_z;
    }

    if ((nbr_interval_x > 0) && (nbr_interval_y > 0)){
        double d_S = d_x*d_y;
        double init_y = pos.y;


        for (int i = 0; i < nbr_interval_x; ++i){
            for (int j = 0; j < nbr_interval_y; ++j){
                //here we calculate the force and applys it
                Vector3d V = body->getLocalCoordinates(body->getAbsoluteVelocityForLocalPoint(pos));
                Vector3d V_norm = V / V.length();

                //we the center of the fragment is not underater we skipp it
                if (body->getWorldCoordinates(pos).y > water_level){
                    continue;
                }

                //we check if we are a facing the movement
                double V_eff = V.z*normal.z;
                if (V_eff > 0){
                    double S = d_S;

                    //now that we have the surface we can compute the resulting force
                    Vector3d F = -normal*V_eff*V_eff * 1 / 2 * SimGlobals::liquid_density*S;
                    F = body->getWorldCoordinates(F);
                    if (F.length() > 0.1){
                        F /= 2;
                    }

                    ForceStruct elem_impact;
                    elem_impact.F=F;
                    elem_impact.pt=body->getWorldCoordinates(pos);

                    //now tranfer it to the body COM
                    elem_impact.modifyApplicationPoint(body->getCMPosition());

                    //and add it
                    drag_impact+=elem_impact;

                    /*
                    //this can be used to show the forces
                    ForceStruct cp;
                    cp.F = F;
                    cp.pt = body->getWorldCoordinates(pos);
                    SimGlobals::vect_forces.push_back(cp);
                    //*/

                }
                pos = pos + Point3d(0, d_y, 0);
            }
            pos = Point3d(pos.x + d_x, init_y, pos.z);
        }
    }

    if ((nbr_interval_x > 0) && (nbr_interval_z > 0)){
        double d_S = d_x*d_z;
        double init_z = pos.z;

        for (int i = 0; i < nbr_interval_x; ++i){
            for (int j = 0; j < nbr_interval_z; ++j){
                //here we calculate the force and applys it
                Vector3d V = body->getLocalCoordinates(body->getAbsoluteVelocityForLocalPoint(pos));
                Vector3d V_norm = V / V.length();

                //we the center of the fragment is not underater we skipp it
                if (body->getWorldCoordinates(pos).y > water_level){
                    continue;
                }

                //we check if we are afacing the movement
                double V_eff = V.y*normal.y;
                if (V_eff > 0){
                    double S = d_S;

                    //now that we have the surface we can compute the resulting force
                    Vector3d F = -normal*V_eff*V_eff * 1 / 2 * SimGlobals::liquid_density*S;

                    //if we remove th e approximation of constant speed on the whole toes we need to stop doing the integral
                    //and apply the force on every ds
                    F = body->getWorldCoordinates(F);
                    if (F.length() > 0.1){
                        F /= 2;
                    }


                    ForceStruct elem_impact;
                    elem_impact.F=F;
                    elem_impact.pt=body->getWorldCoordinates(pos);

                    //now tranfer it to the body COM
                    elem_impact.modifyApplicationPoint(body->getCMPosition());

                    //and add it
                    drag_impact+=elem_impact;

                    /*
                    //this can be used to show the forces
                    ForceStruct cp;
                    cp.F = F;
                    cp.pt = body->getWorldCoordinates(pos);
                    SimGlobals::vect_forces.push_back(cp);
                    //*/

                }
                pos = pos + Point3d(0, 0, d_z);
            }
            pos = Point3d(pos.x + d_x, pos.y, init_z);
        }
    }

    if ((nbr_interval_y > 0) && (nbr_interval_z > 0)){
        double d_S = d_z*d_y;
        double init_y = pos.y;

        for (int i = 0; i < nbr_interval_z; ++i){
            for (int j = 0; j < nbr_interval_y; ++j){
                //here we calculate the force and applys it
                Vector3d V = body->getLocalCoordinates(body->getAbsoluteVelocityForLocalPoint(pos));
                Vector3d V_norm = V / V.length();

                //we the center of the fragment is not underater we skipp it
                if (body->getWorldCoordinates(pos).y > water_level){
                    continue;
                }

                //we check if we are afacing the movement
                double V_eff = V.x*normal.x;
                if (V_eff > 0){
                    double S = d_S;

                    //now that we have the surface we can compute the resulting force
                    Vector3d F = -normal*V_eff*V_eff * 1 / 2 * SimGlobals::liquid_density*S;
                    F = body->getWorldCoordinates(F);

                    //if we remove th e approximation of constant speed on the whole toes we need to stop doing the integral
                    //and apply the force on every ds
                    if (F.length() > 0.1){
                        F /= 2;
                    }


                    ForceStruct elem_impact;
                    elem_impact.F=F;
                    elem_impact.pt=body->getWorldCoordinates(pos);

                    //now tranfer it to the body COM
                    elem_impact.modifyApplicationPoint(body->getCMPosition());

                    //and add it
                    drag_impact+=elem_impact;
                    /*
                    //this can be used to show the forces
                    ForceStruct cp;
                    cp.F = F;
                    cp.pt = body->getWorldCoordinates(pos);
                    SimGlobals::vect_forces.push_back(cp);
                    //*/
                }
                pos = pos + Point3d(0, d_y, 0);
            }
            pos = Point3d(pos.x, init_y, pos.z + d_z);
        }
    }

    return drag_impact;
}



ForceStruct ODEWorld::compute_liquid_drag_on_planev2(RigidBody *body, Point3d pos, Vector3d normal, float water_level,
                                                  Vector3d v1, Vector3d v2, int nbr_interval_v1, int nbr_interval_v2, double density, double friction_coef, double l3){


    ForceStruct drag_impact(body->getCMPosition());


    double d_S = v1.length()*v2.length();
    double S_face = 2 * d_S;//front and back
    double S_sides = v1.length()*l3;


    for (int i = 0; i < nbr_interval_v1; ++i){
        Point3d line_start = pos;
        for (int j = 0; j < nbr_interval_v2; ++j){
            //we the center of the fragment is not underater we skipp it
            if (pos.y > water_level){
                continue;
            }

            //here we calculate the force and applys it
            Vector3d V = body->getAbsoluteVelocityForGlobalPoint(pos);


            //we check if we are a facing the movement
            double V_eff = V.dotProductWith(normal);
            if (V_eff > 0){


                //now that we have the surface we can compute the resulting force
                Vector3d F = -normal*V_eff*V_eff * 0.5 * (density*d_S);

                if (l3 > 0){
                    double S_contact = S_face;
                    //now I'll take into account the sides
                    if (j == 0 || j == (nbr_interval_v2 - 1)){
                        S_contact += S_sides;
                    }
                    F += V*V.length()*0.5*density*S_contact*(1.328 / std::sqrt(friction_coef*V.length()));
                }


                ForceStruct elem_impact;
                elem_impact.F=F;
                elem_impact.pt=pos;

                //now tranfer it to the body COM
                elem_impact.modifyApplicationPoint(body->getCMPosition());

                //and add it
                drag_impact+=elem_impact;

                /*
                //this can be used to show the forces
                ForceStruct cp;
                cp.F = F;
                cp.pt = pos;
                SimGlobals::vect_forces.push_back(cp);
                //*/

            }





            pos = pos + v2;
        }
        pos = line_start + v1;
    }
    return drag_impact;
}



/**
this function is a children function of the above one (it prevent mass duplication of code for similar body parts)
this function handle the legs and arms
*/
ForceStruct ODEWorld::compute_liquid_drag_on_legs(RigidBody *body, float water_level, double eff_density, double friction_coef){

    CollisionDetectionPrimitive* cdp = body->cdps.front();
    CapsuleCDP* capsule = dynamic_cast<CapsuleCDP*>(cdp);

    if (capsule == NULL){
        return ForceStruct();
    }

    //I want the lower points to find out if the capsule is in the water
    //it's easy it's forced that the lowest point is the extremity of the cylinder minus the radius
    //I call Z the vertical axis but in this world representation the vertical axis is actualy Y...
    Point3d wA = body->getWorldCoordinates(capsule->getA());
    Point3d wB = body->getWorldCoordinates(capsule->getB());
    double miny = std::fmin(wA.y, wB.y);
    miny -= capsule->radius();


    ForceStruct drag_impact(body->getCMPosition());


    //we vrify that the water hit the capsule before doing anything
    if (miny<water_level){
        //now I'll subdivide the faces in smaller surfaces and apply the necessary force on each of them
        //so I'll first concider the cylindric part of the capsule.

        //let's say we will consider 20 intervals on the axis of the cylinder
        //and we will consider 3 intervals on the facet for each interval on the axis
        int axis_intervals = 20;
        int facet_intervals = 3;

        //I precalculate some information on the axis and the facet so the algorithm will be faster
        double facet_interval_length = capsule->radius() * 2 / facet_intervals;
        Vector3d axis_unit_vector = capsule->getB()-capsule->getA();
        if (wA.y> wB.y){
            axis_unit_vector *= -1;
        }

        double axis_length = axis_unit_vector.length();
        axis_unit_vector /= axis_length;
        double axis_interval_length = axis_length / axis_intervals;
        Vector3d axis_interval_vect = axis_unit_vector*axis_interval_length;
        double facet_interval_area=axis_interval_length*facet_interval_length;

        //position at the start
        Point3d axis_cur_pos;
        if (wA.y> wB.y){
            axis_cur_pos = capsule->getB() + axis_interval_vect / 2;
        }
        else{
            axis_cur_pos = capsule->getA() + axis_interval_vect / 2;
        }

        //so now we start to iterate along the axis
        for (int i = 0; i < axis_intervals; ++i){
            //here is an approximation (comming from the fact that the facet are likely to be horizontal (meaning the height))
            //of each subfacet is likely to be the same as the on of the central point
            //if (body->getWorldCoordinates(axis_cur_pos).y > water_level){
            //	continue;
            //}

            //we read the spead on the axis
            Vector3d axis_speed = body->getLocalCoordinates(body->getAbsoluteVelocityForLocalPoint(axis_cur_pos));

            //to create the facet I must finds the vector that have the same direction as the speed but face the axis
            Vector3d n = axis_speed - axis_unit_vector*(axis_unit_vector.dotProductWith(axis_speed));
            n /= n.length();

            //I just compute the last vector of the basis
            Vector3d v2 = n.crossProductWith(axis_unit_vector);
            v2 /= v2.length();

            //now I have to iterate on each interval of the facet
            //so i position myself at the first interval
            Point3d cur_pos = axis_cur_pos + v2*(capsule->radius() - facet_interval_length / 2);

            bool force_applied = false;
            for (int j = 0; j < facet_intervals; ++j){
                if (body->getWorldCoordinates(cur_pos).y < water_level){
                    Vector3d local_speed = body->getLocalCoordinates(body->getAbsoluteVelocityForLocalPoint(cur_pos));

                    //now i want to ponderate the area by the orientation along the axis
                    //but what I realy want is the sinus (since the ponderation is of 1 if the two vectors are perpendicular)
                    double S = facet_interval_area*
                            ((local_speed / local_speed.length()).crossProductWith(axis_unit_vector)).length();

                    //now I can compute the force and apply it
                    Vector3d F = -local_speed*local_speed.length() * 1 / 2 * eff_density*S;
                    F = body->getWorldCoordinates(F);

                    ForceStruct elem_impact;
                    elem_impact.F=F;
                    elem_impact.pt=body->getWorldCoordinates(cur_pos);

                    //now tranfer it to the body COM
                    elem_impact.modifyApplicationPoint(body->getCMPosition());

                    //and add it
                    drag_impact+=elem_impact;

                    /*
                    //this can be used to show the forces
                    ForceStruct cp;
                    cp.F = F;
                    cp.pt = body->getWorldCoordinates(cur_pos);
                    SimGlobals::vect_forces.push_back(cp);
                    //*/

                    force_applied = true;
                }
                cur_pos = cur_pos - v2*facet_interval_length;
            }

            //we stop if we realise that no forces were applied on a whole row
            if (!force_applied){
                break;
            }

            axis_cur_pos += axis_interval_vect;
        }
    }
    return drag_impact;
}


ForceStruct ODEWorld::compute_buoyancy(RigidBody *body, float water_level){

    CollisionDetectionPrimitive* cdp = body->cdps.front();
    SphereCDP* sphere = dynamic_cast<SphereCDP*>(cdp);
    BoxCDP* box = dynamic_cast<BoxCDP*>(cdp);
    CapsuleCDP* capsule = dynamic_cast<CapsuleCDP*>(cdp);

    ForceStruct result_force;
    if (sphere != NULL){
        result_force = compute_buoyancy_on_sphere(body, water_level, -SimGlobals::gravity, SimGlobals::liquid_density);// +SimGlobals::force_alpha);
    }
    else if (box != NULL){
        result_force = compute_buoyancy_on_box(body, water_level, -SimGlobals::gravity, SimGlobals::liquid_density);// +SimGlobals::force_alpha);
    }
    else if (capsule != NULL){
        result_force = compute_buoyancy_on_capsule(body, water_level, -SimGlobals::gravity, SimGlobals::liquid_density);// +SimGlobals::force_alpha);
    }

    return result_force;
}

ForceStruct ODEWorld::compute_buoyancy_on_sphere(RigidBody* body, float water_level, double gravity, double density){

    CollisionDetectionPrimitive* cdp = body->cdps.front();
    SphereCDP* sphere = dynamic_cast<SphereCDP*>(cdp);

    if (sphere == NULL){
        throwError("the toes should only have a sphere primitive...");
        return ForceStruct();
    }

    double r = sphere->getRadius();
    double h = water_level - (body->getWorldCoordinates(sphere->getCenter()).getY() + r);

    ForceStruct result;

    //we vrify that the water hit the ball before doing anything
    if (h>0){
        //I first eliminate the case where the spere is fully in the water (for efficiency purposes)
        if (h >= (2 * r)){
            double V = 4.0/3.0 * PI* r*r*r;
            Vector3d F = Vector3d(0, V*density*gravity, 0);
            result.F = F;
            result.pt = body->getWorldCoordinates(sphere->getCenter());

            return result;
        }

        //formula for volume here https://www.lmnoeng.com/Volume/CylConeSphere.php
        double V = PI / 3.0 * h*h* (3.0 * r - h);
        Vector3d F = Vector3d(0, V*density*gravity, 0);

        //pr une demontration of that formula look for spherical cap
        //https://math.stackexchange.com/questions/845959/centre-of-mass-and-moment-of-inertia-of-a-sphere-spherical-cap
        double dy = 3.0/4.0 * (2.0 * r - h)*(2.0 * r - h) / (3.0 * r - h);
        Point3d pt = body->getWorldCoordinates(sphere->getCenter()) - Point3d(0, dy, 0);


        result.F = F;
        result.pt = pt;

        return result;

    }
    return ForceStruct();
}

ForceStruct ODEWorld::compute_buoyancy_on_box(RigidBody* body, float water_level, double gravity, double density){

    CollisionDetectionPrimitive* cdp = body->cdps.front();
    BoxCDP* box = dynamic_cast<BoxCDP*>(cdp);

    if (box == NULL){
        //throwError("the toes should only have a sphere primitive...");
        return ForceStruct();
    }

    //I want the lower points to find out if the box is in the water
    double lx = box->X_length(), ly = box->Y_length(), lz = box->Z_length();
    Point3d center = box->center();
    Point3d corners[8], wcorners[8];

    corners[0] = center + Point3d(lx / 2, ly / 2, -lz / 2);
    corners[1] = center + Point3d(lx / 2, -ly / 2, -lz / 2);
    corners[2] = center + Point3d(-lx / 2, ly / 2, -lz / 2);
    corners[3] = center + Point3d(-lx / 2, -ly / 2, -lz / 2);
    corners[4] = center + Point3d(lx / 2, ly / 2, lz / 2);
    corners[5] = center + Point3d(lx / 2, -ly / 2, lz / 2);
    corners[6] = center + Point3d(-lx / 2, ly / 2, lz / 2);
    corners[7] = center + Point3d(-lx / 2, -ly / 2, lz / 2);

    double miny = body->getWorldCoordinates(corners[0]).getY();
    int idx_min = 0;

    double maxy= body->getWorldCoordinates(corners[0]).getY();

    for (int i = 0; i < 8; ++i){
        wcorners[i] = body->getWorldCoordinates(corners[i]);
        if (wcorners[i].getY() < miny){
            miny = wcorners[i].getY();
            idx_min = i;
        }
        if (wcorners[i].getY() > maxy){
            maxy = wcorners[i].getY();
        }
    }



    //we vrify that the water hit the box before doing anything
    if (miny<water_level){
        // I want to check some easy to compute cases.
        // First if the object is fully immersed (easy to find the highest point if we know the positions
        //of the lower int the box)
        //*
        if (maxy < water_level){
            double V = lx*ly*lz;
            Vector3d F = Vector3d(0, V*density*gravity, 0);

            ForceStruct result_force;
            result_force.F = F;
            result_force.pt = body->getWorldCoordinates(center);

            return result_force;
        }//*/


        // for all that is after it ill be easier if we know the basis formed by a corner
        // please note that the condition are set like that because of the way I built the corner structure
        Vector3d vx(1,0,0), vy(0,1,0), vz(0,0,1);
        if(idx_min > 3){
            vz *= -1;
        }
        if ((idx_min & 1) == 0){
            vy *= -1;
        }
        if (idx_min % 4 < 2){
            vx *= -1;
        }



        //I'll initialize some variable since i'll need them for every following tests
        int nbr_interval_x = 3;
        int nbr_interval_y = 3;
        int nbr_interval_z = 7;


        //second test is to check if e could not simplify the plroblem by a prism
        Vector3d wvx = body->getWorldCoordinates(vx);
        Vector3d wvy = body->getWorldCoordinates(vy);
        Vector3d wvz = body->getWorldCoordinates(vz);

        int count_alligned = 0;
        double epsilon = 1.0/(float)nbr_interval_y;

        if (wvx.y < epsilon){
            ++count_alligned;
        }
        if (wvy.y < epsilon){
            ++count_alligned;
        }
        if (wvz.y < epsilon){
            ++count_alligned;
        }

        if (count_alligned>0){
            //so we have in a configuration were I simplify by a prism calculation
            //I'm not sure of those computations so i'll forget that case ...

            /*if (count_alligned>1){
                //so we have a vertical prism calculationare simple so I won't comment it
                //the calculations
                double V, h;
                Point3d pt;
                h = (water_level - wcorners[idx_min].y);
                if (wvx.y < epsilon){
                    if (wvy.y < epsilon){
                        V = lx*ly;
                        pt = corners[idx_min] + vx*(lx / 2) + vy*(ly / 2) + vz*(h / 2);
                    }
                    else{
                        V = lx*lz;
                        pt = corners[idx_min] + vx*(lx / 2) + vy*(h / 2) + vz*(lz / 2);
                    }
                }
                else{
                    V = ly*lz;
                    pt = corners[idx_min] + vx*(h / 2) + vy*(ly / 2) + vz*(lz / 2);
                }
                V *= h;
                Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;
                return;
            }

            //so we have an inclined prism
            //here it's more tricky be cause I need the immerged surface on the side face.
            //so first I'll find the idxs of the corners of the face and i'll order them from
            //the lowest to the highest
            //seeing the code of that optimisation (which is not finished btw) I think it may take more time than simply
            //doing an integral...
            //so i'll disactivate it for now
            /*

            double V,h;
            int face[4];
            face[0] = idx_min;
            if (wvx.y < epsilon){
                int fz = 4 * vz.z;
                int fy = 1 * vy.y;

                if (wvy.y < wvz.y){
                    face[1] = idx_min + fy;
                    face[2] = idx_min + fz;
                }
                else {
                    face[1] = idx_min + fz;
                    face[2] = idx_min + fy;
                }
                face[3] = idx_min+fz+fy;
                h= lx;
            }
            if (wvy.y < epsilon){
                int fz = 4 * vx.x;
                int fx = 2 * vy.y;

                if (wvx.y < wvz.y){
                    face[1] = idx_min + fx;
                    face[2] = idx_min + fz;
                }
                else {
                    face[1] = idx_min + fz;
                    face[2] = idx_min + fx;
                }
                face[3] = idx_min + fz + fx;
                h= ly;
            }
            else{
                int fx = 2 * vx.x;
                int fy = 1 * vy.y;

                if (wvx.y < wvy.y){
                    face[1] = idx_min + fx;
                    face[2] = idx_min + fy;
                }
                else {
                    face[1] = idx_min + fy;
                    face[2] = idx_min + fx;
                }
                face[3] = idx_min + fy + fx;

                h= lz;
            }
            //no that we have the ordored face corner we can calculate the surface
            //there are 3 cases :

            //the first one is when the water level is above the 3rd point. In that cas e we have a partial
            //triangle (it's a trapezoid) and a second trapezoid under it



            if (water_level>wcorners[face[2]].y){
                //first I need the last point for the base of the upper triangle
                double ht = wcorners[face[3]].y - wcorners[face[2]].y;
                double dht = (wcorners[face[3]].y - water_level) / ht;
                Point3d triangle_pt = wcorners[face[3]] + (wcorners[face[2]] - wcorners[face[3]])*dht;

                double A_triangle = ht* (triangle_pt - wcorners[face[2]]).length();
                A_triangle -= A_triangle*dht;

                //and now the area of the trapezoid
            }
            else if (water_level > wcorners[face[2]].y){

            }
            else{

            }

            Vector3d F = Vector3d(0, 1, 0)*V*SimGlobals::liquid_density*SimGlobals::gravity;
            return;
            //*/
        }

        //I we reach here it mean we are not a in a case where I can do a trivial calculation
        //now I'll subdivide the box in smaller boxes and simply count the ones underwater
        double d_x = lx / nbr_interval_x;
        double d_y = ly / nbr_interval_y;
        double d_z = lz / nbr_interval_z;
        double dV = d_x*d_y*d_z;

        //I'll have to work in word coordinate else I'd have to convert everytime I check with the water level
        Vector3d vxp = body->getWorldCoordinates(vx*d_x);
        Vector3d vyp = body->getWorldCoordinates(vy*d_y);
        Vector3d vzp = body->getWorldCoordinates(vz*d_z);


        Vector3d result = Vector3d(0, 0, 0);
        int count=0;
        int i, j, k;
        Vector3d cur_pos=wcorners[idx_min]+vxp/2+vyp/2+vzp/2;
        Vector3d cur_plane,cur_box;
        double V = 0;
        for (i = 0; i < nbr_interval_x; ++i){
            cur_plane = cur_pos;
            for (j = 0; j < nbr_interval_y; ++j){
                cur_box = cur_plane;
                for (k = 0; k < nbr_interval_z; ++k){
                    if (cur_box.y>water_level){
                        break;
                    }
                    //here it mean the box if underwater
                    result += cur_box;
                    V+=dV;
                    count++;

                    cur_box += vzp;
                }
                if (k == 0){
                    break;
                }
                cur_plane += vyp;
            }
            if (j == 0){
                break;
            }
            cur_pos += vxp;
        }

        //and now we apply the force if it is possible
        if (count != 0){
            Vector3d F = Vector3d(0, V*density*gravity, 0);
            Point3d pt = result / count;

            ForceStruct result_force;
            result_force.F = F;
            result_force.pt = pt;

            return result_force;
        }
    }
    return ForceStruct();
}

ForceStruct ODEWorld::compute_buoyancy_on_capsule(RigidBody* body, float water_level, double gravity, double density){
    CollisionDetectionPrimitive* cdp = body->cdps.front();
    CapsuleCDP* capsule = dynamic_cast<CapsuleCDP*>(cdp);

    if (capsule == NULL){
        //throwError("the toes should only have a sphere primitive...");
        return ForceStruct();
    }

    //I want the lower points to find out if the capsule is in the water
    //it's easy it's forced that the lowest point is the extremity of the cylinder minus the radius
    //I call Z the vertical axis but in this world representation the vertical axis is actualy Y...
    Point3d wA = body->getWorldCoordinates(capsule->getA());
    Point3d wB = body->getWorldCoordinates(capsule->getB());
    double miny = std::fmin(wA.y, wB.y);
    miny -= capsule->radius();


    //we vrify that the water hit the capsule before doing anything
    if (miny < water_level){
        ForceStruct result_force;

        double r = capsule->radius();

        //I need the normal of the vertica plane to create the rotation to get the lowest point
        //also used to know which shepre is the bottom one
        Point3d axis_lowest_pt = wB;
        Point3d axis_upper_pt = wA;
        if (axis_upper_pt.y < axis_lowest_pt.y){
            axis_lowest_pt = wA;
            axis_upper_pt = wB;
        }

        //compute the axis
        Vector3d axis_vector = axis_upper_pt - axis_lowest_pt;
        double axis_len = axis_vector.length();
        axis_vector /= axis_len;

        //there are 3 parts: lower sphere, upper sphere and cylinder
        //FOr each part I'll tryto handle easy cases then use a general method in case none of the optimisation are possible
        //but I have no idea for the formulae of a cut half sphere.
        //so the best solution is to extend the cynlindrical part a bit to simulate the impact of the two half spheres
        axis_upper_pt=axis_upper_pt+axis_vector*r*3.0/4.0;
        axis_lowest_pt=axis_lowest_pt-axis_vector*r*3.0/4.0;

        //update the axis for the approximation
        axis_vector = axis_upper_pt - axis_lowest_pt;
        axis_len = axis_vector.length();
        axis_vector /= axis_len;

        //and finaly I handle the cylindric part
        //that for is just an easy way to skip the complex calculation in the case the simplification works
        for (int uselessvar = 0; uselessvar < 1; ++uselessvar){

            Vector3d n = Vector3d(0, 1, 0).crossProductWith(axis_vector);
            double sin_angle = n.length();
            n /= sin_angle;

            //and now I calculate the lowest point
            Quaternion quat = Quaternion::getRotationQuaternion(PI / 2, n);
            Vector3d vh = quat.rotate(axis_vector)*r;
            if (vh.y > 0){
                vh *= -1;
                sin_angle *= -1;
            }
            Point3d lowest_pt = axis_lowest_pt + vh;


            if (lowest_pt.y < water_level){
                //now I check if the full cylinder is
                Point3d highest_pt = axis_upper_pt - vh;
                if (highest_pt.y < water_level){
                    double V = PI*r*r*axis_len;
                    Vector3d F = Vector3d(0, V*density*gravity, 0);
                    Point3d inter = Vector3d(axis_upper_pt + axis_lowest_pt) / 2;
                    Point3d pt = inter;


                    result_force.F = F;
                    result_force.pt = pt;

                    break;
                }

                //another easy case is when the cilinder is near vertical
                //I will supose that as long as the angle is less than 5� it is vertical
                //trully I conpare with 5.73 so that the sin is near 0.1
                if (std::abs(sin_angle) < 0.01){
                    double h = water_level - lowest_pt.y + vh.y / 2;
                    if (h > axis_len){
                        h = axis_len;
                    }
                    if (h > 0){
                        double V = PI*r*r*h;
                        Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;
                        Point3d inter = axis_lowest_pt + axis_vector*h/ 2 +vh*sin_angle;
                        Point3d pt = inter;

                        result_force.F = F;
                        result_force.pt = pt;


                    }
                    break;
                }

                //on the same logic I'll handle the cases where the cylinder is near horizontal
                //my goal is to eliminate the case of the near horizontal cylinder. But it is revelant to do it
                //only if the 2 bases are affected by water
                Point3d low_high_pt = lowest_pt + axis_vector*axis_len;
                if (((1 - std::abs(sin_angle)) < 0.001) && (low_high_pt.y < water_level)){
                    double H = water_level - lowest_pt.y;
                    if (H > r * 2){
                        H = r * 2;
                    }
                    if (H > 0){
                        //now I want to conform the notation used by wolfram
                        //using their formula I can only achieve a result if the immersed part is lower than
                        //half of the circle
                        double h=H;
                        if (H > r){
                            h = 2 * r - H;
                        }
                        double theta = safeACOS((r - h) / r);
                        double circle_segment_area = r*r*(theta - std::sin(theta))/2;
                        double area = circle_segment_area;
                        if (axis_lowest_pt.y<water_level){
                            area = PI*r*r - area;
                        }
                        Vector3d v_inter(vh.x, 0, vh.z);
                        double cylinder_length = axis_len - v_inter.length();
                        double V = cylinder_length*area;
                        if (V > 0.00001){
                            Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;

                            double sin_inter = std::sin(theta / 2);
                            double dy=0;
                            if ((theta - std::sin(theta)) > 0.0000001){
                                dy= 4 * r*sin_inter*sin_inter*sin_inter / (3 * (theta - std::sin(theta)));
                                if (H>r){
                                    dy = -dy*circle_segment_area/area;
                                }
                            }

                            Point3d inter = Vector3d(axis_lowest_pt + Point3d(0, dy, 0) + axis_vector*(cylinder_length-v_inter.length()) / 2);
                            Point3d pt = inter;

                            result_force.F = F;
                            result_force.pt = pt;

                        }
                    }
                    break;
                }

                //now I have 2 case left. depending on the water level we have either a cylindrical wegde or a cylindrical segment
                //so I need to check in which case we are
                Point3d high_low_pt = axis_lowest_pt - vh;

                if (high_low_pt.y < water_level){
                    //this mean we have a cylindrical segment
                    double h1 = (axis_vector*((water_level - high_low_pt.y) / axis_vector.y)).length();
                    double h2 = (axis_vector*((water_level - lowest_pt.y) / axis_vector.y)).length();//possible to change this to rmv the /

                    double V = PI*r*r*(h1 + h2) / 2;
                    Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;

                    //to compute the center of mass I use a formula from wolfram cilynder segment page
                    //in their coordinate the cylinder axis is following Z and the  base f the cylinder is in the plane (xOy)
                    double dx = r*(h2 - h1) / 4 * (h1 + h2);
                    double dz = (5*h1*h1+6*h1*h2+5*h2*h2)/(16*(h1+h2));

                    Point3d inter = Vector3d(axis_lowest_pt + axis_vector*dz + vh / vh.length()*dx);
                    Point3d pt = inter;

                    result_force.F = F;
                    result_force.pt = pt;

                }
                else{
                    //this mean we have a cylindrical wedge
                    //formula found on the wolfram page for cylindrical wedge
                    double h = (axis_vector*((water_level - lowest_pt.y) / axis_vector.y)).length();
                    double b = (-vh *((water_level - lowest_pt.y) / (-vh.y))).length();
                    double a = std::sqrt(b*(2 * r - b));

                    double V = (h / (6 * b))*(2 * a*(3 * r*r - 2 * a*b + b*b) - 3 * PI*r*r*(r - b) +
                                              6 * r*r*(r - b)*safeASIN((r - b) / r));
                    Vector3d F = Vector3d(0, 1, 0)*V*density*gravity;

                    //since the calculation of the real application point seems to be pretty hard
                    //I'll use a simplification conidering that the point is on the triangle centroid
                    Point3d inter = Vector3d(lowest_pt - vh / vh.length()*b + lowest_pt + lowest_pt + axis_vector*h) / 3;
                    Point3d pt = inter;

                    result_force.F = F;
                    result_force.pt = pt;

                }

                try{
                    result_force.check_for_undefined();
                }catch(const char* c){
                    std::string msg(c);
                    std::ostringstream oss;
                    oss<<msg<<"  //  "<<" boyancy capsule complex middle for body : "<<body->name();
                    throw(oss.str().c_str());
                }


                //now I have to compute a negative weight to handle the case where the cylinder is horizontal enougth
                //to let the water go above the upper face. So that negative weight will correspond to the cylindrical wedge
                //above that face.
                if (low_high_pt.y < water_level){
                    double h = (axis_vector*((water_level - low_high_pt.y) / axis_vector.y)).length();
                    double b = (-vh *((water_level - low_high_pt.y) / (-vh.y))).length();
                    double a = std::sqrt(b*(2 * r - b));

                    double V = (h / (6 * b))*(2 * a*(3 * r*r - 2 * a*b + b*b) - 3 * PI*r*r*(r - b) +
                                              6 * r*r*(r - b)*safeASIN((r - b) / r));
                    Vector3d F = Vector3d(0, -1, 0)*V*density*gravity;

                    //since the calculation of the real application point seems to be pretty hard
                    //I'll use a simplification conidering that the point is on the triangle centroid
                    Point3d pt = Vector3d(low_high_pt - vh / vh.length()*b + low_high_pt + low_high_pt + axis_vector*h) / 3;

                    //add the forces to and use the barycenter for new application point (its correct since there is only
                    //one component to the force
                    result_force.pt = result_force.pt*result_force.F.length() + pt*F.length();
                    result_force.pt = result_force.pt/(result_force.F.length()+F.length());
                    result_force.F += F;

                    try{
                        result_force.check_for_undefined();
                        pt.check_for_undefined();
                    }catch(const char* c){
                        std::string msg(c);
                        std::ostringstream oss;
                        oss<<msg<<"  //  "<<" boyancy capsule complex end for body : "<<body->name()<<
                             " vh.y: "<<vh.y<<"   b: "<<b<<"   V: "<<V<<"   a: "<<a;
                        throw(oss.str().c_str());
                    }
                }

            }
        }



        return result_force;
    }
    return ForceStruct();
}


void ODEWorld::initParticleFluid(){
    //create the fluid and load everything
    Interface::initFluid(SimGlobals::dt);


    //now the problem is that I need ot add the objects that interact witht he fluid in the physics simulation
    //add all the objects
    /*
    //std::string vect_obj_name[]={"pelvis","torso","head","lUpperarm","lLowerarm","rUpperarm","rLowerarm",
    //                             "lUpperleg","lLowerleg","rUpperleg","rLowerleg","lFoot","rFoot"};

    std::string vect_obj_name[]={"lUpperleg","lLowerleg","rUpperleg","rLowerleg","lFoot","rFoot"};
    int numObj=6;

    for(int i=0;i<numObj;++i){
        if(getRBByName(vect_obj_name[i])!=NULL){
            vect_objects_fluid_interaction.push_back(getRBByName(vect_obj_name[i]));
        }else{
            std::cout<<"object "<<vect_obj_name[i]<<" not found"<<std::endl;
        }
    }
    sendDataToParticleFluidEngine();
    Interface::forceUpdateDynamicBodies();
    //*/


    /*
    //this is a test with a sphere
    //std::string objs_models[]={"configuration_data/fluid_data/objects/boxTest.rbs"};
    std::string objs_models[]={"configuration_data/fluid_data/objects/ballTest.rbs"};

    //1.5,3.0,0.5
    int index = objects.size();
    int index_afs = AFs.size();

    for (int i=0; i< 1; ++i){
        std::string path;
        char effective_path[256];

        //first i need to add the part of the path to go to the configutation_data folder
        path=objs_models[i];
        path = interpret_path(path);

        //and now we cna use it
        strcpy(effective_path, path.c_str());
        AbstractRBEngine::loadRBsFromFile(effective_path);

        vect_objects_fluid_interaction.push_back(objects.back());
    }

    //so this is my test sphere
    //I'll set it's position manually for now// in the end the objects will be the one that
    //are part of the character so this problem will not exists
    RigidBody* body=vect_objects_fluid_interaction[0];
    body->setCMPosition(Point3d(1,1,0.5));
    body->setOrientation(Quaternion::getRotationQuaternion(0.7,Vector3d(0,0,1)));

    //and actually create the objects in ODE
    initODEStruct(index,index_afs);

    sendDataToParticleFluidEngine();
    Interface::forceUpdateDynamicBodies();
    //*/
}

void ODEWorld::create_cubes()
{
    if (SimGlobals::nb_container_boxes<1||SimGlobals::nb_filler<1){
        return;
    }

    std::string objs_models[2]={"configuration_data/objects/emptyBox.rbs",
                                "configuration_data/objects/fillerObj.rbs"};

    std::vector<RigidBody*> vect_models;

    for (int i=0; i< 2; ++i){
        std::string path;
        char effective_path[256];

        //first i need to add the part of the path to go to the configutation_data folder
        path=objs_models[i];
        path = interpret_path(path);

        //and now we cna use it
        strcpy(effective_path, path.c_str());
        AbstractRBEngine::loadRBsFromFile(effective_path);

        vect_models.push_back(objects.back());
        objects.pop_back();
    }

    int index = objects.size();
    int index_afs = AFs.size();

    Point3d p_box=vect_models.front()->getCMPosition();

    Point3d start_boxes_pos=p_box;
    start_boxes_pos.z-=1.5;

    Quaternion filler_ori=Quaternion::getRotationQuaternion(PI/4,Vector3d(1,0,0))*
            Quaternion::getRotationQuaternion(PI/4,Vector3d(0,1,0));

    Point3d newbox_pos(start_boxes_pos);
    for (int i=0; i< SimGlobals::nb_container_boxes;++i){

        TimeCube tc;
        RigidBody* newbox= vect_models.front()->cpy();
        newbox->setCMPosition(newbox_pos);
        objects.push_back(newbox);
        tc.cube=newbox;

        newbox_pos.z+=2;
        if ((i+1)%4==0 && i!=0){
            newbox_pos.y+=2;
            newbox_pos.z=start_boxes_pos.z;
        }

        Point3d start_fill_pos=newbox->getCMPosition();
        start_fill_pos.x-=0.4;
        start_fill_pos.y-=0.4;
        start_fill_pos.z-=0.4;
        Point3d newBody_pos=start_fill_pos;

        int count_filler=0;
        for (int i=0; (i<4) &&(count_filler<SimGlobals::nb_filler); ++i){
            newBody_pos.z+=0.15;
            newBody_pos.y=start_fill_pos.y+(i%2)*0.05;

            for (int j=0; (j< 4)&&(count_filler<SimGlobals::nb_filler); ++j){
                newBody_pos.y+=0.15;
                newBody_pos.x=start_fill_pos.x +((i+j)%2)*0.05;

                for (int k=0; (k< 4)&&(count_filler<SimGlobals::nb_filler); ++k){
                    newBody_pos.x+=0.15;

                    RigidBody* newBody= vect_models.back()->cpy();

                    newBody->setCMPosition(newBody_pos);


                    newBody->setOrientation(filler_ori);

                    tc.fillerObjs.push_back(newBody);
                    objects.push_back(newBody);
                    count_filler++;
                }
            }
        }

        vect_container_cubes.push_back(tc);

    }
    initODEStruct(index,index_afs);

    //I will now use the filler objet to create the motors
    index = objects.size();
    index_afs = AFs.size();

    vect_models.back()->setCMPosition(Point3d(0,3,0));
    vect_models.back()->name("interbody");
    objects.push_back(vect_models.back());

    initODEStruct(index,index_afs);


    dBodyID inter_id= odeToRbs[(int)(vect_models.back()->idx())].id;

    //create an hinge joint bettween the inter and the ground
    //*
    {
        dJointID j =dJointCreateSlider(worldID,0);
        dJointAttach(j, inter_id, odeToRbs[(int)(objects[0]->idx())].id);
        dJointSetSliderAxis (j,0,0,1);
    }
    //*/

    //now I have to set a linear motr between the inter and the ground
    //since all boxes are linked to the inter body I just need to move the inter body
    //*
    {
        dJointID j = dJointCreateLMotor(worldID, 0);
        dJointAttach(j, inter_id, odeToRbs[(int)(objects[0]->idx())].id);

        dJointSetLMotorNumAxes (j, 1);
        dJointSetLMotorAxis(j,0,1,0,0,1);
        dJointSetLMotorParam (j, dParamVel1, 0.7);
        dJointSetLMotorParam(j,dParamFMax1, 50000);
    }
    //*/

    //Now I need to create hinge joint between all containers and the inter body
    //and make them rotate
    for (int i=0;i<vect_container_cubes.size();++i){
        TimeCube &tc=vect_container_cubes[i];
        RigidBody* rb=tc.cube;
        dBodyID cube_id= odeToRbs[(int)(rb->idx())].id;
        dBodySetGravityMode(cube_id,0);


        //*
        {
            dJointID j =dJointCreateHinge(worldID,0);
            dJointAttach(j, cube_id, inter_id);
            Point3d p=rb->getCMPosition();
            dJointSetHingeAnchor(j, p.x, p.y, p.z);
            dJointSetHingeAxis (j, 1, 0, 0);
        }
        //*/

        //*
        //and now we make the container rotate
        {
            dJointID j = dJointCreateAMotor(worldID, 0);
            dJointAttach(j, cube_id, inter_id);

            dJointSetAMotorNumAxes (j, 3);
            dJointSetAMotorAxis(j,0,1,1,0,0);
            dJointSetAMotorAxis(j,2,2,0,0,1);
            dJointSetAMotorAngle (j,0,0);

            dJointSetAMotorParam(j,dParamVel1, 3);

            dJointSetAMotorParam(j,dParamFMax1, 500000);
        }
        //*/

    }


}
