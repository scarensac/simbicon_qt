#ifndef ODE_WORLD_H
#define ODE_WORLD_H

#include <Physics/rb/AbstractRBEngine.h>
#include <ode/ode.h>
//#include <ode/joint.h>
#include <Physics/cdp/CollisionDetectionPrimitive.h>
#include <Physics/cdp/SphereCDP.h>
#include <Physics/cdp/CapsuleCDP.h>
#include <Physics/cdp/BoxCDP.h>
#include <Physics/cdp/PlaneCDP.h>
#include <Physics/PreCollisionQuery.h>


#include "Core\ForcesUtilitary.h"


#define MAX_CONTACT_FEEDBACK 200

class Character;
//this structure is used to map a rigid body to the id of its ODE counterpart
typedef struct ODE_RB_Map_struct{
	dBodyID id;
	RigidBody* rb;
    ODE_RB_Map_struct(dBodyID newId, RigidBody* newRb){ this->id = newId; this->rb = newRb;}
} ODE_RB_Map;

/*-------------------------------------------------------------------------------------------------------------------------------------------------*
 * This class is used as a wrapper that is designed to work with the Open Dynamics Engine. It uses all the rigid bodies (together with the joints) *
 * that are loaded with RBCollection, and has methods that link with ODE to simulate the physics. If a different physics engine is to be used,     *
 * then ideally only the methods of this class need to be re-implemented, and the rest of the application can stay the same.                       *
 *-------------------------------------------------------------------------------------------------------------------------------------------------*/
class SoftBodyPointMass;
class ODEWorld : public AbstractRBEngine{
friend void collisionCallBack(void* odeWorld, dGeomID o1, dGeomID o2);

public:

std::vector<SoftBodyPointMass*> vect_soft_bodies_point_mass;
dGeomID ground_geom_id;

struct TimeCube{
    RigidBody* cube;
    RigidBody* inter_body;
    std::vector<RigidBody*> fillerObjs;
};


protected:
    //this boolean store the fact that this world has been initialized so that it can be run
    //simultenaously to other worlds

    bool m_is_parallel_world;
    dThreadingImplementationID m_ode_threading_id;

	// ODE's id for the simulation world
    dWorldID worldID;
	// id of collision detection space
	dSpaceID spaceID;
	// id of contact group
	dJointGroupID contactGroupID;
	//keep track of the mapping between the rigid bodies and their ODE counterparts with this
	DynamicArray<ODE_RB_Map> odeToRbs;

	//keep an array of contact points that is used for each pair of geom collisions
	dContact *cps;
    //this is the max number of contacts that are going to be processed between any two objects
	int maxContactCount;

	dJointFeedback jointFeedback[MAX_CONTACT_FEEDBACK];
	//this is the current number of contact joints, for the current step of the simulation
	int jointFeedbackCount;

	//this is a pointer to a physical interface object that is used as an abstract way of communicating between the simulator and the application
	PreCollisionQuery* pcQuery;

    std::vector<TimeCube> vect_container_cubes;

    //this is a list of all the objects in the world
    std::vector<RigidBody*> vect_objects_fluid_interaction;

    int nb_collisions_cumul;
    double time_counter;

	/**
		This method is used to set up an ode hinge joint, based on the information in the hinge joint passed in as a parameter
	*/
	void setupODEHingeJoint(HingeJoint* hj);

	/**
		This method is used to set up an ode universal joint, based on the information in the universal joint passed in as a parameter
	*/
	void setupODEUniversalJoint(UniversalJoint* uj);

	/**
		This method is used to set up an ode ball-and-socket joint, based on the information in the ball in socket joint passed in as a parameter
	*/
	void setupODEBallAndSocketJoint(BallInSocketJoint* basj);

	/**
		this method is used to copy the state of the ith rigid body to its ode counterpart.
	*/
	void setODEStateFromRB(int i);

	/**
		this method is used to copy the state of the ith rigid body, from the ode object to its rigid body counterpart 
	*/
	void setRBStateFromODE(int i);

	/**
		this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
	*/
	dGeomID getSphereGeom(SphereCDP* s);

	/**
		this method is used to set up an ODE box geom. It is properly placed in body coordinates.
	*/
	dGeomID getBoxGeom(BoxCDP* b);

	/**
		this method is used to set up an ODE plane geom. It is properly placed in body coordinates.
	*/
	dGeomID getPlaneGeom(PlaneCDP* p, RigidBody* parent);

	/**
		this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
	*/
	dGeomID getCapsuleGeom(CapsuleCDP* c);

    /**
        this method is used to process the collision between the two objects passed in as parameters. More generally,
        it is used to determine if the collision should take place, and if so, it calls the method that generates the
        contact points.
    */
    void processCollisions(dGeomID o1, dGeomID o2);


	/**
		this method is used to create ODE geoms for all the collision primitives of the rigid body that is passed in as a paramter
	*/
	void createODECollisionPrimitives(RigidBody* body);

	/**
		this method is used to transfer the state of the rigid bodies, from the simulator to the rigid body wrapper
	*/
	virtual void setRBStateFromEngine();

	/**
		this method is used to transfer the state of the rigid bodies, from the rigid body wrapper to the simulator's rigid bodies
	*/
	virtual void setEngineStateFromRB();


public:
	/**
		default constructor
	*/
    ODEWorld(bool i_is_parallel_world=false);

	/**
		destructor
	*/
	virtual ~ODEWorld(void);



    int getJointCount(){return (int)jts.size();}
    //this function does not get the joint by it's idx but by it's posiotn in the vertor (it is used to iterate...)
    Joint* getJoint(int i){if (i>=0&&i<getJointCount()){return jts[i];} return NULL;}

    void create_contact(dContact *cps_bd, int contact_idx, dBodyID bd1_id, dBodyID bd2_id, RigidBody* rb1, RigidBody* rb2);

    /**
     * @brief get an RB by it's index
     * I need this function for special use. Don't use it unless you know for sure the structure of the objects
     */
    RigidBody* getRBByIdx(int idx){
        return objects[idx];
    }

    /**
     * @brief get_rb_list
     * @return
     */
    const std::vector<RigidBody*>& get_rb_list(){return objects;}

    /**
     * @brief add a copy of the rbs given as parameters inside the world
     */
    virtual void loadRBsFromList(std::vector<RigidBody*> vect_rbs);

    /**
     * @brief add a copy of the rbs given as parameters inside the world.
     * The arbs need to be part of the same figure
     * !! The first arbs will be considered as the root of the new figure !!
     */
    virtual void loadARBsFromList(std::vector<ArticulatedRigidBody*> vect_arbs);

    /**
     * @brief this has to be called after adding some RB, ARB, or AF
     * leave to -1 if you don't wanna handle one type (if you know you haven't added one)
     * if you hava added something in one of the type the start value represent the
     * number of element there was before the addition
     */
    virtual void initODEStruct(int start_RB=-1,int start_AF=-1);

    /**
     * @brief initODEMassProperties this need to be called if the moment of inertia of a bodie is modified
     * for simplicity this check all the bodies and update them all even if they have not been changed
     */
    virtual void initODEMassProperties();

	/**
		This method reads a list of rigid bodies from the specified file.
	*/
	virtual void loadRBsFromFile(char* fName);


    virtual void sendDataToEngine(RigidBody* root_bdy=NULL,Vector3d sup_root_torque=Vector3d(0,0,0));

    dBodyID get_body_id(RigidBody* rb);

	/**
		This method is used to integrate the forward simulation in time.
	*/
	virtual void advanceInTime(double deltaT);

    virtual void readDataFromEngine();


    virtual void sendDataToParticleFluidEngine();
    virtual void advanceInTimeParticleFluidEngine(double deltaT);
    virtual void readDataFromParticleFluidEngine();

    /**
        This method is used to set the state of all the rigid body in the physical world.
    */
    void setState(DynamicArray<double>* state, int start = 0);

    /**
		this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
		and the force is also specified in local coordinates.
	*/
	virtual void applyRelForceTo(RigidBody* b, const Vector3d& f, const Point3d& p);


    /**
        this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
        and the force is specified in world coordinates.
    */
    virtual void applyForceTo(RigidBody* b, const Vector3d& f, const Point3d& p);

    /**
        this method applies a force to a rigid body, at the specified point. The point is specified in world coordinates,
        and the force is specified in world coordinates.
    */
    virtual void applyForceToWorldPos(RigidBody* b, const Vector3d& f, const Point3d& p);

	/**
		this method applies a torque to a rigid body. The torque is specified in world coordinates.
	*/
	virtual void applyTorqueTo(RigidBody* b, const Vector3d& t);

    /**
     * @brief create the joint to fix the two rigid bodies in the simulation
     * @param bd1
     * @param bd2
     */
    dJointID setupODEFixedJoint(RigidBody *bd1, RigidBody *bd2);
    void destroyODEFixedJoint(dJointID joint_id);




	/**
		this method is used to compute the effect of water (it convert a level of water into the induced forces
	*/
    void compute_water_impact(Character* character,float water_level,   WaterImpact& resulting_impact);



	/**
		this function is a children function of the above one (it prevent mass duplication of code for similar body parts
		this function handle
	*/
	Vector3d compute_liquid_drag_on_toes(Joint* joint, float water_level, double eff_density);
	Vector3d compute_liquid_drag_on_feet(Joint* joint, float water_level, double eff_density, double friction_coef);
	Vector3d compute_liquid_drag_on_legs(Joint* joint, float water_level, double eff_density, double friction_coef);

	/**
		this function is an utilitary that is used to compute the liquid forces on a rectangular plane
		The plande have to follow one of the main directions (meaning the normal have to be one of the world basis vectors)
		parameters explanation:
		body: body containing the geometry
		l_x, l_y, l_z: those are the dimention of the plane (one of them should be equal to zero)
		pos: starting position of the algorithm on the face
		water_level: level of the water
		normal: normal of the face (so we know if it face the movement).
		nbr_interval_x, nbr_interval_x, nbr_interval_x: number of interval in each direction (same logic as the l_*)

	*/
	Vector3d compute_liquid_drag_on_plane(Joint* joint, double l_x, double l_y, double l_z, Point3d pos,
		Vector3d normal, float water_level, int nbr_interval_x, int nbr_interval_y, int nbr_interval_z);

	/*
	the default parameters indicate that the friction does not need to be computed on that plane
	*/
	Vector3d compute_liquid_drag_on_planev2(Joint* joint, Point3d pos, Vector3d normal, float water_level,
		Vector3d v1, Vector3d v2, int nbr_interval_v1, int nbr_interval_v2, double density, double friction_coef=0, double l3=0);


	/**
	This method compute and apply the forces caused by buoyancy.
	this version uses the physic representation of the objects to compute the immersed volume
	*/
	ForceStruct compute_buoyancy(Joint* joint, float water_level);
	ForceStruct compute_buoyancy_on_sphere(RigidBody* body, float water_level, double gravity, double density);
	ForceStruct compute_buoyancy_on_box(RigidBody* body, float water_level, double gravity, double density);
	ForceStruct compute_buoyancy_on_capsule(RigidBody* body, float water_level, double gravity, double density);


    void drawRBs(int flags);

    void initParticleFluid();
    void create_cubes();

};

#endif
