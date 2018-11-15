#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <string>
#include <Utils/Utils.h>
#include <Physics/rb/RBState.h>
#include <Physics/rb/RBProperties.h>
#include <GLUtils/GLMesh.h>
#include <MathLib/TransformationMatrix.h>
#include <Physics/cdp/CollisionDetectionPrimitive.h>
#include <GLUtils/GLUtils.h>

class Force;
class ArticulatedFigure;


//define some drawing flags:
#define SHOW_MESH				0x0001
#define SHOW_BODY_FRAME			0x0002
#define SHOW_CD_PRIMITIVES		0x0008
#define SHOW_MIN_BDG_SPHERE		0x0010
#define SHOW_JOINTS				0x0020
#define SHOW_COLOURS			0x0040
#define SHOW_FRICTION_PARTICLES 0x0080

/*=========================================================================================================================================================================*
 | This is the implementation of a Rigid Body class. It holds all the attributes that characterize the rigid body (state information, collision detection primitives, etc).|
 | This class is used as the basis for an Articulated Rigid Body. Together with the PhysicalWorld class, this class is used to implement the dynamics of rigid bodies.     |
 | NOTE: It is assumed that the location of the center of mass of the object in local coordinates is (0,0,0) and that the principal moments of inertia are aligned to the  |
 | local coordinates x, y, and z axes!                                                                                                                                     |
 *=========================================================================================================================================================================*/

class RigidBody  {
friend class AbstractRBEngine;
friend class HingeJoint;
friend class UniversalJoint;
friend class BallInSocketJoint;
friend class ODEWorld;
friend class Character;
friend class SimBiController;
friend class Joint;
friend class PoseController;
friend class SoftBodyPointMass;


protected:
	//--> the state of the rigid body: made up of the object's position in the world, its orientation and linear/angular velocities (stored in world coordinates)
	RBState state;
	//--> the physical properties of the rigid bodies: mass and inertia, stored in a convenient to use form
	RBProperties props;
	//--> an array with all the collision detection primitives that are relevant for this rigid body
	DynamicArray<CollisionDetectionPrimitive*> cdps;
	//--> the mesh(es) that are used when displaying this rigid body
	DynamicArray<GLMesh*> meshes;
	//--> the name of the rigid body - it might be used to reference the object for articulated bodies
    std::string _name;
	//--> the id of the rigid body
    int _idx;


	//--> this transformation matrix is used to transform points/vectors from local coordinates to global coordinates. It will be updated using the state
	//information, and is therefore redundant, but it will be used to draw the object quickly. Everytime the state is updated, this matrix must also be updated!
//	TransformationMatrix toWorld;
    float r,g,b,a;

public:
    //the next members will be used to store an avg on the velocity so that we are sure to have a continuous velcity
    std::vector<Vector3d> vect_angular_velocity_avg;
    Vector3d angular_velocity_avg;
    int angular_vel_avg_max_size;
	/**
		Default constructor
	*/
    RigidBody();

    /**
        constructor from file
     */
    RigidBody(FILE* f);

    /**
		Default destructor
	*/
	virtual ~RigidBody(void);

    /**
     * @brief return a copy of the object. If keep_graphics==false then mesh are not copied
     */
    virtual RigidBody* cpy(bool keep_graphics=false);

    /**
        getters.
    */
    inline std::string name(){return _name;}
    inline int idx(){return _idx;}
    inline const DynamicArray<CollisionDetectionPrimitive*>& get_cdps(){
        return cdps;
    }

    /**
        setters.
    */
    inline void idx(int idx){_idx=idx;}
    inline void name(std::string name){_name=name;}
    inline void set_props(RBProperties i_props){props=i_props;}
    inline void add_cdp(CollisionDetectionPrimitive* i_cdp){cdps.push_back(i_cdp);}

	/**
		This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	Point3d getWorldCoordinates(const Point3d& localPoint);

	/**
		This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
	*/
	Point3d getLocalCoordinates(const Point3d& globalPoint);

	/**
		This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
	*/
	Vector3d getLocalCoordinates(const Vector3d& globalVector);

	/**
		This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	Vector3d getWorldCoordinates(const Vector3d& localVector);

        /**
                This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
                resulting velocity will be expressed in world coordinates.
        */
        Vector3d getAbsoluteVelocityForLocalPoint(const Point3d& localPoint);

        /**
                This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
                resulting velocity will be expressed in world coordinates.
        */
        Vector3d getAbsoluteVelocityAvgForLocalPoint(const Point3d& localPoint);

	/**
		This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
		resulting velocity will be expressed in world coordinates.
	*/
	Vector3d getAbsoluteVelocityForGlobalPoint(const Point3d& globalPoint);

	/**
		This method returns the world coordinates of the position of the center of mass of the object
	*/
    inline Point3d getCMPosition(){return state.position;}

    /**
        This method returns the body's center of mass velocity
    */
    inline Vector3d getCMVelocity(){return state.velocity;}

    /**
        this method returns the body's orientation
    */
    inline Quaternion getOrientation(){return state.orientation;}

    /**
     this function compute th estimated orientation at the next time step by usng the angular velocity
     */
    Quaternion getOrientationFuture();

    /**
        this method returns the body's angular velocity
    */
    inline Vector3d getAngularVelocity(){return state.angular_velocity;}

    /**
        this function etimate the angular velocity at the net time step by usng the angular acceleration
    */
    Vector3d getAngularVelocityFuture();



    /**
		This method sets the world coordinate of the posision of the center of mass of the object
    */
    inline void setCMPosition(const Point3d& newCMPos){state.position = newCMPos;}
    inline void setCMPosition(const double* newCMPos){state.position.x = newCMPos[0];
                                                      state.position.y = newCMPos[1];state.position.z = newCMPos[2];}


	/**
		This method sets the velocity of the center of mass of the object
    */
    inline void setCMVelocity(const Vector3d& newCMVel){state.velocity = newCMVel;}
    inline void setCMVelocity(const double* newCMVel){state.velocity.x = newCMVel[0];
                                                     state.velocity.y = newCMVel[1];state.velocity.z = newCMVel[2];}

    /**
        this method sets the body's orientation
    */
    inline void setOrientation(Quaternion q){state.orientation = q;}
    inline void setOrientation(const double* q){state.orientation.s = q[0];state.orientation.v.x = q[1];
                                               state.orientation.v.y = q[2];state.orientation.v.z = q[3];}


    /**
		this method sets the angular velocity of the body
    */
    inline void setAngularVelocity(const Vector3d& newAVel){state.angular_velocity = newAVel;}
    inline void setAngularVelocity(const double* newAVel){state.angular_velocity.x = newAVel[0];
                                                         state.angular_velocity.y = newAVel[1];state.angular_velocity.z = newAVel[2];}



    /**
		This method returns the rigid body's coefficient of restitution
	*/
	inline double getRestitutionCoefficient(){
		return props.epsilon;
	}

	/**
		This method returns the rigid body's coefficient of restitution
	*/
	inline double getFrictionCoefficient(){
		return props.mu;
	}

	/**
		This method draws the current rigid body.
	*/
	virtual void draw(int flags);

	/**
		This method renders the rigid body in its current state as a set of vertices 
		and faces that will be appended to the passed OBJ file.

		vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
		multiple different meshes to the same OBJ file
		 
		Returns the number of vertices written to the file
	*/
	uint renderToObjFile(FILE* fp, uint vertexIdxOffset);


	/**
		This method loads all the pertinent information regarding the rigid body from a file.
	*/
	void loadFromFile(FILE* fp);

	/**
		Returns the mass of the rigid body
	*/
    inline double getMass(){
        return props.mass;
    }
    inline void setMass(double mass){
        props.set_mass(mass);
    }
    inline void setMOI(Vector3d moi){
        props.set_MOI(moi.x,moi.y,moi.z);
    }

	/**
		This method is used to compute the correct toWorld coordinates matrix based on the state of the rigid body
	*/
//	void updateToWorldTransformation();


	/**
		this method returns the body's principal moments of inertia, about the principal axes (which correspond to the bodie's local coordinate
		frame axes)
	*/
	inline Vector3d getPMI(){
		return props.MOI_local;
	}



	/**
		this method returns true if the current body is locked, false otherwised
	*/
	inline bool isLocked(){
		return props.isLocked;
	}

	/**
		this method returns false if this body is a simple rigid bory, false if it is an articulated figure
	*/
	virtual bool isArticulated(){
		return false;
	}

	/**
		this method is used to update the world positions of the collision detection primitives
	*/
	void updateWorldCDPs();

    virtual ArticulatedFigure* af(){return NULL;}

	/**
	this function is used to set the mesh color (in case u wanna change it during the execution, to follow the stance foot for exemple)
	*/
	void set_mesh_color(float r, float g, float b, float a){
		for (int i = 0; i < (int)meshes.size(); ++i){
			meshes[i]->setColour(r, g, b, a);
		}
	}


    void set_state(RBState astate){
        state=astate;
    }

    void cpy_state(RigidBody* rb){
       set_state(rb->state);
    }

    /**
     * @brief this function should be called once (and only once) every simulation step
     */
    void update_angular_velocity_avg(){
        vect_angular_velocity_avg.push_back(getAngularVelocity());

        if (vect_angular_velocity_avg.size()>angular_vel_avg_max_size){
            vect_angular_velocity_avg.erase(vect_angular_velocity_avg.begin());
        }

        Vector3d wRel_avg=Vector3d(0,0,0);
        for (int j=0;j<vect_angular_velocity_avg.size();++j){
            wRel_avg+=vect_angular_velocity_avg[j];
        }
        wRel_avg=wRel_avg/((double)vect_angular_velocity_avg.size());
        angular_velocity_avg=wRel_avg;
    }

    bool angular_velocity_avg_is_valid(){return vect_angular_velocity_avg.size()==angular_vel_avg_max_size;}

    /**
     * @brief get_angular_velocity_avg
     * @return values are in world coordinates btw....
     */
    Vector3d get_angular_velocity_avg(){return angular_velocity_avg;}

    void set_angular_vel_avg_max_size(int val){angular_vel_avg_max_size=val;}

    void reset_angular_velocity_avg(){vect_angular_velocity_avg.clear();}

};

#endif
