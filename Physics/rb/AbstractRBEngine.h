#ifndef ABSTRACT_RB_ENGINE_H
#define ABSTRACT_RB_ENGINE_H



#include <Utils/Utils.h>
#include <Physics/rb/RigidBody.h>
#include <Physics/rb/ArticulatedRigidBody.h>
#include <Physics/rb/ArticulatedFigure.h>
#include <Physics/ContactPoint.h>



/*--------------------------------------------------------------------------------------------------------------------------------------------*
 * This class implements a container for rigid bodies (both stand alone and articulated). It reads a .rbs file and interprets it.             *
 *--------------------------------------------------------------------------------------------------------------------------------------------*/
class AbstractRBEngine{
friend class ConCompositionFramework;
protected:
	//this is a list of all the objects in the world
	DynamicArray<RigidBody*> objects;
	//keep a list of all the articulated rigid bodies in the world as well, because we need to make sure that the user 
	//only ever links through joints articulated bodies, not just simple rigid bodies
	DynamicArray<ArticulatedRigidBody*> ABs;
	//we will also keep a list of all the articulated figures in the world, because we will need to access their joints, etc.
	DynamicArray<ArticulatedFigure*> AFs;
	//we'll keep a list of all the joints in the world as well, for quick access
	DynamicArray<Joint*> jts;

	//this is a list of all the contact points
	DynamicArray<ContactPoint> contactPoints;

public:
	//the constructor
	AbstractRBEngine(void);
	//the destructor
	virtual ~AbstractRBEngine(void);

	/**
		This method is used to call the draw method of all the rigid bodies in the world
	*/
    virtual void drawRBs(int flags = SHOW_MESH);

	/**
		This method renders all the rigid bodies as a set of vertices 
		and faces that will be appended to the passed OBJ file.

		vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
		multiple different meshes to the same OBJ file
		 
		Returns the number of vertices written to the file
	*/
	uint renderRBsToObjFile(FILE* fp, uint vertexIdxOffset);


	/**
		This method is used to return a pointer to the list of contact forces
	*/
	inline DynamicArray<ContactPoint>* getContactForces(){
		return &contactPoints;
    }

    /**
        This method is used to integrate the forward simulation in time.
    */
    virtual void advanceInTime(double deltaT)=0;

    virtual void readDataFromEngine()=0;

	/**
		This method reads a list of rigid bodies from the specified file.
	*/
	virtual void loadRBsFromFile(char* fName);

    /**
        This method returns the reference to the articulated rigid body with the given name, or NULL if it is not found
    */
    ArticulatedRigidBody* getARBByName(std::string name);


    /**
        This method returns the number of articulated rigid bodies
    */
    int getARBCount(){return (int)ABs.size();}

    /**
        This method returns the number of rigid bodies
    */
    int getRBCount(){return (int)objects.size();}

    /**
        This method returns the reference to the articulated rigid body with the given name, or NULL if it is not found
    */
    ArticulatedRigidBody* getARBByPositionInData(int i){
        if (i>=0&&i<ABs.size()){return ABs[i];} return NULL;
    }

	/**
		This method returns the reference to the rigid body with the given name, or NULL if it is not found
	*/
    RigidBody* getRBByName(std::string name);

	/**
		This method is used to get the state of all the rigid body in this collection.
	*/
	void getState(DynamicArray<double>* state);

	/**
		This method is used to set the state of all the rigid body in this collection.
	*/
	void setState(DynamicArray<double>* state, int start = 0);

	/**
		This method returns the number of articulated figures in this collection.
	*/
	inline int getAFCount(){
        return (int)AFs.size();
	}

	/**
		This method returns the a reference to the ith articulated body
	*/
	inline ArticulatedFigure* getAF(int i){
		if (i<0 || (uint)i>=AFs.size())
			return NULL;
		return AFs[i];
	}

	/**
		This method is used to update the toWorld transformations for the rigid bodies in the world
	*/
//	inline void updateTransformations(){
//		//and now make sure that each rigid body's toWorld transformation is updated
//		for (uint i=0;i<objects.size();i++)
//			objects[i]->updateToWorldTransformation();
//	}

	/**
		this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
		and the force is also specified in local coordinates.
	*/
	virtual void applyRelForceTo(RigidBody* b, const Vector3d& f, const Point3d& p) = 0;

	/**
		this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
		and the force is specified in world coordinates.
	*/
	virtual void applyForceTo(RigidBody* b, const Vector3d& f, const Point3d& p) = 0;

	/**
		this method applies a torque to a rigid body. The torque is specified in world coordinates.
	*/
	virtual void applyTorqueTo(RigidBody* b, const Vector3d& t) = 0;

	/**
		another test method for advancing the world forward in time...
	*/
	virtual void runTest(){}


};

#endif
