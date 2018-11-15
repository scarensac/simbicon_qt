#ifndef RB_STATE_H
#define RB_STATE_H


#include <MathLib/Point3d.h>
#include <MathLib/Vector3d.h>
#include <MathLib/Quaternion.h>

/*======================================================================================================================================================================*
 * This class acts as a container for the state information (position, orientation, velocity and angular velocity - all of them stored in world coordinates, about the  *
 * center of mass) of a rigid body.                                                                                                                                     *
 *======================================================================================================================================================================*/

class RBState{
public:
	//NOTE: all the quantities here are in world coordinates

	// the position of the center of mass of the rigid body
	Point3d position;
	// its orientation
	Quaternion orientation;
    // the velocity of the center of mass
    Vector3d velocity;
    // the angular velocity about the center of mass
    Vector3d angular_velocity;
    // the acceleration of the center of mass
    Vector3d acc;
    // the angular acceleration about the center of mass
    Vector3d angular_acc;
	
public:
	/**
		Default constructor - populate the data members using safe values..
	*/
	RBState(void);

	/**
		A copy constructor.
	*/
	RBState(const RBState& other);

	/**
		and a copy operator	
	*/
	RBState& operator = (const RBState& other);
	/**
		Default destructor.
	*/
	~RBState(void);
};

#endif
