#ifndef UNIVERSAL_JOINT_H
#define UNIVERSAL_JOINT_H

#include <Physics/joints/Joint.h>

/*======================================================================================================================================================================*
 * This class is used to implement a universal joint - angular impulses that allow only two degrees of freedom between the parent and the child must be computed.       *
 *======================================================================================================================================================================*/

class UniversalJoint : public Joint{
friend class ODEWorld;
private:
	//This joint can only rotate about the vector a, that is stored in parent coordinates
	Vector3d a;
	//or about vector b that is stored in child coordinates
	Vector3d b;
	//and the min and max allowed angles (around a axis)
	double minAngleA, maxAngleA;
	//and around the b axis
	double minAngleB, maxAngleB;

public:
    UniversalJoint(const char* axes);
    UniversalJoint(const char* axes, FILE* f, AbstractRBEngine* world, ArticulatedFigure* figure);
    ~UniversalJoint(void);

    UniversalJoint(UniversalJoint *j):Joint((Joint*)j){
        a=j->a;
        b=j->b;
        minAngleA=j->minAngleA;
        maxAngleA=j->maxAngleA;
        minAngleB=j->minAngleB;
        maxAngleB=j->maxAngleB;
    }

	/**
		This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
		been read from an input file.
	*/
    virtual void read_axes(const char* axes);

	/**
		This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
		have been read from an input file.
	*/
    virtual void read_joint_limits(const char* limits);

	/**
		This method is used to fix the joint angular constraint to correct for drift. This is done by changing
		the orientation of the child.
	*/
    virtual void fix_angular_constraint(const Quaternion& qRel);

	/**
		Return the A rotation axis
	*/
    inline Vector3d axis_a(){return a;}

	/**
		Return the B rotation axis
	*/
    inline Vector3d axis_b(){return b;}


	/**
		Returns the type of the current joint
	*/
    virtual int get_joint_type(){ return UNIVERSAL_JOINT;}

    std::string to_xml(int nb_tab);
};


#endif
