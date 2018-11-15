#ifndef HINGE_JOINT_H
#define HINGE_JOINT_H

#include <Physics/joints/Joint.h>


/*======================================================================================================================================================================*
 * This class is used to implement a hinge joint - angular impulses that allow relative rotation between the parent and the child only around a given axis must be      *
 * computed.                                                                                                                                                            *
 *======================================================================================================================================================================*/
class Joint;
class HingeJoint : public Joint{
private:
/**
	Quantities that do not change
*/
    //This joint only allows relative motion about axis - stored in parent coordinates
    Vector3d _axis;
	//keep track of the joint limits as well - min and max allowed angles around the rotation axis
    double _min_angle;
    double _max_angle;

public:
    HingeJoint(const char* axes);
    HingeJoint(const char *axes, FILE *f, AbstractRBEngine *world, ArticulatedFigure *figure);
    HingeJoint(HingeJoint *j):Joint((Joint*)j){
        _axis=j->_axis;
        _min_angle=j->_min_angle;
        _max_angle=j->_max_angle;
    }

	virtual ~HingeJoint(void);

    /**
        Returns the type of the current joint
    */
    virtual int get_joint_type(){return HINGE_JOINT;}

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
        getters
	*/
    inline Vector3d axis(){return _axis;}
    inline double min_angle(){return _min_angle;}
    inline double max_angle(){return _max_angle;}


    std::string to_xml(int nb_tab);
};

#endif
