#ifndef BALL_IN_SOCKET_JOINT_H
#define BALL_IN_SOCKET_JOINT_H

#include <Physics/joints/Joint.h>

/*==============================================================================================================================================================*
 * This class implements a ball in socket joint type.                                                                                                           *
 *==============================================================================================================================================================*/
class BallInSocketJoint : public Joint{
    friend class ODEWorld;
private:
    /**
    Quantities that do not change
*/

    //BallInSocket joints are free to rotate about any axis. However, in order to be able to apply joint limits, we will
    //identify some axis, about which we will place joint limits. In particular, we'll use a swing and twist decomposition
    //of the relative orientations between the two bodies of a hinge joint

    //these two axes define the plane of vectors along which the rotations represent a swing - stored in parent coordinates
    Vector3d swingAxis1, swingAxis2;
    //and this one is stored in child coordinates - this is the twist axis
    Vector3d twistAxis;
    //and the min and max allowed angles along the two swing axes (define an ellipsoid that can be offset if the min/max angles are not equal in magnitude)
    double minSwingAngle1, maxSwingAngle1, minSwingAngle2, maxSwingAngle2;
    //and limits around the twist axis
    double minTwistAngle, maxTwistAngle;
public:
    BallInSocketJoint(const char* axes);
    BallInSocketJoint(const char *axes, FILE *f, AbstractRBEngine *world, ArticulatedFigure *figure);
    BallInSocketJoint(BallInSocketJoint *j):Joint((Joint*)j){
        swingAxis1=j->swingAxis1;
        swingAxis2=j->swingAxis2;
        twistAxis=j->twistAxis;
        minSwingAngle1=j->minSwingAngle1;
        maxSwingAngle1=j->maxSwingAngle1;
        minSwingAngle2=j->minSwingAngle2;
        maxSwingAngle2=j->maxSwingAngle2;
        minTwistAngle=j->minTwistAngle;
        maxTwistAngle=j->maxTwistAngle;
    }

    ~BallInSocketJoint(void);

    Vector3d get_swingAxis1(){return swingAxis1;}
    Vector3d get_swingAxis2(){return swingAxis2;}
    Vector3d get_twistAxis(){return twistAxis;}

    /**
        This method is used to fix the joint angular constraint to correct for drift. This is done by changing
        the orientation of the child.
    */
    virtual void fix_angular_constraint(const Quaternion& qRel);

    /**
        This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
        have been read from an input file.
    */
    virtual void read_joint_limits(const char* limits);

    /**
        This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
        been read from an input file.
    */
    virtual void read_axes(const char* axes);

    /**
        Returns the type of the current joint
    */
    virtual int get_joint_type(){ return BALL_IN_SOCKET_JOINT;}

    std::string to_xml(int nb_tab);
};

#endif
