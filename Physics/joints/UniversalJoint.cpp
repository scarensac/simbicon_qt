
#include "universaljoint.h"
#include <Utils/Utils.h>

#define ANGLE_A_CONSTRAINT		1
#define ANGLE_B_CONSTRAINT		2

UniversalJoint::UniversalJoint(const char* axes):Joint(){
    read_axes(axes);
    minAngleA = 0;
    maxAngleA = 0;
    minAngleB = 0;
    maxAngleB = 0;
}

UniversalJoint::UniversalJoint(const char *axes, FILE *f, AbstractRBEngine *world, ArticulatedFigure *figure)
    :UniversalJoint(axes)
{
    load_from_file(f, world);
}

UniversalJoint::~UniversalJoint(void){

}



/**
    This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
    been read from an input file.
*/
void UniversalJoint::read_axes(const char* axes){
    if (sscanf(axes, "%lf %lf %lf %lf %lf %lf", &a.x, &a.y, &a.z, &b.x, &b.y, &b.z) != 6)
        throwError("Universal joints require two rotation axes to be provided as parameters!");

    a.toUnit();
    b.toUnit();
}

/**
    This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
    have been read from an input file.
*/
void UniversalJoint::read_joint_limits(const char* limits){
    if (sscanf(limits, "%lf %lf %lf %lf", &minAngleA, &maxAngleA, &minAngleB, &maxAngleB)!=4)
        throwError("Universal joints require 4 joint limites (minAngleA, maxAngleA, minAngleB, maxAngleB)!");
    else
        _use_joint_limits = true;
}

/**
    This method is used to fix the joint angular constraint to correct for drift. This is done by changing
        the orientation of the child.
*/
void UniversalJoint::fix_angular_constraint(const Quaternion& qRel){
    //to go from the child's coord frame to its parent, first rotate around the axis b, then around the axis a.
    Quaternion tmpQ1, tmpQ2;
    //compute two rotations, such that qRel = tmpQ1 * tmpQ2, and tmpQ2 is a rotation about the vector b (expressed in child coordinates)
    qRel.decomposeRotation(&tmpQ1, &tmpQ2, b);

    //now make sure that tmpQ1 represents a rotation about axis a (expressed in parent coordinates)
    double angA = tmpQ1.getRotationAngle(a);
    Vector3d tmpV1 = tmpQ1.v;
    tmpV1.toUnit();
    double mod = tmpV1.dotProductWith(a);
    if (mod < 0) mod = -mod;
    angA *= mod;
    _child->state.orientation = _parent->state.orientation * Quaternion::getRotationQuaternion(angA, a) * tmpQ2;
}


#include <sstream>
std::string UniversalJoint::to_xml(int nb_tab)
{
    std::ostringstream oss;

    Point3d p=child_joint_position();
    oss<<tab_string(nb_tab)<<"<joint type=\"hinge\" "<<
         "name=\""<<name()<<"_1\" "<<
         "pos=\""<<-p.x<<" "<<p.z<<" "<<p.y<<"\" "<<
         "axis=\""<<-a.x<<" "<<a.z<<" "<<a.y<<"\" "<<
         "range=\""<<minAngleA<<" "<<maxAngleA<<"\" "<<
         "/>"<<std::endl;

    oss<<tab_string(nb_tab)<<"<joint type=\"hinge\" "<<
         "name=\""<<name()<<"_2\" "<<
         "pos=\""<<-p.x<<" "<<p.z<<" "<<p.y<<"\" "<<
         "axis=\""<<-b.x<<" "<<b.z<<" "<<b.y<<"\" "<<
         "range=\""<<minAngleB<<" "<<maxAngleB<<"\" "<<
         "/>"<<std::endl;

    return oss.str();
}
