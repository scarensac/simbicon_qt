
#include "hingejoint.h"
#include <Utils/Utils.h>

HingeJoint::HingeJoint(const char* axes):Joint(){
    read_axes(axes);
    _min_angle = 0;
    _max_angle = 0;
}

HingeJoint::HingeJoint(const char *axes, FILE *f, AbstractRBEngine *world, ArticulatedFigure *figure)
        :HingeJoint(axes){
    load_from_file(f, world);
    set_articuled_figure(figure);
}

HingeJoint::~HingeJoint(void){

}

/**
	This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
	been read from an input file.
*/
void HingeJoint::read_axes(const char* axes){
    if (sscanf(axes, "%lf %lf %lf", &_axis.x, &_axis.y, &_axis.z) != 3)
		throwError("Hinge joints require the rotation axis to be provided as a parameter!");

    _axis.toUnit();

}

/**
	This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
	have been read from an input file.
*/
void HingeJoint::read_joint_limits(const char* limits){
    if (sscanf(limits, "%lf %lf", &_min_angle, &_max_angle) != 2)
		throwError("Two parameters are needed to specify joint limits for a hinge joint!");
    _use_joint_limits = true;
}


//FILE* fp = fopen("jointAng.txt", "w");


/**
	This method is used to fix the joint angular constraint to correct for drift. This is done by changing
		the orientation of the child.
*/
void HingeJoint::fix_angular_constraint(const Quaternion& qRel){
	//make sure that the relative rotation between the child and the parent is around the a axis
	Vector3d axis = qRel.getV().toUnit();
	//this is the rotation angle around the axis above, which may not be the rotation axis
	double rotAngle = 2 * safeACOS(qRel.getS());
	//get the rotation angle around the correct axis now (we are not in the world frame now)
    double ang = axis.dotProductWith(_axis) * rotAngle;

/* DEBUG ONLY
	if (strcmp(parent->name, "lupperarm") == 0 || strcmp(parent->name, "rupperarm") == 0){
		fprintf(fp, "%lf\n", ang);
		fflush(fp);
	}
*/
	//and compute the correct child orientation
    _child->state.orientation = _parent->state.orientation * Quaternion::getRotationQuaternion(ang, _axis);
}


#include <sstream>
std::string HingeJoint::to_xml(int nb_tab)
{
    std::ostringstream oss;
    oss<<tab_string(nb_tab)<<"<joint type=\"hinge\" "<<
         Joint::to_xml(nb_tab)<<
         "axis=\""<<-_axis.x<<" "<<_axis.z<<" "<<_axis.y<<"\" "<<
         "range=\""<<_min_angle<<" "<<_max_angle<<"\" "<<
         "/>"<<std::endl;

    return oss.str();

}
