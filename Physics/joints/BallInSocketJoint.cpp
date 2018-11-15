
#include "ballinsocketjoint.h"
#include <Utils/Utils.h>

#define ANGLE_A_CONSTRAINT		1
#define ANGLE_B_CONSTRAINT		2


BallInSocketJoint::BallInSocketJoint(const char* axes):Joint(){
    read_axes(axes);
}


BallInSocketJoint::BallInSocketJoint(const char *axes, FILE *f, AbstractRBEngine *world, ArticulatedFigure *figure)
        :BallInSocketJoint(axes){
    load_from_file(f, world);
    set_articuled_figure(figure);
}


BallInSocketJoint::~BallInSocketJoint(void){

}

//FILE* fp = fopen("jointAng.txt", "w");

/**
	This method is used to fix the joint angular constraint to correct for drift. This is done by changing
	the orientation of the child.
*/
void BallInSocketJoint::fix_angular_constraint(const Quaternion& qRel){


	//nothing to fix here (well maybe joint limits at some point)

/**  DEBUG only
	angleB = decomposeRotation(qRel, b, &angleA, &a);


	if (strcmp(this->joint->child->name, "torso") == 0){
		fprintf(fp, "%lf\t%lf\n", angleA, angleB);
		fflush(fp);
	}
*/
}

/**
	This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
	been read from an input file.
*/
void BallInSocketJoint::read_axes(const char* axes){
	if (sscanf(axes, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",&swingAxis1.x, &swingAxis1.y, &swingAxis1.z, &swingAxis2.x, &swingAxis2.y, &swingAxis2.z, &twistAxis.x, &twistAxis.y, &twistAxis.z) != 9){
		if (sscanf(axes, "%lf %lf %lf %lf %lf %lf",&swingAxis1.x, &swingAxis1.y, &swingAxis1.z, &twistAxis.x, &twistAxis.y, &twistAxis.z) != 6){
			throwError("Ball in socket joints require two or three axis to be specified!");
		}
		else
			swingAxis2 = swingAxis1.crossProductWith(twistAxis);
	}
	swingAxis1.toUnit();
	swingAxis2.toUnit();
	twistAxis.toUnit();
}

/**
	This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
	have been read from an input file.
*/
void BallInSocketJoint::read_joint_limits(const char* limits){
	int n = sscanf(limits, "%lf %lf %lf %lf %lf %lf", &minSwingAngle1, &maxSwingAngle1, &minSwingAngle2, &maxSwingAngle2, &minTwistAngle, &maxTwistAngle);
	if (n!= 6)
		throwError("Ball in socket joints require 6 joint limit parameters (min/max angle for the three axis)!");

    _use_joint_limits= true;
}


#include <sstream>
std::string BallInSocketJoint::to_xml(int nb_tab)
{
    std::ostringstream oss;
    //oss<<tab_string(nb_tab)<<"<joint type=\"ball\" "<<
    //     Joint::to_xml(nb_tab)<<
    //     //"range=\""<<_min_angle<<" "<<_max_angle<<"\" "<<
    //     "/>"<<std::endl;

    Point3d p=child_joint_position();
    oss<<tab_string(nb_tab)<<"<joint type=\"hinge\" "<<
         "name=\""<<name()<<"_1\" "<<
         "pos=\""<<-p.x<<" "<<p.z<<" "<<p.y<<"\" "<<
         "axis=\""<<-swingAxis1.x<<" "<<swingAxis1.z<<" "<<swingAxis1.y<<"\" "<<
         "range=\""<<minSwingAngle1<<" "<<maxSwingAngle1<<"\" "<<
         "/>"<<std::endl;

    oss<<tab_string(nb_tab)<<"<joint type=\"hinge\" "<<
         "name=\""<<name()<<"_2\" "<<
         "pos=\""<<-p.x<<" "<<p.z<<" "<<p.y<<"\" "<<
         "axis=\""<<-swingAxis2.x<<" "<<swingAxis2.z<<" "<<swingAxis2.y<<"\" "<<
         "range=\""<<minSwingAngle2<<" "<<maxSwingAngle2<<"\" "<<
         "/>"<<std::endl;

    oss<<tab_string(nb_tab)<<"<joint type=\"hinge\" "<<
         "name=\""<<name()<<"_3\" "<<
         "pos=\""<<-p.x<<" "<<p.z<<" "<<p.y<<"\" "<<
         "axis=\""<<-twistAxis.x<<" "<<twistAxis.z<<" "<<twistAxis.y<<"\" "<<
         "range=\""<<minTwistAngle<<" "<<maxTwistAngle<<"\" "<<
         "/>"<<std::endl;


    return oss.str();
}
