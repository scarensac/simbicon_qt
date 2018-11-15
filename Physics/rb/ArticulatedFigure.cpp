
#include "articulatedfigure.h"
#include <Physics/rb/AbstractRBEngine.h>
#include <Physics/rb/RBUtils.h>
#include <Utils/Utils.h>
#include <iostream>
/**
	Default constructor
*/
ArticulatedFigure::ArticulatedFigure(void){
	root = NULL;
    _name="";
    _mass = 0;
}

ArticulatedFigure::ArticulatedFigure(ArticulatedRigidBody *aroot)
{
    _name="";
    _mass = 0;
    root = aroot;
    root->af(this);
}

ArticulatedFigure::~ArticulatedFigure(void){
}

/**
	This method is used to automatically fix the errors in the joints (i.e. drift errors caused by numercial integration). At some future
	point it can be changed into a proper stabilization technique.
*/
void ArticulatedFigure::fix_joint_constraints(bool fixOrientations, bool fixVelocities){
	if (!root)
		return;

    for (uint i=0;i<root->child_joints().size();i++)
        root->child_joints()[i]->fix_joint_constraints(fixOrientations, fixVelocities, true);
}

/**
	This method is used to get all the joints in the articulated figure and add them to the list of joints that is passed in as a paramter.
*/
void ArticulatedFigure::add_joints_to_list(DynamicArray<Joint*> *joints){
	if (!root)
		return;
	DynamicArray<ArticulatedRigidBody*> bodies;
	bodies.push_back(root);

	int currentBody = 0;

	while ((uint)currentBody<bodies.size()){
		//add all the children joints to the list
        for (uint i=0;i<bodies[currentBody]->child_joints().size();i++){
            joints->push_back(bodies[currentBody]->child_joints()[i]);
            bodies.push_back(bodies[currentBody]->child_joints()[i]->child());
		}
		currentBody++;
	}
}

/**
	This method is used to compute the total mass of the articulated figure.
*/
void ArticulatedFigure::compute_mass(){
	double curMass = root->getMass();
	double totalMass = curMass;
	DynamicArray <Joint*> joints;

	this->add_joints_to_list(&joints);

	for (uint i=0; i < joints.size(); i++){
        curMass = joints[i]->child()->getMass();
		totalMass += curMass;
	}

    _mass = totalMass;
}


/**
	This method is used to load the details of an articulated figure from file. The PhysicalWorld parameter points to the world in which the objects
	that need to be linked live in.
*/
void ArticulatedFigure::load_from_file(FILE* f, AbstractRBEngine* world){
	if (f == NULL)
		throwError("Invalid file pointer.");
	if (world == NULL)
		throwError("A valid physical world must be passed in as a parameter");
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	char tempName[100];
	Joint* tempJoint;

	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer)>195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		int lineType = getRBLineType(line);
		switch (lineType) {
			case RB_ROOT:
				sscanf(line, "%s", tempName);
				if (root != NULL)
					throwError("This articulated figure already has a root");
				root = world->getARBByName(tempName);
				if (root == NULL)
					throwError("The articulated rigid body \'%s\' cannot be found!", tempName);
				break;
			case RB_JOINT_TYPE_UNIVERSAL:
                tempJoint = new UniversalJoint(line, f, world, this);
				break;
			case RB_JOINT_TYPE_HINGE:
                tempJoint = new HingeJoint(line, f, world, this);
				break;
			case RB_JOINT_TYPE_BALL_IN_SOCKET:
                tempJoint = new BallInSocketJoint(line, f, world, this);
				break;
			case RB_END_ARTICULATED_FIGURE:
				//make sure that the root does not have a parent, otherwise we'll end up with loops in the articulated figure]
                if (root->parent_joint() != NULL)
					throwError("The root of the articulated figure is not allowed to have a parent!");
				return;//and... done
				break;
			case RB_NOT_IMPORTANT:
				if (strlen(line)!=0 && line[0] != '#')
                    std::cerr<<"Ignoring input line: "<<line<<std::endl;
				break;
			default:
				throwError("Incorrect articulated body input file: \'%s\' - unexpected line.", buffer);
		}
	}
	throwError("Incorrect articulated body input file! No /ArticulatedFigure found");
}

#include <fstream>
#include <sstream>


std::string ArticulatedFigure::to_xml(int nb_tab)
{
    return std::string();
    std::ostringstream text;
    int nbr_tab=0;
    text<<tab_string(nbr_tab++)<<"<mujoco>"<<std::endl;
    text<<tab_string(nbr_tab)<<"<compiler angle=\"radian\"/>"<<std::endl;


    text<<tab_string(nbr_tab++)<<"<default>"<<std::endl;

    text<<tab_string(nbr_tab++)<<"<default class=\"base_class\">"<<std::endl;
    text<<tab_string(nbr_tab)<<"<joint limited=\"true\" damping=\"0.2\" armature=\".01\"/>"<<std::endl;
    text<<tab_string(nbr_tab)<<"<position ctrllimited=\"true\" kp=\"10\"/>"<<std::endl;
    text<<tab_string(--nbr_tab)<<"</default>"<<std::endl;

    text<<tab_string(nbr_tab++)<<"<default class=\"free\">"<<std::endl;
    text<<tab_string(nbr_tab)<<"<joint type=\"free\" damping=\"0\" armature=\"0\" limited=\"false\"/>"<<std::endl;
    text<<tab_string(--nbr_tab)<<"</default>"<<std::endl;

    text<<tab_string(--nbr_tab)<<"</default>"<<std::endl;





    //start defining the structure
    text<<tab_string(nbr_tab++)<<"<worldbody>"<<std::endl;

    text<<tab_string(nbr_tab)<<"<light diffuse=\".5 .5 .5\" pos=\"0 0 3\" dir=\"0 0 -1\"/>"<<std::endl;
    text<<tab_string(nbr_tab)<<"<geom type=\"plane\" size=\"10 10 0.1\" rgba=\".9 0 0 1\"/>"<<std::endl;

    text<<root->to_xml(nbr_tab);

    text<<tab_string(--nbr_tab)<<"</worldbody>"<<std::endl;
    //End of structure



    //start defining the controls
    text<<tab_string(nbr_tab++)<<"<actuator>"<<std::endl;
    /*
      <position name="actu_base"  class="MPL" joint="base"  ctrlrange="-1.54 1.54"/>
      <position name="actu_second"  class="MPL" joint="second"  ctrlrange="-1.54 1.54"/>
    //*/

    text<<tab_string(--nbr_tab)<<"</actuator>"<<std::endl;

    //end of controls
    text<<tab_string(--nbr_tab)<<"</mujoco>"<<std::endl;

    std::ofstream file;
    file.open("test.xml");
    file<<text.str();
    file.close();

    return text.str();
}
