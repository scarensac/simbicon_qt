#ifndef ARTICULED_FIGURE_H
#define ARTICULED_FIGURE_H


#include <Physics/rb/ArticulatedRigidBody.h>
#include <Physics/joints/UniversalJoint.h>
#include <Physics/joints/HingeJoint.h>
#include <Physics/joints/BallInSocketJoint.h>

/*======================================================================================================================================================================*
 * An articulated figure is composed of many articulated rigid bodies that are interconnected by joints. Characters, cars, ropes, etc, can all be viewed as articulated *
 * figures. One note is that we will only allow tree structures - no loops.                                                                                             *
 *======================================================================================================================================================================*/
class RBCollection;
class ArticulatedFigure  {
friend class Character;
friend class ODEWorld;
protected:
	//we will keep track of the root of the articulated figure. Based on the outgoing joints we can access its, children, and so on
	ArticulatedRigidBody* root;

	//this is the name of the articulated figure
    std::string _name;
    double _mass;
public:
	/**
		Default constructor
    */
    ArticulatedFigure(void);

    ArticulatedFigure(ArticulatedRigidBody* aroot);

	/**
		Default destructor
	*/
	~ArticulatedFigure(void);



	/**
		This method is used to automatically fix the errors in the joints (i.e. drift errors caused by numercial integration). At some future
		point it can be changed into a proper stabilization technique.
	*/
    void fix_joint_constraints(bool fixOrientations = true, bool fixVelocities = false);

	/**
		This method is used to get all the joints in the articulated figure and add them to the list of joints that is passed in as a paramter.
	*/
    void add_joints_to_list(DynamicArray<Joint*> *joints);

	/**
		This method is used to compute the total mass of the articulated figure.
	*/
    void compute_mass();

	/**
        getters
    */
    double mass(){return _mass;}
    std::string name(){return _name;}

	/**
		This method is used to load the details of an articulated figure from file. The PhysicalWorld parameter points to the world in which the objects
		that need to be linked live in.
	*/
    void load_from_file(FILE* fp, AbstractRBEngine* world);

    std::string to_xml(int nb_tab);
};

#endif
