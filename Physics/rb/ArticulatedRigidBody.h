#ifndef ARTICULED_RIGID_BODY_H
#define ARTICULED_RIGID_BODY_H


#include <Physics/rb/RigidBody.h>

/*=======================================================================================================================================================================*
 * We will treat the articulated rigid bodies as normal rigid bodies that are connected by joints. The joints are needed to enforce constraints between the articulated  *
 * rigid bodies, but other than that, the dynamics are the same as for rigid bodies. We will assume that every articulated figure will be loop-free (tree hierarchies).  *
 *=======================================================================================================================================================================*/
class Joint;
class ArticulatedRigidBody : public RigidBody{
friend class Joint;
friend class ArticulatedFigure;
friend class SimBiController;
protected:
	//this is the parent joint.
    Joint* _parent_joint;
	//and these are the child joints - it can have as many as it wants.
    DynamicArray<Joint*> _child_joints;
	//this is the articulated figure that the rigid body belongs to
    ArticulatedFigure* _af;
public:
    /**
        Default constructor
    */
    ArticulatedRigidBody();

    /**
        constructor from file
    */
    ArticulatedRigidBody(FILE* f);


    /**
     * @brief return a copy of the object. If keep_graphics==false then mesh are not copied
     */
    virtual ArticulatedRigidBody* cpy(bool keep_graphics=false);


	/**
		This method draws the current rigid body.
	*/
	virtual void draw(int flags);

	/**
		Default destructor
	*/
	virtual ~ArticulatedRigidBody(void);

	/**
	returns the parent joint for the current articulated body
	*/
    inline Joint* parent_joint(){ return _parent_joint; }

	/**
	returns the child joints for the current articulated body
	*/
    DynamicArray<Joint*> child_joints(){ return _child_joints; }

	/**
		this method always returns true
	*/
    virtual bool is_articulated(){return true;}


    virtual ArticulatedFigure* af(){return _af;}
    virtual void af(ArticulatedFigure* aaf){if (_af==NULL){_af=aaf;}}

    virtual void add_link(Joint* j, ArticulatedRigidBody* child);

    std::string to_xml(int nb_tab);

};

#endif
