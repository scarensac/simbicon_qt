#ifndef JOINT_H
#define JOINT_H

#include <string>
#include <MathLib/Vector3d.h>
#include <MathLib/Quaternion.h>

#include "Physics/rb/ArticulatedRigidBody.h"

class AbstractRBEngine;
/**
    this structure contains the Joint members that could differs depending on the controller used
    It also contains every memebr that are modified during the simulation
*/
class ControlParams{
public:
    //this variable is set to true if the current joint is being actively controlled, false otherwise
    bool controlled;
    //these two variables are the proporitonal and derivative gains for the PD controller used to compute the torque
    double kp, kd;
    //this is the maximum absolute value torque allowed at this joint
    double max_abs_torque;
    //the torques about the about the x, y and z axis will be scaled differently to account for the potentially different principal moments of inertia
    //of the child
    Vector3d scale;



    //the torque applied to this joint. It should be set/reset by a controller acting on this joint.
    Vector3d torque;

    double strength;
    //this variable, if true, indicates that the desired orientation is specified in the character coordinate frame
    bool relative_to_char_frame;
    //and this is the coordinate frame that the desired orientation is specified in, if relative_to_char_frame is true
    Quaternion char_frame;

    /**
        This constructor initializes the variables to some safe, default values
    */
    ControlParams(){
        controlled = false;
        kp = kd = 0;
        max_abs_torque = 0;
        scale = Vector3d();
        strength = 1;
        relative_to_char_frame = false;
        torque= Vector3d();
    }
};




#define STIFF_JOINT 1
#define HINGE_JOINT 2
#define BALL_IN_SOCKET_JOINT 3
#define UNIVERSAL_JOINT 4

/*=======================================================================================================================================================================*
 * This class is responsible with the implementation of the methods that are neccessary to implement joints in an articulated system. The joints impose constraints on   *
 * the articulated rigid bodies that they connect. Each joint will be used to link a parent body to a child body. The joints that will be considered, for now at least,  *
 * are all rotationals joints with 1, 2 or 3 degrees of freedom. The default type of joint is a Ball in Socket joint with no joint limits.                               *
 *=======================================================================================================================================================================*/
class Joint{

public:
    ControlParams control_params;

protected:
	//this is the parent link
    ArticulatedRigidBody* _parent;
	//this is the location of the joint on the parent body - expressed in the parent's local coordinates
    Point3d _parent_pos;
	//this is the child link
    ArticulatedRigidBody* _child;
	//this is the location of the joint on the child body - expressed in the child's local coordinates 
	//NOTE: the locations of the parent and child joint locations must overlap in world coordinates
    Point3d _child_pos;
	//this variable is used to indicate if this joint has joint limits or not (the details regarding the limits are specified on a per joint type basis)
    bool _use_joint_limits;
	//this is the name of the joint
    std::string _name;
	//This is the index of the joint (so i search them easily whitout wasting time comparing strings)
    uint _idx;


	/**
		This method is used to fix the joint angular constraint to correct for drift. This is done by changing
		the orientation of the child relative to the parent
	*/
    virtual void fix_angular_constraint(const Quaternion& qRel) = 0;

	/**
		This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
		been read from an input file.
	*/
    virtual void read_axes(const char* axes)=0;

	/**
		This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
		have been read from an input file.
	*/
    virtual void read_joint_limits(const char* limits)=0;

    /**
     * @brief only used in constructor (set the af pointers of the parent and chaild arb
     */
    void set_articuled_figure(ArticulatedFigure *figure);

public:
	/**
		Default constructor
	*/
    Joint();

    Joint(Joint* j){
        control_params=j->control_params;
        _parent=j->_parent;
        _parent_pos=j->_parent_pos;
        _child=j->_child;
        _child_pos=j->_child_pos;
        _use_joint_limits=j->_use_joint_limits;
        _name=j->_name;
        _idx=j->_idx;
    }

	/**
		Default destructor
	*/
	virtual ~Joint(void);

    /**
        This method is used to compute the relative orientation between the parent and the child rigid bodies, expressed in
        the frame coordinate of the parent.
    */
    void compute_relative_orientation(Quaternion& qRel);

    /**
        This method is used to compute the relative orientation between the parent and the child rigid bodies, expressed in
        the frame coordinate of the parent.
        This mehod add the angular veloties to calculate an estimation of the furure timestep
    */
    void compute_relative_orientation_Future(Quaternion& qRel);

    /**
        getter /setter
    */
    uint idx(){ return _idx; }
    void idx(int id){_idx=id;}
    bool controlled(){return control_params.controlled;}
    void controlled(bool is_controlled){control_params.controlled=is_controlled;}

	/**
		This method is used to automatically fix the errors in the joints (i.e. drift errors caused by numercial integration). At some future
		point it can be changed into a proper stabilization technique.
	*/
    void fix_joint_constraints(bool fixOrientations, bool fixVelocities, bool recursive);

	/**
		This method is used to load the details of a joint from file. The PhysicalWorld parameter points to the world in which the objects
		that need to be linked live in.
	*/
    void load_from_file(FILE* fp, AbstractRBEngine* world);

	/**
		Returns the type of the current joint
	*/
    virtual int get_joint_type() = 0;

    void set_parent_child(ArticulatedRigidBody* p,ArticulatedRigidBody* c){
        _parent=p;
        _child=c;
    }

	/**
        getters
	*/
    inline ArticulatedRigidBody* parent(){return _parent;}
    inline ArticulatedRigidBody* child(){return _child;}
    inline Point3d child_joint_position(){return _child_pos;}
    inline Point3d parent_joint_position(){return _parent_pos;}
    inline std::string name() { return _name; }
    inline bool use_joint_limits(){return _use_joint_limits;}
    /**



    /**
        sets the torque
    */
    inline void torque(const Vector3d& t){control_params.torque = t;}
    inline Vector3d torque(){return control_params.torque;}


    virtual std::string to_xml(int nb_tab);
};

#endif
