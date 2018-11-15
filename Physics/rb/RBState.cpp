#include "rbstate.h"

/**
	Default constructor - populate the data members using safe values..
*/
RBState::RBState(void){
	//guess some default values if this constructor is used...
	this->position = Point3d(0,0,0);
    this->orientation = Quaternion(1,Vector3d(0,0,0));
    this->velocity = Vector3d(0,0,0);
    this->angular_velocity = Vector3d(0,0,0);
    this->acc = Vector3d(0,0,0);
    this->angular_acc = Vector3d(0,0,0);
}

/**
	A copy constructor.
*/
RBState::RBState(const RBState& other){
	this->position = other.position;
    this->orientation = other.orientation;
    this->velocity = other.velocity;
    this->angular_velocity = other.angular_velocity;
    this->acc = other.acc;
    this->angular_acc = other.angular_acc;
}

/**
	and a copy operator	
*/
RBState& RBState::operator = (const RBState& other){
	this->position = other.position;
	this->orientation = other.orientation;
	this->velocity = other.velocity;
    this->angular_velocity = other.angular_velocity;
    this->acc = other.acc;
    this->angular_acc = other.angular_acc;
	return *this;
}

RBState::~RBState(void){
		
}

