
#include "capsulecdp.h"
#include <GLUtils/GLUtils.h>
#include <Physics/cdp/SphereCDP.h>
#include <Physics/cdp/PlaneCDP.h>
#include <Physics/rb/RigidBody.h>

CapsuleCDP::CapsuleCDP(RigidBody* theBody, Point3d& a_, Point3d& b_, double r_) : CollisionDetectionPrimitive(theBody){
	this->c.p1 = a_;
	this->c.p2 = b_;
	this->c.radius = r_;
    _type = CAPSULE_CDP;
}

CapsuleCDP::~CapsuleCDP(void){
}

CollisionDetectionPrimitive *CapsuleCDP::cpy()
{
    return new CapsuleCDP(_bdy,c.p1,c.p2,c.radius);
}


/**
	draw an outline of the capsule
*/

void CapsuleCDP::draw(){
	GLUtils::drawCylinder(this->c.radius, Vector3d(this->c.p1, this->c.p2), this->c.p1, 6);
	GLUtils::drawSphere(this->c.p1, this->c.radius, 5);
	GLUtils::drawSphere(this->c.p2, this->c.radius, 5);
}

void CapsuleCDP::update_to_world_primitive(){
    wC.p1 = _bdy->getWorldCoordinates(c.p1);
    wC.p2 = _bdy->getWorldCoordinates(c.p2);
	wC.radius = c.radius;
}

#include <sstream>
std::string CapsuleCDP::to_xml(int nb_tab)
{
    std::ostringstream oss;

    oss<<tab_string(nb_tab)<<"<geom type=\"capsule\" "<<
         "fromto=\""<<-c.p1.x<<" "<<c.p1.z<<" "<<c.p1.y<<" "<<-c.p2.x<<" "<<c.p2.z<<" "<<c.p2.y<<"\" "<<
         "size=\""<<c.radius<<"\" "<<
         "/>"<<std::endl;
    return oss.str();
}
