
#include "planecdp.h"
#include <Physics/cdp/SphereCDP.h>
#include <Physics/cdp/CapsuleCDP.h>
#include <Physics/rb/RigidBody.h>

PlaneCDP::PlaneCDP(RigidBody* theBody, Vector3d n_, Point3d o_) : CollisionDetectionPrimitive(theBody){
	//make sure we have a unit vector;
	p.n = n_;
	p.n.toUnit();
	p.p = o_;

    _type = PLANE_CDP;
}

PlaneCDP::~PlaneCDP(void){
}

CollisionDetectionPrimitive *PlaneCDP::cpy()
{
    return new PlaneCDP(_bdy,p.n,p.p);
}

	
/**
	Draw an outline of the capsule
*/
void PlaneCDP::draw(){
	//we won't draw the plane...
}


void PlaneCDP::update_to_world_primitive(){
//	bdy->state.orientation.fastRotate(p.n, &wP.n);
    wP.n = _bdy->getWorldCoordinates(p.n);
    wP.p = _bdy->getWorldCoordinates(p.p);
}

#include <sstream>
std::string PlaneCDP::to_xml(int nb_tab)
{
    std::ostringstream oss;
    /*
    oss<<tab_string(nb_tab)<<"<joint "<<
         "name=\""<<name()<<"\" "<<
         "type=\"hinge\" "<<
         "name=\""<<base<<"\" "<<
         "name=\""<<base<<"\" "<<
         "name=\""<<base<<"\" "<<


         " pos="0 0 1" axis="1 0 0" range="-1.54 1.54"/>
         */
    return oss.str();
}

