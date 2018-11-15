
#include "spherecdp.h"
#include <GLUtils/GLUtils.h>
#include <Physics/cdp/CapsuleCDP.h>
#include <Physics/cdp/PlaneCDP.h>
#include <Physics/rb/RigidBody.h>

SphereCDP::SphereCDP(RigidBody* theBdy, Point3d& c_, double r_) : CollisionDetectionPrimitive(theBdy){
	s.pos = c_;
	s.radius = r_;
    _type = SPHERE_CDP;
}

SphereCDP::~SphereCDP(void){
}

CollisionDetectionPrimitive *SphereCDP::cpy()
{
    return new SphereCDP(_bdy,s.pos,s.radius);
}


/**
	Draw an outline of the sphere
*/
void SphereCDP::draw(){
	GLUtils::drawSphere(s.pos, s.radius, 5);
}

/**
	updates the world sphere.
*/
void SphereCDP::update_to_world_primitive(){
    wS.pos = _bdy->getWorldCoordinates(s.pos);
	wS.radius = s.radius;
}

#include <sstream>
std::string SphereCDP::to_xml(int nb_tab)
{
    // <geom type="box" group="1" "0.02 0.02 0.02" contype="0" conaffinity="0" rgba=".9 .5 .5 1"/>
     std::ostringstream oss;
     oss<<tab_string(nb_tab)<<"<geom type=\"sphere\" "<<
          "size=\""<<s.radius<<"\" "<<
          "/>"<<std::endl;
     return oss.str();
}
