
#include "Collisiondetectionprimitive.h"
#include <Physics/rb/RigidBody.h>
#include <Utils/Utils.h>


CollisionDetectionPrimitive::CollisionDetectionPrimitive(RigidBody* theBody, int type){
    this->_bdy = theBody;
    _type = type;
}

CollisionDetectionPrimitive::~CollisionDetectionPrimitive(void){
}




#include <sstream>
std::string CollisionDetectionPrimitive::to_xml(int nb_tab)
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
