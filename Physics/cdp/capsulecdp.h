#ifndef CAPSULE_CDP_H
#define CAPSULE_CDP_H


#include <Physics/cdp/CollisionDetectionPrimitive.h>
#include <MathLib/Point3d.h>
#include <MathLib/TransformationMatrix.h>
#include <Utils/Utils.h>
#include <MathLib/Capsule.h>

/*========================================================================================================================================================================*
 * This class implements a capsule class that will be used as a collision detection primitive.                                                                            *
 * A capsule is represented by the position of the two end points and the radius length. We will also store a temp position for the world coordinates of the endppoints   * 
 * of the capsule. This will be used when evaluating the contact points with other primitives, and it needs to be updated any time the world position of the object that  *                      
 * owns this capsule changes.                                                                                                                                             *
 *========================================================================================================================================================================*/
class CapsuleCDP : public CollisionDetectionPrimitive{
private:
	//a capsule is really just an infinite number of spheres that have the center along a segment. Therefore, to define the capsule we need the
	//two end points and the radius
	Capsule c;
	Capsule wC;
public:
	CapsuleCDP(RigidBody* theBody, Point3d& a_, Point3d& b_, double r_);
	~CapsuleCDP(void);

    virtual CollisionDetectionPrimitive* cpy();

    virtual void update_to_world_primitive();

	/**
		Draw an outline of the capsule
	*/
	virtual void draw();

    /**
     @brief getters
    */
    inline double radius(){return this->c.radius;}
    inline Point3d getA(){return c.p1;}
    inline Point3d getB(){return c.p2;}

    std::string to_xml(int nb_tab);
};

#endif
