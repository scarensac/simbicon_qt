#ifndef PLANE_CDP_H
#define PLANE_CDP_H


#include <MathLib/Point3d.h>
#include <MathLib/Vector3d.h>
#include <Utils/Utils.h>
#include <MathLib/Plane.h>
#include <Physics/cdp/CollisionDetectionPrimitive.h>


/*========================================================================================================================================================================*
 * This class implements a plane class that will be used as a collision detection primitive.                                                                              *
 * A plane is represented by the position of one point on the plane, as well as the normal (unit vector) to the plane. We will store these two quantities both in local   *
 * coordinates, and in world coordinates which will be used for the collision detection. NOTE: we will not be evaluating the collision between different planes because   *
 * I will assume that they are all part of fixed objects only.
 *========================================================================================================================================================================*/
class PlaneCDP : public CollisionDetectionPrimitive{
private:
	//this is the plane, expressed in the local coordinates of the rigid body that owns it
	Plane p;
	//and this is the plane expressed in world coordinates
	Plane wP;
	
public:
	PlaneCDP(RigidBody* theBody, Vector3d n_, Point3d o_);
	~PlaneCDP(void);

    virtual CollisionDetectionPrimitive* cpy();

    virtual void update_to_world_primitive();

	virtual void draw();
    Vector3d normal(){return p.n;}
    Point3d point(){return p.p;}

    std::string to_xml(int nb_tab);
};

#endif
