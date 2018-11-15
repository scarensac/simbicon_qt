#ifndef SPHERE_CDP_H
#define SPHERE_CDP_H


#include <Physics/cdp/CollisionDetectionPrimitive.h>
#include <MathLib/Point3d.h>
#include <MathLib/Sphere.h>
#include <MathLib/TransformationMatrix.h>
#include <Utils/Utils.h>

/*========================================================================================================================================================================*
 * This class implements a sphere class that will be used as a collision detection primitive.                                                                             *
 * A sphere is represented by the position of the center and the radius length. We will also store a temp position for the world coordinates of the center of the sphere. *
 * This will be used when evaluating the contact points with other primitives, and it will be automatically 
 *========================================================================================================================================================================*/
class SphereCDP : public CollisionDetectionPrimitive{
public:
	//keep track of the local-coordinates sphere used by this collision detection primitive
	Sphere s;
	//and this is the sphere, expressed in world coordinates
	Sphere wS;
public:
	SphereCDP(RigidBody* theBdy, Point3d& c_, double r_);
	virtual ~SphereCDP(void);

    virtual CollisionDetectionPrimitive* cpy();

    virtual void update_to_world_primitive();

	/**
		Draw an outline of the sphere
	*/
	virtual void draw();

	/**
		return the radius of the sphere
	*/
	inline double getRadius(){
		return s.radius;
	}

	/**
		return the center of the sphere, expressed in local coordinates
	*/
	inline Point3d getCenter(){
		return s.pos;
	}

        std::string to_xml(int nb_tab);
};

#endif
