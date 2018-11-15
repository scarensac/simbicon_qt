#ifndef COLLISION_DETECTION_PROMITIVE_H
#define COLLISION_DETECTION_PROMITIVE_H

#include <Utils/Utils.h>
#include <MathLib/TransformationMatrix.h>

/*========================================================================================================================================================================*
 * This class implements an interface for collision detection primitives such as spheres, capsules and so on.                                                             *
 *========================================================================================================================================================================*/

class RigidBody;
class SphereCDP;
class CapsuleCDP;
class PlaneCDP;
class BoxCDP;

#define UNKNOWN_CDP 0
#define SPHERE_CDP 1
#define CAPSULE_CDP 2
#define PLANE_CDP 3
#define BOX_CDP 4



class CollisionDetectionPrimitive{
protected:
	//keep track of the rigid body that this collision detection primitive belongs to - useful to update world coordinates, etc
    RigidBody* _bdy;
    int _type;

public:
        CollisionDetectionPrimitive(RigidBody* theBody, int type=UNKNOWN_CDP);
	virtual ~CollisionDetectionPrimitive(void);

        virtual CollisionDetectionPrimitive* cpy() =0;

    virtual void update_to_world_primitive() = 0;

	/**
		draw an outline of the primitive...
	*/
    virtual void draw()=0;

	/**
		returns the type of this collision detection primitive.
	*/
    inline int type(){return _type;}

	virtual void compute_finites_elements(){
        throw "this type of collition doesn't have a finite elements decomposition";
	}

    virtual std::string to_xml(int nb_tab);
};

#endif
