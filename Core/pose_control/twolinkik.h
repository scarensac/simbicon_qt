#ifndef TWO_LINK_IK_H
#define TWO_LINK_IK_H

#include <MathLib/Point3d.h>
#include <MathLib/Vector3d.h>
#include <MathLib/Quaternion.h>


/**
This class implements an analytical solution for the inverse kinematics of a linkage composed of two links - a parent link and a child link.
The origin of the parent link is fixed, and the desired position for the end effector of the child is given as input. The position of the origin of
the child link is computed, and, with this information, the orientation of the parent and child links are also computed. This can be seen, generally speaking,
as the solution of the intersection of two spheres, which gives an infinte number of solutions. To reduce this, a vector that represents the normal to the plane
that the three points (parent origin, child origin and child end effector) should lie on is given as input. The child can only rotate relative to the parent
around this normal axis, which can be used as a prior. This results in two possible solutions. The direction of the normal is used to select a unique solution.
*/
class TwoLinkIK {
	TwoLinkIK(void);
	~TwoLinkIK(void);
public:
	/**
	This method determines the position of the joint between the child and parent links.
	Input (all quantities need to be measured in the same coordinate frame):
	p1 - location of parent's origin (for example the shoulder for an arm)
	p2 - target location of end effector of child link (for example, the wrist location)
	n - normal to the plane in which the relative motion between parent and child will take place
	r1 - the length of the parent link
	r2 - the length of the child link
	Return:
	p - the position of the joint (the elbow, for instance), measured in the same coordinates as the values passed in here
	*/
    static inline Point3d solve(const Point3d& p1, const Point3d& p2, const Vector3d& n, double r1, double r2);

	/**
	This method determines the orientation for the parent link, relative to some other coordinate ("global") frame.
	Two vectors (one that goes from the parent origin to the child origin v, as well as a normal vector n) are known,
	expressed both in the global frame and the parent frame. Using this information, we can figure out the relative orientation
	between the parent frame and the global frame.

	Input:
	vGlobal - parent's v expressed in grandparent coordinates
	nGlobal	- this is the rotation axis that is used for the relative rotation between the child and the parent joint, expressed in
	grandparent coordinates
	vLocal  - parent's v expressed in the parent's local coordinates
	nLocal  - this is the rotation axis that is used for the relative rotation between the child and the parent joint, expressed in
	parent's local coordinates
	Output:
	q		- the relative orientation between the parent and the grandparent (i.e. transforms vectors from parent coordinates to grandparent coordinates).
	*/
    static inline Quaternion getParentOrientation(const Vector3d& vGlobal, const Vector3d& nGlobal, Vector3d vLocal, const Vector3d& nLocal);

	/**
	This method determines the rotation angle about the axis n for the child link, relative to the orientation of the parent
	Input (all quantities need to be measured in the same coordinate frame):
	(Note: v is the vector from a link's origin to the end effector or the child link's origin).
	vParent - the v qunatity for the parent
	vChild  - the v quntity for the child (vector between the child origin and the end effector).
	n		- the axis of rotation
	Output:
	q		- the relative orientation between the child and parent frames
	*/
    static inline double getChildRotationAngle(const Vector3d& vParent, const Vector3d& vChild, const Vector3d n);


	static void getIKOrientations(const Point3d& p1, const Point3d& p2, const Vector3d& n, const Vector3d& vParent, const Vector3d& nParent, const Vector3d& vChild, Quaternion* qP, Quaternion* qC);


};


#endif;
