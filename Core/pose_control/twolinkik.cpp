#include "twolinkik.h"
#include <iostream>

TwoLinkIK::TwoLinkIK(void){
}

TwoLinkIK::~TwoLinkIK(void){
}

/**
All quantities that are passed in as parameters here need to be expressed in the same "global" coordinate frame, unless otherwise noted:
- p1: this is the location of the origin of the parent link
- p2: this is the target location of the end effector on the child link
- n: this is the default normal to the rotation plane (it will get modified as little as possible to account for the target location)

- vParent: vector from parent origin to child origin, expressed in parent coordinates
- nParent: this is the rotation axis, expressed in parent coordinates. The relative orientation between child and parent will be about this axis
- vChild: vector from child origin, to position of the end effector, expressed in child coordinates

Output:
- qP: relative orientation of parent, relative to "global" coordinate frame
- qC: relative orientation of child, relative to the parent coordinate frame


NOTE: for now, the vector vChild is pretty much ignored. Only its length is taken into account. The axis that the child is lined up around is the same as the direction
of the vector from the parent's origin to the child's origin (when there is zero relative orientation between the two, the axis has the same coordinates in both
child and parent frame).
*/
void TwoLinkIK::getIKOrientations(const Point3d& p1, const Point3d& p2, const Vector3d& n, const Vector3d& vParent,
                                  const Vector3d& nParent, const Vector3d& vChild, Quaternion* qP, Quaternion* qC){
	//modify n so that it's perpendicular to the vector (p1, p2), and still as close as possible to n
	Vector3d line = Vector3d(p1, p2);
    line.toUnit();
	Vector3d temp = n.crossProductWith(line);
    temp.toUnit();
    Vector3d nG = line.crossProductWith(temp);
	nG.toUnit();

	//now compute the location of the child origin, in "global" coordinates
	Vector3d solvedJointPosW = TwoLinkIK::solve(p1, p2, nG, vParent.length(), vChild.length());
	Vector3d vParentG = Vector3d(p1, solvedJointPosW);
	Vector3d vChildG = Vector3d(solvedJointPosW, p2);



	//now we need to solve for the orientations of the parent and child
	//if the parent has 0 relative orientation to the grandparent (default), then the grandparent's orientation is also the parent's
	*qP = TwoLinkIK::getParentOrientation(vParentG, nG, vParent, nParent);
    {
        Vector3d t=qP->v;
        if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
            std::cout<<"TwoLinkIK::getIKOrientations nan detected";
        }
    }
    qP->toUnit();

	double childAngle = TwoLinkIK::getChildRotationAngle(vParentG, vChildG, nG);
	*qC = Quaternion::getRotationQuaternion(childAngle, nParent);
    {
        Vector3d t=qC->v;
        if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
            std::cout<<"TwoLinkIK::getIKOrientations nan detected";
        }
    }
    qC->toUnit();
}


Point3d TwoLinkIK::solve(const Point3d &p1, const Point3d &p2, const Vector3d &n, double r1, double r2){
    //the solution for this comes from computation of the intersection of two circles of radii r1 and r2, located at
    //p1 and p2 respectively. There are, of course, two solutions to this problem. The calling application can differentiate between these
    //by passing in n or -n for the plane normal.

    //this is the distance between p1 and p2. If it is > r1+r2, then we have no solutions. To be nice about it,
    //we will set r to r1+r2 - the behaviour will be to reach as much as possible, even though you don't hit the target
    double r = Vector3d(p1, p2).length();
    if (r > (r1 + r2) * 0.993){
        r = (r1 + r2) * 0.993;
//        std::cout<<"TwoLinkIK::solve had to reduce the legnth to find a solution";
    }
    //this is the length of the vector starting at p1 and going to the midpoint between p1 and p2
    //I commented the old formula for 'a' because I relay don't get what they tried  and their calculation don't give the middle for sure
    double a = r / 2;
    //double a = (r1 * r1 - r2 * r2 + r * r) / (2 * r);
    double tmp = r1*r1 - a*a;
    if (tmp < 0)
        tmp = 0;
    //and this is the distance from the midpoint of p1-p2 to the intersection point
    double h = sqrt(tmp);
    //now we need to get the two directions needed to reconstruct the intersection point
    Vector3d d1 = Vector3d(p1, p2).toUnit();
    Vector3d d2 = d1.crossProductWith(n).toUnit();

    //and now get the intersection point
    Point3d p = p1 + d1 * a + d2 * (-h);

    return p;
}


Quaternion TwoLinkIK::getParentOrientation(const Vector3d &vGlobal, const Vector3d &nGlobal, Vector3d vLocal, const Vector3d &nLocal){
    Quaternion q;

    Vector3d result, u = nLocal, v=nGlobal;
    result.x = u.y * v.z - u.z * v.y;
    result.y = u.z * v.x - u.x * v.z;
    result.z = u.x * v.y - u.y * v.x;

    //first off, compute the quaternion that rotates nLocal into nGlobal
    Vector3d axis = nLocal.crossProductWith(nGlobal);
    axis.toUnit();
    double ang = nLocal.angleWith(nGlobal);

    q = Quaternion::getRotationQuaternion(ang, axis);

    //now q rotates nLocal into nGlobal. We now need an aditional orientation that aligns vLocal to vGlobal

    //nLocal is perpendicular to vLocal so q*nLocal is perpendicular to q*vLocal. Also, q*nLocal is equal to nGlobal,
    //so nGlobal is perpendicular to both q*vLocal and vGlobal, which means that this rotation better be about vGlobal!!!
    vLocal = q.rotate(vLocal);
    axis = vLocal.crossProductWith(vGlobal);
    axis.toUnit();
    ang = vLocal.angleWith(vGlobal);

    q = Quaternion::getRotationQuaternion(ang, axis) * q;


    {
        Vector3d t=q.v;
        if ((t.x!=t.x)||(t.y!=t.y)||(t.z!=t.z)){
            std::cout<<"Quaternion getParentOrientation nan detected";
        }
    }

    return q;
}


double TwoLinkIK::getChildRotationAngle(const Vector3d &vParent, const Vector3d &vChild, const Vector3d n){
    //compute the angle between the vectors (p1, p) and (p, p2), and that's our result
    double angle = vParent.angleWith(vChild);
    if (vParent.crossProductWith(vChild).dotProductWith(n)<0)
        angle = -angle;

    return angle;
}
