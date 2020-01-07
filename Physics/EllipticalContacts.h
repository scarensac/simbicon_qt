#ifndef ELLIPTICALCONTACTS_H
#define ELLIPTICALCONTACTS_H

#include <vector>
#include "MathLib/Point3d.h"
#include "MathLib/Vector3d.h"
#include "MathLib/Quaternion.h"


class Ellipsoid{
public:

    Point3d p;
    Vector3d r;
    Quaternion Q;

    Ellipsoid(float x,float y,float z,float rx,float ry,float rz,float rotx,float roty,float rotz);
    Ellipsoid(){}

    /**
        This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
    */
    Point3d getWorldCoordinates(const Point3d& localPoint);

    /**
        This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
    */
    Point3d getLocalCoordinates(const Point3d& globalPoint);

    /**
        This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
    */
    Vector3d getLocalCoordinates(const Vector3d& globalVector);

    /**
        This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
    */
    Vector3d getWorldCoordinates(const Vector3d& localVector);

    void draw();
};

class ArticulatedRigidBody;

class EllipticalContacts{

    //the static rigidity
    float Kv;
    //the dynamic rigidity
    float Av;

    ArticulatedRigidBody* foot;
    ArticulatedRigidBody* toes;

    std::vector<Ellipsoid> ellipsoids;
    std::vector<ArticulatedRigidBody*> bodies;
    std::vector<float> IntersectionVolume;
    std::vector<Point3d> PressureCenter;
    std::vector<Vector3d> forces_N;
    std::vector<Vector3d> forces_T;

public:
    EllipticalContacts( ArticulatedRigidBody* foot_i);

    void draw();
    void computeForces();

};



#endif
