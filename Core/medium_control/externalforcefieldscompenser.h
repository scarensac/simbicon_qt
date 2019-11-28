#ifndef EXTERNALFORCEFIELDSCOMPENSER_H
#define EXTERNALFORCEFIELDSCOMPENSER_H

#include <map>
#include <MathLib/Point3d.h>
#include <MathLib/Vector3d.h>
#include "Core/ForcesUtilitary.h"


class Character;
class Joint;

class ExternalForceFieldsCompenser
{
protected:
    Character* character;

    ///Members for results
    std::vector<Vector3d> _torques;
    std::vector<Vector3d> _torques2;

    std::vector<Vector3d> _torques3_gravity;
    std::vector<Vector3d> _torques3_fluid;

    std::vector<Vector3d> _torques4_gravity;
    std::vector<Vector3d> _torques4_fluid;

    ///Internal members
    std::vector<int> stance_leg_idxs;

public:
    ExternalForceFieldsCompenser(Character *c);
    virtual ~ExternalForceFieldsCompenser();

    std::vector<Vector3d>& torques(){return _torques;}
    std::vector<Vector3d>& torques2(){return _torques2;}
    std::vector<Vector3d>& torques3_gravity(){return _torques3_gravity;}
    std::vector<Vector3d>& torques3_fluid(){return _torques3_fluid;}
    std::vector<Vector3d>& torques4_gravity(){return _torques4_gravity;}
    std::vector<Vector3d>& torques4_fluid(){return _torques4_fluid;}

    /**
    This method computes the torques that cancel out the effects of gravity,
    for better tracking purposes
    */
    void compute_compensation(WaterImpact& resulting_impact);

    /**
    this version uses an addition of the forces before doing the compensation
    but contains an amélioration by using a leaf rb consideration
    */
    void compute_compensation_v2(WaterImpact &resulting_impact);

    /**
    This version treat the gravity and the fluid separately
    uses the same leaf consideration as v2
    this version uses the sum of the force and propagate it. it only works if all the forces are in the same direction
    */
    void compute_compensation_v3(std::vector<ForceStruct> &force_field, std::vector<Vector3d> &result_ptr);

    /**
    This version treat the gravity and the fluid separately
    same as v3 but do propagate each force separately
    also has an option to have the ankle compensate the foot force for the support leg(s)
    */
    void compute_compensation_v4(std::vector<ForceStruct> &force_field, std::vector<Vector3d> &result_ptr, bool ankle_compensate_foot=false,
                                 bool support_foot_as_root=true);


    void preprocess_simulation_step();

    void simulation_step();

    /**
     * @brief compute_fluid_impact_compensation
     * @param type 0 only boyancy, 1 only drag
     */
    void compute_fluid_impact_compensation(WaterImpact &resulting_impact, int type=0);


protected:
    /**
        This method is used to compute the torques that mimick the effect of applying a force on
        a rigid body, at some point. It works best if the end joint is connected to something that
        is grounded, otherwise (I think) this is just an approximation.

        This function works by making use of the formula:

        t = J' * f, where J' is dp/dq, where p is the position where the force is applied, q is
        'sorta' the relative orientation between links. It makes the connection between the velocity
        of the point p and the relative angular velocities at each joint. Here's an example of how to compute it.

        Assume: p = pBase + R1 * v1 + R2 * v2, where R1 is the matrix from link 1 to whatever pBase is specified in,
        and R2 is the rotation matrix from link 2 to whatever pBase is specified in, v1 is the point from link 1's
        origin to link 2's origin (in link 1 coordinates), and v2 is the vector from origin of link 2 to p
        (in link 2 coordinates).

        dp/dt = d(R1 * v1)/dt + d(R2 * v2)/dt = d R1/dt * v1 + d R2/dt * v2, and dR/dt = wx * R, where wx is
        the cross product matrix associated with the angular velocity w
        so dp/dt = w1x * R1 * v1 + w2x * R2 * v2, and w2 = w1 + wRel

        = [-(R1*v1 + R2*v2)x   -(R2*v2)x ] [w1   wRel]', so the first matrix is the Jacobian.

        for a larger chain, we get:
        dp/dt = [-(R1*v1 + R2*v2 + ... + Rn*vn)x; -(R2*v2 + R3*v3 + ... + Rn*vn)x; ...; -(Rn*vn)x ] [w1; w2; ...; wn]'
        or
        dp/dt = [-p1x; -p2x; ...; -pnx ] [w1; w2; ...; wn]'
        where pi is the vector from the location of the ith joint to p;

        and all omegas are relative to the parent. The ith cross product vector in the jacobian is a vector from the
        location of the ith joint to the world location of the point where the force is applied.

        The first entry is the cross product matrix of the vector (in pBase coordinates) from the
        origin of link 1 to p, and the second entry is the vector (in pBase coordinates) from
        the origin of link 2 to p (and therein lies the general way of writing this).
        */
    void compute_joint_torques_equivalent_to_force(Joint* start, const Point3d& pLocal,
                                               const Vector3d& fGlobal, Joint* end);

    /**
        Similar to the function above but only do the computation for a single joint and return the result
        also directly require the global force ...
    */
    Vector3d compute_joint_torques_equivalent_to_force(Joint *joint, const Point3d& pGlobal, const Vector3d& fGlobal);

};

#endif // EXTERNALFORCEFIELDSCOMPENSER_H
