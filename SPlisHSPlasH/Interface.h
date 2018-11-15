#ifndef INTERFACE_H
#define INTERFACE_H

#include <vector>
#include "SPlisHSPlasH/DFSPH/DFSPH_c_arrays_structure.h"
#include "MathLib/Point3d.h"
#include "MathLib/Quaternion.h"

class Interface{
public:


    //this file contains function to be able to communicate between the character simulation and the fluid simulation
    //the main problem is the I have quaternion and vector classes existing in the two project that are not the same
    //to do so I'll use a pure c_array_type data

    class ArrayForceInfo{
    public:
        double position[3];
        double force[3];
    };

    //must be called at the start

#ifdef FLUID_COMPIL

    static void initFluid(double timeStep);
    static void forceUpdateDynamicBodies();
    static void updateDynamicBodies(const std::vector<SPH::DynamicBody> &vect_new_info);
    static void getFluidImpactOnDynamicBodies(std::vector<Vector3d>& forces, std::vector<Vector3d>& moments);
    static void fluidSimulationStep();
    static void updateTimeStepDuration(double duration);
    static void handleDynamicBodiesPause(bool pause);
    static void zeroFluidVelocities();
    static void moveFluidSimulation(Point3d target_Position);
    static void drawParticles(bool drawFluid, bool drawBodies, bool drawBoundaries);
#else
    static inline void initFluid(double timeStep){}
    static inline void forceUpdateDynamicBodies(){}
    static inline void updateDynamicBodies(const std::vector<SPH::DynamicBody> &vect_new_info){}
    static inline void getFluidImpactOnDynamicBodies(std::vector<Vector3d>& forces, std::vector<Vector3d>& moments){}
    static inline void fluidSimulationStep(){}
    static inline void updateTimeStepDuration(double duration){}
    static inline void handleDynamicBodiesPause(bool pause){}
    static inline void zeroFluidVelocities(){}
    static inline void moveFluidSimulation(Point3d target_Position){}
    static inline void drawParticles(bool drawFluid, bool drawBodies, bool drawBoundaries){}

#endif

    //note I use the point3d instead of vector3d since mathlib is cappable of converting a point to a vector
    //but I don't think it handles the oposite
    static SPH::Vector3d vector3dToSPH3d(const Point3d& v) { return SPH::Vector3d(v.x, v.y, v.z); }
    static Point3d vectorSPH3dTo3d(const SPH::Vector3d& v) { return Point3d(v.x, v.y, v.z);}
    static SPH::Quaternion quaternionToSPHQuaternion(const Quaternion& q){ return SPH::Quaternion(q.v.x, q.v.y, q.v.z, q.s);}

};

#endif
