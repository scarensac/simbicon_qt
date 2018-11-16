#pragma once

//#include "Core\SimGlobals.h"
#include <vector>

/**
	the application point is defined in local coordinates but the F vector is defined in wolrd coordinates
*/

struct ForceStruct
{
	Point3d pt;
	Vector3d F;

	ForceStruct(){
		pt = Point3d(0, 0, 0);
		F = Vector3d(0, 0, 0);
	}

        void toZero(){
            pt=Point3d(0,0,0);
            F=Vector3d(0,0,0);
        }
	
};

struct WaterImpact
{
    std::vector<ForceStruct> forces;
    std::vector<ForceStruct> moments;

    //those two thing are just here to be able to compile
    //they are from the old version
    //as long as they are not removed that means that my water computation are false
    ForceStruct boyancy;
    Vector3d drag_torque;

    WaterImpact(){
    }

    ~WaterImpact(){
        forces.clear();
        moments.clear();
    }

    void init(int num_bodies){
        forces.clear();
        forces.resize(num_bodies);

        moments.clear();
        moments.resize(num_bodies);
    }

    void clear(){
        for (int i=0;i<forces.size();++i){
            forces[i].toZero();
            moments[i].toZero();
        }
    }
};
