#pragma once

//#include "Core\SimGlobals.h"
#include <vector>
#include "MathLib/Vector3d.h"
#include <sstream>
#include <map>

/**
        the application point is defined in local coordinates but the F vector is defined in wolrd coordinates
*/

struct ForceStruct
{
    Point3d pt;
    Vector3d F;
    Vector3d M;

    ForceStruct(){
        toZero();
    }

    ForceStruct(Point3d point){
        toZero();

        pt=point;
    }

    void toZero(){
        pt=Point3d(0,0,0);
        F=Vector3d(0,0,0);
        M = Vector3d(0, 0, 0);
    }

    bool isZero() const{
        return F.isZeroVector()&&M.isZeroVector();
    }

    void modifyApplicationPoint(Point3d p){
        if (pt==p){
            return;
        }

        //so I need to do 2 things
        // 1 :translate the force (and calculate the moment created)
        // 2 :translate the moment (this most likely generate a force I guess)

        Vector3d F_result=Vector3d(0,0,0);
        Vector3d M_result=Vector3d(0,0,0);

        if (!F.isZeroVector()){
            M_result+=Vector3d(pt-p).crossProductWith(F);
            F_result+=F;
        }

        if (!M.isZeroVector()){
            throw("ForceStruct::modifyApplicationPoint modying the application point of a force that has an associated moment is not possible");
        }

        pt=p;
        F=F_result;
        M=M_result;
    }

    inline ForceStruct& operator+= (const ForceStruct& fs) {
        if (fs.isZero()){
            return *this;
        }

        if (isZero()){
            pt=fs.pt;
            F=fs.F;
            M=fs.M;
        }else{
            //if the two don't have the same application point I need to send a crash for now
            if (pt!=fs.pt){
                throw("ForceStruct operator+= trying to add two forces structs that don't have the same application point");
            }else{
                //if they have the same point I cna simply add them
                F+=fs.F;
                M+=fs.M;
            }
        }

        return *this;
    }

    inline friend ForceStruct operator+ (const ForceStruct& fs, const ForceStruct& fs2) {
        ForceStruct result;

        result=fs2;
        result+=fs;

        return result;
    }

    void check_for_undefined(){
        if ((pt.x!=pt.x)||(pt.y!=pt.y)||(pt.z!=pt.z)){
            throw("ForceStruct  undefinned pt  ");
        }

        if ((F.x!=F.x)||(F.y!=F.y)||(F.z!=F.z)){
            throw("ForceStruct  undefinned F  ");
        }

        if ((M.x!=M.x)||(M.y!=M.y)||(M.z!=M.z)){
            throw("ForceStruct  undefinned M  ");
        }
    }




};

struct WaterImpact
{
protected:
    std::vector<ForceStruct> impact_boyancy;
    std::vector<ForceStruct> impact_drag;
    std::map<int,int> bodies_id_map;
    std::vector<int> bodies_id;

public:
    WaterImpact(){
    }

    ~WaterImpact(){
        impact_boyancy.clear();
        impact_drag.clear();
    }

    int getNumBodies(){return static_cast<int>(bodies_id.size());}

    ForceStruct getDrag(int body_id){
        return impact_drag[bodies_id_map[body_id]];
    }

    ForceStruct getDragInOrder(int i){
        return impact_drag[i];
    }

    void setDrag(int body_id, ForceStruct& drag){
        impact_drag[bodies_id_map[body_id]]=drag;
    }


    ForceStruct getBoyancy(int body_id){
        return impact_boyancy[bodies_id_map[body_id]];
    }

    ForceStruct getBoyancyInOrder(int i){
        return impact_boyancy[i];
    }

    void setBoyancy(int body_id, ForceStruct& boyancy){
        impact_boyancy[bodies_id_map[body_id]]=boyancy;
    }


    void init(int num_bodies, std::vector<int> bodies_id_i){
        if(num_bodies!=bodies_id_i.size()){
            throw(std::string("WaterImpact::init the num body does not corespond to the number of ids"));
        }

        bodies_id=bodies_id_i;
        for (int i=0;i<num_bodies;++i){
            bodies_id_map[bodies_id_i[i]]=i;
        }


        impact_boyancy.clear();
        impact_boyancy.resize(num_bodies);

        impact_drag.clear();
        impact_drag.resize(num_bodies);

        clear();
    }

    void clear(){
        for (int i=0;i<impact_boyancy.size();++i){
            impact_boyancy[i].toZero();
            impact_drag[i].toZero();
        }
    }

    void check_for_undefined(){

        for (int i=0;i<impact_boyancy.size();++i){
            try{
                impact_boyancy[i].check_for_undefined();
            }catch(const char* c){
                std::string msg(c);
                std::ostringstream oss;
                oss<<msg<<"  //  "<<" boyancy at id: "<<i;
                throw(oss.str().c_str());
            }

            try{
                impact_drag[i].check_for_undefined();
            }catch(const char* c){
                std::string msg(c);
                std::ostringstream oss;
                oss<<msg<<"  //  "<<" drag at id: "<<i;
                throw(oss.str().c_str());
            }

        }
    }
};
