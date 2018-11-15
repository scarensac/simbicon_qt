#include "softbodypointmass.h"

#include <sstream>
#include <iostream>

#include "Physics/cdp/BoxCDP.h"
#include "Physics/rb/RigidBody.h"
#include "Physics/ODEWorld.h"
#include "Core/SimGlobals.h"


SoftBodyPointMass::SoftBodyPointMass(RigidBody *rb_i, ODEWorld *world_i, double km_i, double fx_p_i, double fv_i, double fe_i)
{
    world=world_i;
    rb=rb_i;
    rb_ground=world->getRBByName("ground");


    body_id_rb=world->get_body_id(rb);
    geom_id_rb=dBodyGetFirstGeom(body_id_rb);

    geom_id_ground=world->ground_geom_id;
    body_id_ground = dGeomGetBody(geom_id_ground);

    km=km_i;
    fe=fe_i;
    fv=fv_i;
    fx_p=fx_p_i;
}

void SoftBodyPointMass::init(int nb_subdivision)
{
    BoxCDP* box=static_cast<BoxCDP*>(rb->get_cdps().front());

    discrete_pts_local=box->generate_subdivision(nb_subdivision);
    discrete_pts.resize(discrete_pts_local.size(),Vector3d(0,0,0));
    discrete_pts_world.resize(discrete_pts.size(),Vector3d(0,0,0));
    for (int i=0; i<discrete_pts.size();++i){
        Point3d corner=discrete_pts_local[i];
        discrete_pts[i]=rb->getWorldCoordinates(corner);
        discrete_pts_world[i]=discrete_pts[i];

        std::ostringstream oss;
        oss<< corner.x<<" // "<< corner.y<<" // "<< corner.z;
        std::cout<<oss.str();
    }

    discrete_pts_neighbours=box->generate_subdivision_neighbours(nb_subdivision);

    for (int i=0; i<discrete_pts.size();++i){
        std::vector<int> neighbours=discrete_pts_neighbours[i];

        std::ostringstream oss;
        oss<<i<<" : ";
        for (int j=0;j<(int)neighbours.size();++j){
            oss<< neighbours[j]<<"   ";
        }
        std::cout<<oss.str();
    }

    //store the neighbours id
    discrete_pts_neighbours=box->generate_subdivision_neighbours(nb_subdivision);

    //store the velocity
    discrete_pts_vel_world.resize(discrete_pts.size(),Vector3d(0,0,0));
    discrete_pts_vel.resize(discrete_pts.size(),Vector3d(0,0,0));
    for (int i=0; i<discrete_pts.size();++i){
        discrete_pts_vel[i]=rb->getAbsoluteVelocityForGlobalPoint(discrete_pts[i]);
        discrete_pts_vel_world[i]=discrete_pts_vel[i];
    }

    std::ostringstream oss;
    oss<<"init done: "<<rb->name();
    std::cout<<oss.str();

    //update the target positions
    update_target_position();
}

void SoftBodyPointMass::update_target_position()
{
    for (int i=0; i<discrete_pts.size();++i){
        discrete_pts[i]=rb->getWorldCoordinates(discrete_pts_local[i]);
        discrete_pts_vel[i]=rb->getAbsoluteVelocityForGlobalPoint(discrete_pts[i]);
    }
}

void SoftBodyPointMass::generate_contacts()
{
    BoxCDP* box=static_cast<BoxCDP*>(rb->get_cdps().front());

    //detect_collision with ground
    int count_contacts=0;

    //std::cout<<"start collision detection";
    for (int i=0; i<discrete_pts_world.size();++i){
        std::ostringstream oss;
        oss<<discrete_pts_world[i].y;
        if (discrete_pts_world[i].y<=0)
        {
            oss<<" // collision detectected";
            count_contacts++;
        }
        //std::cout<<oss.str();
    }
    //std::cout<<"collisiosn finished";

    if (count_contacts==0){
        return;
    }

    RigidBody* rb1=rb;
    RigidBody* rb2=rb_ground;
    dContact *cps= new dContact[count_contacts];
    int i=0;

    for (int j=0; j<discrete_pts_world.size();++j){
        int count=0;
        if (discrete_pts_world[j].y<=0)
        {
            cps[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 ;//| dContactBounce;
            cps[i].surface.mu = std::min(rb->getFrictionCoefficient(),rb2->getFrictionCoefficient());
            double groundSoftness = 0, groundPenalty = 0;
            groundSoftness = rb->props.groundSoftness;
            groundPenalty = rb->props.groundPenalty;
//            cps[i].surface.soft_cfm = 0.0005;
//            cps[i].surface.soft_erp = 0;
            cps[i].surface.soft_cfm = groundSoftness;
            cps[i].surface.soft_erp = groundPenalty;

            Point3d pt_col=discrete_pts_world[j];
            //                pt_col.y=-0.0001;
            cps[i].geom.side1=-1;
            cps[i].geom.side2=-1;
            cps[i].geom.depth=-pt_col.y;
            cps[i].geom.g1=geom_id_rb;
            cps[i].geom.g2=geom_id_ground;
            cps[i].geom.normal[0]=0;
            cps[i].geom.normal[1]=1;
            cps[i].geom.normal[2]=0;
            cps[i].geom.pos[0]=pt_col.x;
            cps[i].geom.pos[1]=pt_col.y;
            cps[i].geom.pos[2]=pt_col.z;

            world->create_contact(cps,i,body_id_rb,body_id_ground,rb1,rb2);

            count ++;
        }
    }

}

void SoftBodyPointMass::load_contact_forces()
{
    if (pts_force.size()!=discrete_pts.size()){
        pts_force.resize(discrete_pts_world.size(),Vector3d(0,0,0));
    }else{
        for (int i=0;i<(int)pts_force.size();++i){
            pts_force[i]=Vector3d(0,0,0);
        }
    }

    int count_contacts=0;
    std::vector<ContactPoint>* cfs=world->getContactForces();
    std::vector<ContactPoint>& contactPoints=*cfs;
    for (uint i = 0; i<contactPoints.size(); i++){
        if ((contactPoints[i].rb1 == rb) || (contactPoints[i].rb2 == rb)){
            Point3d tmpP;
            tmpP = contactPoints[i].cp;
            //            tmpP = rb->getLocalCoordinates(contactPoints[i].cp);
            auto it=std::find(discrete_pts_world.begin(),discrete_pts_world.end(),tmpP);
            if (it!=discrete_pts_world.end()){
                int id=it-discrete_pts_world.begin();
                pts_force[id]=contactPoints[i].f;
            }else{
                std::cout<<"not here";
            }
            count_contacts++;
        }
    }
    {
        std::ostringstream oss;
        oss<< "Nb contact depending on my solid: "<<count_contacts;
        std::cout<<oss.str();
    }
}

void SoftBodyPointMass::forward_mass_points(int integration_type)
{
    //for a test taking in account that with just do a better discretisation of the points
    for (int i=0;i<(int)discrete_pts_world.size();++i){
        discrete_pts_world[i]=rb->getWorldCoordinates(discrete_pts_local[i]);
    }
    return;

    load_contact_forces();

    BoxCDP* box=static_cast<BoxCDP*>(rb->get_cdps().front());

    std::vector<Vector3d> external_forces_vel;
    external_forces_vel.resize(discrete_pts_world.size());

    for (int i=0;i<4;++i){
        force_reader[i]=Vector3d(0,0,0);
        force_pt_reader[i]=rb->getWorldCoordinates(box->get_corner_position(i));
    }
    //count the umber of point with an actual contact to distribute the mass
    int eff_contacts_count=0;
    for (int i=0;i<(int)discrete_pts_world.size();++i){
        if (pts_force[i].length()>0.00001){
            eff_contacts_count++;
        }
    }
    double part_mass=1.0;
    //no need to do anything complicated because if it's
    if (eff_contacts_count>0){
//        part_mass=rb->getMass()/discrete_pts_world.size();
        part_mass=rb->getMass()/eff_contacts_count;
    }
    for (int i=0;i<(int)discrete_pts_world.size();++i){

        Vector3d eff_vel=Vector3d(0,0,0);
        //Vector3d grf_point_mass=Vector3d(0,pts_force[i].y,0);
        eff_vel+=Vector3d(0,SimGlobals::gravity,0)*SimGlobals::dt;
        if (pts_force[i].length()>0.00001){
            eff_vel+=pts_force[i]*SimGlobals::dt/part_mass;
        }
        external_forces_vel[i]=eff_vel;


        if (discrete_pts_local[i].z<0){
            if (discrete_pts_local[i].x<0){
                force_reader[0]+=pts_force[i];
            }else{
                force_reader[1]+=pts_force[i];
            }
        }else{
            if (discrete_pts_local[i].x<0){
                force_reader[2]+=pts_force[i];
            }else{
                force_reader[3]+=pts_force[i];
            }
        }

        std::ostringstream oss;
        oss<<discrete_pts_world[i].y;
        oss<<" // force_applyed: "<<pts_force[i].y;
        std::cout<< oss.str();
    }

    //*
    double kv=fv*km;
    double ke=fe*km;
    double kx_p=fx_p*km;
    //and now we have to compute the elastic forces
    //first I need to store the deformations
    std::vector<Vector3d> vect_deform;
    vect_deform.resize(discrete_pts_world.size());
    Point3d p_target;
    Point3d p_current;
    for (int i=0;i<(int)discrete_pts_world.size();++i){
        if (integration_type==0){
            p_target=discrete_pts[i];
            p_current=discrete_pts_world[i];
        }else if (integration_type==1){
        }else if (integration_type==2){
            p_target=rb->getWorldCoordinates(discrete_pts_local[i]);
            p_current=discrete_pts_world[i]+(discrete_pts_vel_world[i]+external_forces_vel[i])*SimGlobals::dt;
        }else{
            exit(356);
        }
        vect_deform[i]=p_current-p_target;
    }

    for (int i=0;i<(int)discrete_pts_world.size();++i){
        //and now we have to compute the displacement of the point due to the material resistance

        //vertex deformation
        Vector3d f1=vect_deform[i]*(-kv);

        //edge deformation
        Vector3d f2=Vector3d(0,0,0);
        //deformation relative to the point above which is satic so it does not deform
        f2+=vect_deform[i];
        //and now the other neighbours
        for (int j=0;j<discrete_pts_neighbours[i].size();++j){
            int neighbour_id=discrete_pts_neighbours[i][j];
            f2+=vect_deform[i]-vect_deform[neighbour_id];
        }
        f2*=-ke;
        Vector3d d_vel_deformation=(f1+f2)/part_mass*SimGlobals::dt;
        Vector3d d_vel_material=d_vel_deformation;

        //damping
        Vector3d v_target,v_current;
        if (integration_type==0){
            v_target= discrete_pts_vel[i];
            v_current=(discrete_pts_vel_world[i]);
        }else if (integration_type==1){
        }else if (integration_type==2){
            v_target= rb->getAbsoluteVelocityForLocalPoint(discrete_pts_local[i]);
            v_current=(discrete_pts_vel_world[i])+external_forces_vel[i]+d_vel_deformation;
        }else{
            exit(357);
        }
        Vector3d d_v=v_current-v_target;
        Vector3d f_damping=d_v*(-kx_p);
        Vector3d d_vel_damping=f_damping/part_mass*SimGlobals::dt;
        d_vel_material+=d_vel_damping;


        discrete_pts_vel_world[i]+=external_forces_vel[i];
        discrete_pts_vel_world[i]+=d_vel_material;
        discrete_pts_world[i]+=discrete_pts_vel_world[i]*SimGlobals::dt;

        std::ostringstream oss;
        oss<< discrete_pts_world[i].y<<"//"<<(rb->getLocalCoordinates(discrete_pts_world[i]).y-discrete_pts_local[i].y)<<
              "   vel material(deform//damping): "<<d_vel_material.y <<
              " ( "<<d_vel_deformation.y<<" // "<<d_vel_damping.y<<") "<<
              "    vel ext forces: "<<external_forces_vel[i].y;
        std::cout<<oss.str();

    }
    //*/

}

