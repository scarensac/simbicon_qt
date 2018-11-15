#ifndef SOFTBODYPOINTMASS_H
#define SOFTBODYPOINTMASS_H

#include <vector>
#include <MathLib/Point3d.h>
#include <MathLib/Vector3d.h>
#include <ode/ode.h>

class RigidBody;
class ODEWorld;
class SoftBodyPointMass
{
public:
    RigidBody* rb;
    dGeomID geom_id_rb;
    dBodyID body_id_rb;
    RigidBody* rb_ground;
    dGeomID geom_id_ground;
    dBodyID body_id_ground;
    ODEWorld* world;

    std::vector<Point3d> discrete_pts;
    std::vector<Point3d> discrete_pts_vel;
    std::vector<Point3d> discrete_pts_world;
    std::vector<Point3d> discrete_pts_vel_world;
    std::vector<std::vector<int> > discrete_pts_neighbours;
    std::vector<Point3d> discrete_pts_local;


    Vector3d force_reader[4];
    Point3d force_pt_reader[4];

    std::vector<Vector3d> pts_force;


    double km;
    double fe;
    double fv;
    double fx_p;


    SoftBodyPointMass(RigidBody* rb_i, ODEWorld* world_i, double km_i, double fx_p_i=0.1 , double fv_i=0.4,
                      double fe_i=0.6);

    /**
     * @brief init the values (this have to be done everytime the boy is moved by means outside of the simulation
     * @param nb_subdivision
     */
    void init(int nb_subdivision);

    /**
     * @brief update_target_position you need to execute BEFORE every simulation step (before checking for collisions
     */
    void update_target_position();

    /**
     * @brief generate_contacts need to be executed right before forwarding the simulation
     */
    void generate_contacts();

    /**
     * @brief load_contact_forces need to be executed right after the simulation before doing any computation on the point mass
     * actualy it is called at the start of the forward_mass_points so don't call it ...
     */
    void load_contact_forces();

    /**
     * @brief forward_mass_points
     * @param integration_type 0 for explicit, 1 for leap frog, 2 for implicit
     */
    void forward_mass_points(int integration_type=0);


};

#endif // SOFTBODYPOINTMASS_H
