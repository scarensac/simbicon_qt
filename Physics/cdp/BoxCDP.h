#ifndef BOX_CDP_H
#define BOX_CDP_H

#include <Physics/cdp/CollisionDetectionPrimitive.h>
#include <MathLib/Point3d.h>
#include <MathLib/TransformationMatrix.h>
#include <Utils/Utils.h>

/*========================================================================================================================================================================*
 * This class implements a rectangular box class that will be used as a collision detection primitive.                                                                    *
 * A box is represented by the position of two opposite corners.                                                                                                          *
 *========================================================================================================================================================================*/
class BoxCDP : public CollisionDetectionPrimitive{
private:
	//these are the two corners of the box, expressed in local coordinates.
	Point3d p1, p2;
	
public:
	BoxCDP(RigidBody* theBdy, Point3d& p1_, Point3d& p2_);
	virtual ~BoxCDP(void);


    virtual CollisionDetectionPrimitive* cpy();

    virtual void update_to_world_primitive(){}

	/**
		Draw an outline of the box
	*/
	virtual void draw();

	/**
        some computed getters for geometric values
	*/
    inline Point3d center(){return Point3d((p1.x+p2.x)/2, (p1.y+p2.y)/2, (p1.z+p2.z)/2);}
    inline double X_length(){return (fabs(p1.x-p2.x));}
    inline double Y_length(){return (fabs(p1.y-p2.y));}
    inline double Z_length(){return (fabs(p1.z-p2.z));}

	void compute_finites_elements(){
        /*
		int nbr_interval_x = 3;
		int nbr_interval_y = 2;
		int nbr_interval_z = 9;

        double l_x = X_length();
        double l_y = Y_length();
        double l_z = Z_length();
		double d_x = l_x / nbr_interval_x;
		double d_y = l_y / nbr_interval_y;
		double d_z = l_z / nbr_interval_z;

        Point3d center = this->center();
		Point3d cur_pos, cur_normal;
		
		//first let's handle the back face
		cur_pos = center + Point3d(l_x / 2 + d_x / 2, -l_y / 2 + d_y / 2, -l_z / 2);
		//now the front face
		cur_pos = body->getWorldCoordinates(center + Point3d(-box->getXLen() / 2 + d_x / 2, -box->getYLen() / 2 + d_y / 2, box->getZLen() / 2));
		cur_normal = nz;
		drag_torque += compute_liquid_drag_on_planev2(joint, cur_pos, cur_normal, water_level, wvx, wvy, nbr_interval_x, nbr_interval_y, eff_density);

		//now the left face
		cur_pos = body->getWorldCoordinates(center + Point3d(box->getXLen() / 2, -box->getYLen() / 2 + d_y / 2, -box->getZLen() / 2 + d_z / 2));
		cur_normal = nx;
		drag_torque += compute_liquid_drag_on_planev2(joint, cur_pos, cur_normal, water_level, wvy, wvz, nbr_interval_y, nbr_interval_z, eff_density);

		//now the right face
		cur_pos = body->getWorldCoordinates(center + Point3d(-box->getXLen() / 2, -box->getYLen() / 2 + d_y / 2, -box->getZLen() / 2 + d_z / 2));
		cur_normal = -nx;
		drag_torque += compute_liquid_drag_on_planev2(joint, cur_pos, cur_normal, water_level, wvy, wvz, nbr_interval_y, nbr_interval_z, eff_density);

		//now the top face
		cur_pos = body->getWorldCoordinates(center + Point3d(-box->getXLen() / 2 + d_x / 2, box->getYLen() / 2, -box->getZLen() / 2 + d_z / 2));
		cur_normal = ny;
		drag_torque += compute_liquid_drag_on_planev2(joint, cur_pos, cur_normal, water_level, wvx, wvz, nbr_interval_x, nbr_interval_z, eff_density, friction_coef*l_y, l_y);

		//now the bottom face to finish
		cur_pos = body->getWorldCoordinates(center + Point3d(-box->getXLen() / 2 + d_x / 2, -box->getYLen() / 2, -box->getZLen() / 2 + d_z / 2));
		cur_normal = -ny;
		drag_torque += compute_liquid_drag_on_planev2(joint, cur_pos, cur_normal, water_level, wvx, wvz, nbr_interval_x, nbr_interval_z, eff_density, friction_coef*l_y, l_y);
        */

    }

    // easy acces to the coordinate oa each corner of the box
    // bottom  2 3      top 6 7
    //         0 1          4 5
    Point3d get_corner_position(int id);


    std::vector<Point3d> generate_subdivision(int nb_division_by_face, int nb_layers=1, bool generate_top_layer=false);
    std::vector<std::vector<int> > generate_subdivision_neighbours(int nb_division_by_face, int nb_layers=1, bool generate_top_layer=false);

    std::string to_xml(int nb_tab);
};

#endif
