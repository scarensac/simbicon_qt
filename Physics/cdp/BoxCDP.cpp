
#include "BoxCDP.h"
#include <GLUtils/GLUtils.h>

BoxCDP::BoxCDP(RigidBody* theBdy, Point3d& p1_, Point3d& p2_) : CollisionDetectionPrimitive(theBdy){
    this->p1 = p1_;
    this->p2 = p2_;
    _type = BOX_CDP;
}


BoxCDP::~BoxCDP(void){

}

CollisionDetectionPrimitive *BoxCDP::cpy()
{
    BoxCDP* tmp=new BoxCDP(_bdy,p1,p2);
    return tmp;
}

/**
    Draw an outline of the box
*/
void BoxCDP::draw(){
    GLUtils::drawBox(p1, p2);
}

Point3d BoxCDP::get_corner_position(int id)
{
    Point3d result=center();
    if (id>3){
        result.y+=Y_length()/2;
    }else{
        result.y-=Y_length()/2;
    }

    if (id&1){
        result.x+=X_length()/2;
    }else{
        result.x-=X_length()/2;
    }

    if ((id&2)){
        result.z+=Z_length()/2;
    }else{
        result.z-=Z_length()/2;
    }
    return result;

}

std::vector<Point3d> BoxCDP::generate_subdivision(int nb_division_by_face, int nb_layers, bool generate_top_layer)
{
    Point3d min_pos;
    min_pos.x=std::min(p1.x,p2.x);
    min_pos.y=std::min(p1.y,p2.y);
    min_pos.z=std::min(p1.z,p2.z);

    Point3d max_pos;
    max_pos.x=std::max(p1.x,p2.x);
    max_pos.y=std::max(p1.y,p2.y);
    max_pos.z=std::max(p1.z,p2.z);

    //loop on the height
    double x_step=X_length()/nb_division_by_face;
    double y_step=Y_length()/nb_layers;
    double z_step=Z_length()/nb_division_by_face;
    std::vector<Point3d> vect_points;
    if (generate_top_layer){
        nb_layers++;
    }
    for (int h=0; h<nb_layers ; ++h){
        for (int i=0; i<nb_division_by_face+1 ; ++i){
            for (int j=0; j<nb_division_by_face+1 ; ++j){
                Point3d point=min_pos+Vector3d(j*x_step,h*y_step,i*z_step);
                vect_points.push_back(point);
            }
        }
    }
    return vect_points;
}

std::vector<std::vector<int> > BoxCDP::generate_subdivision_neighbours(int nb_division_by_face, int nb_layers, bool generate_top_layer)
{
    std::vector<int> neighbours;
    std::vector<std::vector<int> > vect_pts_neighbours;

    //I supose there are never multiples layers
    //one note I'd like the neighbours to be sorted.
    //the easy way is to handle bottom left right top
    int nb_points_x=nb_division_by_face+1;
    int nb_points_z=nb_division_by_face+1;
    int nb_points=nb_points_x*nb_points_z;
    for (int i=0; i<nb_points ; ++i){
        neighbours.clear();
        //if we are not on the bottom line we have a lower neighbour
        //this just check if the previous line exist
        if ((i-nb_points_x)>=0){
            neighbours.push_back(i-nb_points_x);
        }

        //if we are not in the left border we have a left neighbour
        if ((i%nb_points_x)>0){
            neighbours.push_back(i-1);
        }

        //if we are not on the right border we have a right neighbour
        //to check it I just check if the next point is the first of the new line
        if (((i+1)%nb_points_x)>0){
            neighbours.push_back(i+1);
        }

        //and lastly if we are not in the last line we have a top neighbour
        //this just check if the next line exist
        if ((i+nb_points_x)<(nb_points)){
            neighbours.push_back(i+nb_points_x);
        }

        vect_pts_neighbours.push_back(neighbours);
    }


    return vect_pts_neighbours;
}

#include <sstream>
std::string BoxCDP::to_xml(int nb_tab)
{
    // <geom type="box" group="1" "0.02 0.02 0.02" contype="0" conaffinity="0" rgba=".9 .5 .5 1"/>
    std::ostringstream oss;
    oss<<tab_string(nb_tab)<<"<geom type=\"box\" "<<
         "size=\""<<(p2.x-p1.x)/2<<" "<<(p2.z-p1.z)/2<<" "<<(p2.y-p1.y)/2<<"\" "<<
         "pos=\""<<-(p2.x+p1.x)/2<<" "<<(p2.z+p1.z)/2<<" "<<(p2.y+p1.y)/2<<"\" "<<
         "/>"<<std::endl;
    return oss.str();
}
