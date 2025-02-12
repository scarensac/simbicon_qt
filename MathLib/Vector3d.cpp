/*
    Simbicon 1.5 Controller Editor Framework,
    Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
    All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

    This file is part of the Simbicon 1.5 Controller Editor Framework.

    Simbicon 1.5 Controller Editor Framework is free software: you can
    redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Simbicon 1.5 Controller Editor Framework is distributed in the hope
    that it will be useful, but WITHOUT ANY WARRANTY; without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Simbicon 1.5 Controller Editor Framework.
    If not, see <http://www.gnu.org/licenses/>.
*/

#include "Vector3d.h"
#include <MathLib/Matrix.h>
#include <iostream>
/*
    This file implements the methods for the Vector3d class.
*/	

Vector3d::~Vector3d(){
}

Vector3d Vector3d::crossProductWith(const Vector3d &v) const{
    /*
            Easiest way to figure it out is to set it up like this:
                                ___________________
                            Ux | Uy   Uz   Ux   Uy | Uz
                            Vx | Vy   Vz   Vx   Vy | Vz
                                -------------------
            Cross product is given by cross multiplying the items in the box, and subing the other
            diagonal
        */
    Vector3d result;
    result.x = y * v.z - z * v.y;
    result.y = z * v.x - x * v.z;
    result.z = x * v.y - y * v.x;

    {
        Vector3d t=result;
        if(result.is_nan()){
            std::cout<<x<<" "<<y<<" "<<z<<std::endl;
            std::cout<<v.x<<" "<<v.y<<" "<<v.z<<std::endl;
            std::cout<<result.x<<" "<<result.y<<" "<<result.z<<std::endl;
            throw("Vector3d::crossProductWith nan detected");
        }
    }

    return result;
}//nothing to do here...


//constructors
Vector3d::Vector3d(): ThreeTuple(){

}

Vector3d::Vector3d(const Point3d &p) : ThreeTuple(p.x, p.y, p.z){

}

Vector3d::Vector3d(double x, double y, double z) : ThreeTuple(x,y,z){
}

Vector3d::Vector3d(double x, double y) : ThreeTuple(x,y){
}


Vector3d::Vector3d(const Point3d &p1, const Point3d &p2) : ThreeTuple(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z){
}

Vector3d::Vector3d(const Vector3d &other) : ThreeTuple(other.x, other.y, other.z){
}//done with constructors


/**
    this method returns a vector that is the current vector, rotated by an angle alpha (in radians) around the axis given as parameter.
    IT IS ASSUMED THAT THE VECTOR PASSED IN IS A UNIT VECTOR!!!
*/
Vector3d Vector3d::rotate(double alpha, const Vector3d &axis){
    //ok... this is fairly hard - check the red book for details.
    double xP = axis.getX();
    double yP = axis.getY();
    double zP = axis.getZ();
    double cosa = cos(alpha);
    double sina = sin(alpha);

    double s[3][3] = {{0,			-zP,		yP},
                      {zP,			0,			-xP},
                      {-yP,			xP,			0}};
    double UUT[3][3]	= {	{xP*xP,		xP*yP,		xP*zP},
                            {yP*xP,		yP*yP,		yP*zP},
                            {zP*xP,		zP*yP,		zP*zP}};
    double I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    double R[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

    for (int i=0;i<3;i++)
        for (int j = 0;j<3;j++)
            R[i][j] = UUT[i][j] + cosa*(I[i][j] - UUT[i][j]) + sina*s[i][j];

    //now that we finally have the transformation matrix set up, we can rotate the vector
    Vector3d result;

    result.setX(R[0][0]*x + R[0][1]*y + R[0][2]*z);
    result.setY(R[1][0]*x + R[1][1]*y + R[1][2]*z);
    result.setZ(R[2][0]*x + R[2][1]*y + R[2][2]*z);

    return result;
}

/**
    this method returns the cross product matrix - r*
*/
void Vector3d::setCrossProductMatrix(Matrix *m) const{
    if (m->getColumnCount()!=3 || m->getRowCount()!=3)
        throwError("the matrix passed in as a target for the cross product matrix has the wrong dimensions.");
    double data[9] =
    {0,		-z,		y,
     z,		0,		-x,
     -y,	x,		0 };

    m->setValues(data);
}


