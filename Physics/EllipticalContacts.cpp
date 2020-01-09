#include "EllipticalContacts.h"
#include "GLutils/glutils.h"
#include "Physics/rb/ArticulatedRigidBody.h"
#include "Physics/joints/Joint.h"
#include "MathLib/Matrix.h"
#include <iostream>
#include <cmath>

//*
Ellipsoid::Ellipsoid(float x, float y, float z, float rx, float ry, float rz, float rotx, float roty, float rotz)
{
    p=Point3d(x,y,z);
    r=Vector3d(rx,ry,rz);
    Q=Quaternion(Vector3d(rotx,roty,rotz));
}//*/

void Ellipsoid::draw()
{
    //*
    glPushMatrix();



    TransformationMatrix toWorld;
    Q.getRotationMatrix(&toWorld);
    toWorld.setTranslation(p);

    double values[16];
    toWorld.getOGLValues(values);
    glMultMatrixd(values);

    GLUtils::drawEllipsoid(15,15,r.x,r.y,r.z);
    glPopMatrix();
    //*/
}

Point3d Ellipsoid::getWorldCoordinates(const Point3d& localPoint){
    return p + getWorldCoordinates(Vector3d(localPoint));
}

Vector3d Ellipsoid::getWorldCoordinates(const Vector3d& localVector){
    return Q.to_world_coordinate(localVector);
}

Point3d Ellipsoid::getLocalCoordinates(const Point3d& globalPoint){
    Vector3d v = globalPoint-p;
    return Point3d(0,0,0) + getLocalCoordinates(v);
}


Vector3d Ellipsoid::getLocalCoordinates(const Vector3d& globalVector){
    return Q.to_local_coordinate(globalVector);
}


EllipticalContacts::EllipticalContacts(ArticulatedRigidBody *foot_i, bool is_left_foot)
{
    if (foot_i==NULL){
        throw("nope too much fail");
    }
    Kv=1.5E7;
    Av=-0.5;

    v_t=0.01;
    mu_k=1;
    mu_s=mu_k;

    foot=foot_i;
    toes=foot->child_joints()[0]->child();

    //create the 3 ellipsoids following the paper
    Ellipsoid elli;
    //elli= Ellipsoid(0,1.5,0,1,2,1,0,0,PI/4);
    //elli= Ellipsoid(0,-0.025,-0.075,0.125,0.125,0.125,0,0,PI/4);
    elli= Ellipsoid(0,-0.025,-0.075,0.030,0.015,0.020,0,0,0);
    ellipsoids.push_back(elli);
    bodies.push_back(foot);
    elli= Ellipsoid(0,-0.025,0.045,0.030,0.015,0.020,0,0,0);
    ellipsoids.push_back(elli);
    bodies.push_back(foot);
    elli= Ellipsoid(0.01*((is_left_foot)?-1:1),-0.005,0.020,0.010,0.015,0.015,0,0,0);
    ellipsoids.push_back(elli);
    bodies.push_back(toes);

    IntersectionVolume.resize(3);
    PressureCenter.resize(3);
    forces_N.resize(3);
    forces_T.resize(3);
}

void EllipticalContacts::draw()
{
    if (ellipsoids.size()!=3){
        return;
    }



    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    //the foot related ellipsoids
    {
        glPushMatrix();
        TransformationMatrix toWorld;
        foot->getOrientation().getRotationMatrix(&toWorld);
        toWorld.setTranslation(foot->getCMPosition());

        double values[16];
        toWorld.getOGLValues(values);
        glMultMatrixd(values);

        ellipsoids[0].draw();
        ellipsoids[1].draw();

        glPopMatrix();
    }


    //the toes related ellipsoids
    {
        glPushMatrix();

        TransformationMatrix toWorld;
        toes->getOrientation().getRotationMatrix(&toWorld);
        toWorld.setTranslation(toes->getCMPosition());

        double values[16];
        toWorld.getOGLValues(values);
        glMultMatrixd(values);


        ellipsoids[2].draw();
        glPopMatrix();

    }


    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


    bool drawCentroid=true;
    if (drawCentroid){
        glDisable(GL_LIGHTING);
        glColor3b(1,0,0);

        {
            glPushMatrix();
            TransformationMatrix toWorld;
            /*
            Quaternion q;
            q.getRotationQuaternion(0,Vector3d(1,0,0));
            q.getRotationMatrix(&toWorld);//*/
            foot->getOrientation().getRotationMatrix(&toWorld);
            toWorld.setTranslation(foot->getCMPosition());

            double values[16];
            toWorld.getOGLValues(values);
            glMultMatrixd(values);

            for(int i=0;i<2;++i){
                if(IntersectionVolume[i]>0){
                    GLUtils::drawSphere(ellipsoids[i].getWorldCoordinates(PressureCenter[i]),0.001);
                }
            }


            glPopMatrix();
        }


        //the toes related ellipsoids
        {
            glPushMatrix();

            TransformationMatrix toWorld;
            toes->getOrientation().getRotationMatrix(&toWorld);
            toWorld.setTranslation(toes->getCMPosition());

            double values[16];
            toWorld.getOGLValues(values);
            glMultMatrixd(values);



            for(int i=2;i<3;++i){
                if(IntersectionVolume[i]>0){
                    GLUtils::drawSphere(ellipsoids[i].getWorldCoordinates(PressureCenter[i]),0.001);
                }
            }

            glPopMatrix();

        }
        glEnable(GL_LIGHTING);
    }
}

void EllipticalContacts::computeForces()
{
    int groundHeight=0;

    for (int i =0;i<3;++i){
        IntersectionVolume[i]=0;
        forces_N[i]=Vector3d(0,0,0);
        forces_T[i]=Vector3d(0,0,0);
        //the concept is to do the comoutations on a sphere, then convert the results to the ellisoid
        //so first I must converthe world to a sphere

        Ellipsoid elli=ellipsoids[i];
        ArticulatedRigidBody* arb=bodies[i];


        //first the scaling matrices
        Matrix S(3,3);
        S.loadZero();
        S.set(0,0,elli.r.x);
        S.set(1,1,elli.r.y);
        S.set(2,2,elli.r.z);

        Matrix S_inv;
        S_inv.setToInverseOf(S);

        //now I need the normal vectors
        //let's try to work in local coordinates
        Quaternion q;
        //q=elli.Q;
        //Vector3d T_to_local=elli.p;
        q.setToProductOf(arb->getOrientation(),elli.Q);
        Vector3d T_to_local=arb->getWorldCoordinates(elli.p);


        //test to see that the matrix computationa are correct
        if (false){
            Vector3d s_t=S_inv*elli.r;
            std::cout<<"test: "<<s_t.x<<"   "<<s_t.y<<"   "<<s_t.z<<std::endl;
        }

        //so the plane should be y=0 in the world so the normal is 0 1 0
        Vector3d n_w(0,1,0);
        Vector3d n_e=q.to_local_coordinate(n_w);
        Vector3d n_s=S*n_e;
        n_s.toUnit();

        //we neeed a point of the plane so let's just use the origin;
        Vector3d p_w=-T_to_local;

        Vector3d p_e=q.to_local_coordinate(p_w);
        Vector3d p_s=S_inv*p_e;
        float d=1+p_s.dotProductWith(n_s);

        //std::cout<<"d"<<i<<": "<<d<<std::endl;
        if(d<0){
            continue;
        }


        //compute the volume in both referencials
        float V_s=PI*d*d*(3-d)/3;
        float V_e=V_s*elli.r.x*elli.r.y*elli.r.z;
        IntersectionVolume[i]=V_e;


        //now the centroid
        float C_s_real=(3*(d-2)*(d-2))/(4*d-12);
        Vector3d C_s=n_s;
        C_s.multiplyBy(C_s_real);
        Vector3d C_e=S*C_s;
        PressureCenter[i]=C_e;


        //and now we can compute the force (normal force)
        Vector3d v_c=arb->getAbsoluteVelocityForLocalPoint(elli.getWorldCoordinates(PressureCenter[i]));
        float F_n_real=Kv*V_s*(1+Av*v_c.y);
        forces_N[i]=Vector3d(0,F_n_real,0);

        //*
        std::cout<<std::iostream::scientific<<"force elli "<<i<<"   : "<<F_n_real<<"   volume: "<<V_e<<"  at position "<<
                   C_e.x<<"  "<<C_e.y<<"  "<<C_e.z<<"  "<<std::endl;
                   //*/

        //now we must compute the frictions force
        //starting with the tangencial friction
        Vector3d v_ct=v_c;
        v_ct.y=0;
        float v_ct_ratio=v_ct.length()/v_t;
        float F_t_real=mu_k*tanh(4*v_ct_ratio);
        F_t_real+=(mu_s-mu_k)*(v_ct_ratio)/(v_ct_ratio*v_ct_ratio/4+3/4);
        forces_T[i]=v_ct.unit()*F_t_real;

        continue;

        //for rolling and spinning friction we need the second moment of area
        //first we define it for the sphere
        Matrix J_s(3,3);
        J_s.loadIdentity();
        float J_s_n=(d*d*d/30.0)*PI*(3*d*d-15*d+20);
        //note J_s_t = J_s_n/2
        J_s.set(0,0,J_s_n/2);
        J_s.set(1,1,J_s_n);
        J_s.set(2,2,J_s_n/2);
        //now to convert that to the ellpsoid I need two thing
        //first the rotation matrix from the ground to the local sphere in the sphere referencial.
        //I cna build it the same way we built n_s
        Vector3d t_w_x(1,0,0);
        Vector3d t_e_x=q.to_local_coordinate(t_w_x);
        Vector3d t_s_x=S*t_e_x;
        t_s_x.toUnit();

        Vector3d t_w_z(0,0,1);
        Vector3d t_e_z=q.to_local_coordinate(t_w_z);
        Vector3d t_s_z=S*t_e_z;
        t_s_z.toUnit();

        ///TODO check that this is actually an orthogonal referencial
        Matrix R_ps(3,3);
        R_ps.set(0,0,t_s_x.x);
        R_ps.set(0,1,t_s_x.x);
        R_ps.set(0,2,t_s_x.z);
        R_ps.set(1,0,n_s.x);
        R_ps.set(1,1,n_s.x);
        R_ps.set(1,2,n_s.z);
        R_ps.set(2,0,t_s_z.x);
        R_ps.set(2,1,t_s_z.x);
        R_ps.set(2,2,t_s_z.z);


    }

}

void EllipticalContacts::scaleSecondMomentMatrix(Matrix &M, float a, float b, float c)
{
    float J_xx=a*b*c/2*(b*b*(M.get(0,0)+M.get(2,2)-M.get(1,1))+c*c*(M.get(0,0)+M.get(1,1)-M.get(2,2)));
    float J_yy=a*b*c/2*(a*a*(M.get(1,1)+M.get(2,2)-M.get(0,0))+c*c*(M.get(1,1)+M.get(0,0)-M.get(2,2)));
    float J_zz=a*b*c/2*(a*a*(M.get(2,2)+M.get(1,1)-M.get(0,0))+b*b*(M.get(2,2)+M.get(0,0)-M.get(1,1)));
    float J_xy=a*a*b*b*c*M.get(0,1);
    float J_xz=a*a*c*c*b*M.get(0,2);
    float J_yz=b*b*c*c*a*M.get(1,2);

    M.set(0,0,J_xx);
    M.set(0,1,J_xy);
    M.set(0,2,J_xz);
    M.set(1,0,J_xy);
    M.set(1,1,J_yy);
    M.set(1,2,J_yz);
    M.set(2,0,J_xz);
    M.set(2,1,J_yz);
    M.set(2,2,J_zz);

}

Vector3d EllipticalContacts::getSumForces()
{
    Vector3d sum_forces=Vector3d(0,0,0);
    for(int i=0;i<ellipsoids.size();++i){
        sum_forces+=forces_N[i]+forces_T[i];
    }
    return sum_forces;
}
