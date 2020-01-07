#include "rigidbody.h"

#include <Utils/Utils.h>
#include <iostream>

#include <Physics/rb/RBUtils.h>
#include <GLUtils/OBJReader.h>
#include <Physics/cdp/CapsuleCDP.h>
#include <Physics/cdp/PlaneCDP.h>
#include <Physics/cdp/BoxCDP.h>
#include <Physics/cdp/SphereCDP.h>

#include "Core/SimGlobals.h"


/**
    Default constructor - give sensible values to the class members
*/
RigidBody::RigidBody(){
    static uint idx_count = 0;
    _idx = idx_count;
    idx_count++;
    r=1;
    g=1;
    b=1;
    a=1;

    angular_vel_avg_max_size=4;
    angular_velocity_avg=Vector3d(0,0,0);
    //	toWorld.loadIdentity();
}

RigidBody::RigidBody(FILE *f):RigidBody()
{
    loadFromFile(f);
}



/**
    Default destructor - free up all the memory that we've used up
*/
RigidBody::~RigidBody(void){
    for (uint i=0;i<meshes.size();i++){
        delete meshes[i];
    }
    meshes.clear();

    for (uint i=0;i<cdps.size();i++){
        delete cdps[i];
    }
    meshes.clear();
}

RigidBody *RigidBody::cpy(bool keep_graphics){
    RigidBody* bdy=new RigidBody();
    bdy->state=state;
    bdy->props=props;
    bdy->_name=_name;
    bdy->_idx=_idx;

    //in depth copy for the cdps
    for (int i=0;i<cdps.size();++i){
        bdy->cdps.push_back(cdps[i]->cpy());
    }

    //same for the meshs
    //forget it for now it's useless and I don't even know what to copy in it ...
    if (keep_graphics){

        //bdy->meshes=meshes;
    }

    return bdy;
}

/**
    This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
*/
Point3d RigidBody::getWorldCoordinates(const Point3d& localPoint){
    return this->state.position + getWorldCoordinates(Vector3d(localPoint));
}

/**
    This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
*/
Vector3d RigidBody::getWorldCoordinates(const Vector3d& localVector){
    //the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
    return this->state.orientation.rotate(localVector);
}

/**
    This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
*/
Point3d RigidBody::getLocalCoordinates(const Point3d& globalPoint){
    Vector3d v = getLocalCoordinates(Vector3d(globalPoint)) - getLocalCoordinates(Vector3d(this->state.position));
    return Point3d(0,0,0) + v;
}

/**
    This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
*/
Vector3d RigidBody::getLocalCoordinates(const Vector3d& globalVector){
    //the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
    return this->state.orientation.getComplexConjugate().rotate(globalVector);
}

/**
    This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
    resulting velocity will be expressed in world coordinates.
*/
Vector3d RigidBody::getAbsoluteVelocityForLocalPoint(const Point3d& localPoint){
    //we need to compute the vector r, from the origin of the body to the point of interest
    Vector3d r(Point3d(), localPoint);
    //the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
    return state.angular_velocity.crossProductWith(getWorldCoordinates(r)) + state.velocity;
}

/**
    This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
    resulting velocity will be expressed in world coordinates.
*/
Vector3d RigidBody::getAbsoluteVelocityAvgForLocalPoint(const Point3d& localPoint){
    //we need to compute the vector r, from the origin of the body to the point of interest
    Vector3d r(Point3d(), localPoint);
    //the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
    return get_angular_velocity_avg().crossProductWith(getWorldCoordinates(r)) + getCMVelocity();
}

/**
    This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
    resulting velocity will be expressed in world coordinates.
*/
Vector3d RigidBody::getAbsoluteVelocityForGlobalPoint(const Point3d& globalPoint){
    //we need to compute the vector r, from the origin of the body to the point of interest
    Vector3d r(state.position, globalPoint);
    //the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
    return state.angular_velocity.crossProductWith(r) + state.velocity;
}


Quaternion RigidBody::getOrientationFuture(){
    Quaternion q=state.orientation;

    //we add the angular velocitues to estimate the future state
    q=Quaternion::deltaRotation(getAngularVelocity(),SimGlobals::dt)*q;
    return q;
}

Vector3d RigidBody::getAngularVelocityFuture(){
    Vector3d ang_vel_fut=getAngularVelocity()+state.angular_acc*SimGlobals::dt;
    return ang_vel_fut;
}


/**
    This method draws the current rigid body.
*/
void RigidBody::draw(int flags){
    //multiply the gl matrix with the transformations needed to go from local space into world space
    glPushMatrix();

    GLboolean lighting = glIsEnabled(GL_LIGHTING);

    TransformationMatrix toWorld;
    this->state.orientation.getRotationMatrix(&toWorld);
    toWorld.setTranslation(this->state.position);

    double values[16];
    toWorld.getOGLValues(values);
    glMultMatrixd(values);

    //draw the collision detection primitives if any
    if (flags & SHOW_CD_PRIMITIVES){

        float tempColor[] = {(float)r,(float)g,(float)b,(float)(a)};
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, tempColor);
        //if (flags & SHOW_JOINTS)
        {
            if (lighting)
                glDisable(GL_LIGHTING);
            glColor3d(0.0, 0.6, 0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }

        if (_name.find("empty") != std::string::npos) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }

        for (uint i=0;i<cdps.size();i++)
            cdps[i]->draw();
        //if (flags & SHOW_JOINTS)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        if (_name.find("empty") != std::string::npos) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
    }


    if (lighting)
        glDisable(GL_LIGHTING);

    /* also draw a set of axes that belong to the local coordinate system*/
    if (flags & SHOW_BODY_FRAME)
        GLUtils::drawAxes(0.05);

    if (lighting)
        glEnable(GL_LIGHTING);

    // now we'll draw the object's mesh
    if (meshes.size()>0 && (flags & SHOW_MESH) && !(flags & SHOW_CD_PRIMITIVES)){
        if ((flags & SHOW_CD_PRIMITIVES) || (flags & SHOW_JOINTS))
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        if (flags & SHOW_COLOURS){
            for (uint i=0;i<meshes.size();i++){
                meshes[i]->drawMesh(true);
            }

        }else
            for (uint i=0;i<meshes.size();i++)
                meshes[i]->drawMesh(false);
        if ((flags & SHOW_CD_PRIMITIVES) || (flags & SHOW_JOINTS))
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    glPopMatrix();
}

/**
    this method is used to update the world positions of the collision detection primitives
*/
void RigidBody::updateWorldCDPs(){
    for (uint i=0;i<cdps.size();i++)
        cdps[i]->update_to_world_primitive();
}

/**
    This method renders the rigid body in its current state as a set of vertices
    and faces that will be appended to the passed OBJ file.

    vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
    multiple different meshes to the same OBJ file

    Returns the number of vertices written to the file
*/
uint RigidBody::renderToObjFile(FILE* fp, uint vertexIdxOffset) {
    int retVal = 0;
    //	for (uint i=0;i<meshes.size();i++)
    //		retVal = meshes[i]->renderToObjFile( fp, vertexIdxOffset, this->toWorld );
    return retVal;
}

/**
    This method is used to compute the correct toWorld coordinates matrix based on the state of the rigid body
*/
//void RigidBody::updateToWorldTransformation(){
//	this->state.orientation.getRotationMatrix(&toWorld);
//	toWorld.setTranslation(this->state.position);
//}

/**
    This method loads all the pertinent information regarding the rigid body from a file.
*/
void RigidBody::loadFromFile(FILE* f){
    if (f == NULL)
        throwError("Invalid file pointer.");

    //have a temporary buffer used to read the file line by line...
    char buffer[200];
    char meshName[200];

    //temporary variables that we may end up populating
    double r, g, b, a;
    Point3d p1, p2;
    Vector3d n;
    double t1, t2, t3;
    double t;
    GLMesh* tmpMesh;

    //this is where it happens.
    while (!feof(f)){
        //get a line from the file...
        fgets(buffer, 200, f);
        if (strlen(buffer)>195)
            throwError("The input file contains a line that is longer than ~200 characters - not allowed");
        char *line = lTrim(buffer);
        int lineType = getRBLineType(line);

        std::string path;

        switch (lineType) {
            case RB_NAME:{
                _name=std::string(line);
                //we remove all the white spaces (' ', '\n',...)
                rmv_white_spaces(_name);
                break;
            }
            case RB_MESH_NAME:{
                //first we must correct the path
                path = std::string(line);
                path=interpret_path(path);
                //and now we can use it
                strcpy(meshName, path.c_str());

                tmpMesh = OBJReader::loadOBJFile(meshName);
                tmpMesh->computeNormals();
                tmpMesh->dontUseTextureMapping();
                meshes.push_back(tmpMesh);
                break;
            }
            case RB_MASS:
                if (sscanf(line, "%lf", &t)!=1)
                    throwError("Incorrect rigid body input file - a mass needs to be specified if the 'mass' keyword is used.");
                this->props.set_mass(t);
                break;
            case RB_MOI:
                if (sscanf(line, "%lf %lf %lf", &t1, &t2, &t3)!=3)
                    throwError("Incorrect rigid body input file - the three principal moments of inertia need to be specified if the 'moi' keyword is used.");
                if (t1<=0 || t2<=0 || t3<=0)
                    throwError("Incorrect values for the principal moments of inertia.");
                this->props.set_MOI(t1, t2, t3);
                break;
            case RB_END_RB:
                return;//and... done
                break;
            case RB_COLOUR:
                if (sscanf(line, "%lf %lf %lf %lf", &r, &g, &b, &a)!=4)
                    throwError("Incorrect rigid body input file - colour parameter expects 4 arguments (colour %s)\n", line);
                this->r=r;
                this->g=g;
                this->b=b;
                this->a=a;
                if (meshes.size()>0)
                    meshes[meshes.size()-1]->setColour(r, g, b, a);
                break;
            case RB_SPHERE:
                if (sscanf(line, "%lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &r)!=4)
                    throwError("Incorrect rigid body input file - 4 arguments are required to specify a sphere collision detection primitive\n", line);
                cdps.push_back(new SphereCDP(this, p1, r));
                break;
            case RB_CAPSULE:
                if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &p2.x, &p2.y, &p2.z, &r)!=7)
                    throwError("Incorrect rigid body input file - 7 arguments are required to specify a capsule collision detection primitive\n", line);
                cdps.push_back(new CapsuleCDP(this, p1, p2, r));
                break;
            case RB_BOX:
                if (sscanf(line, "%lf %lf %lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &p2.x, &p2.y, &p2.z)!=6)
                    throwError("Incorrect rigid body input file - 6 arguments are required to specify a box collision detection primitive\n", line);
                cdps.push_back(new BoxCDP(this, p1, p2));
                break;
            case RB_PLANE:
                if (sscanf(line, "%lf %lf %lf %lf %lf %lf", &n.x, &n.y, &n.z, &p1.x, &p1.y, &p1.z)!=6)
                    throwError("Incorrect rigid body input file - 6 arguments are required to specify a plane collision detection primitive\n", line);
                cdps.push_back(new PlaneCDP(this, n, p1));
                break;
            case RB_NOT_IMPORTANT:
                if (strlen(line)!=0 && line[0] != '#')
                    std::cerr<<"Ignoring input line: "<<line<<std::endl;
                break;
            case RB_LOCKED:
                this->props.lock_body();
                break;
            case RB_POSITION:
                if (sscanf(line, "%lf %lf %lf", &state.position.x, &state.position.y, &state.position.z)!=3)
                    throwError("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates position of a rigid body\n", line);
                break;
            case RB_ORIENTATION:
                if (sscanf(line, "%lf %lf %lf %lf", &t, &t1, &t2, &t3)!=4)
                    throwError("Incorrect rigid body input file - 4 arguments are required to specify the world coordinates orientation of a rigid body\n", line);
                state.orientation = Quaternion::getRotationQuaternion(t, Vector3d(t1, t2, t3).toUnit()) * state.orientation;
                break;
            case RB_VELOCITY:
                if (sscanf(line, "%lf %lf %lf", &state.velocity.x, &state.velocity.y, &state.velocity.z)!=3)
                    throwError("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates velocity of a rigid body\n", line);
                break;
            case RB_ANGULAR_VELOCITY:
                if (sscanf(line, "%lf %lf %lf", &state.angular_velocity.x, &state.angular_velocity.y, &state.angular_velocity.z)!=3)
                    throwError("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates angular velocity of a rigid body\n", line);
                break;
            case RB_FRICTION_COEFF:
                if (sscanf(line, "%lf", &props.mu)!=1)
                    throwError("Incorrect rigid body input file - Expecting a value for the friction coefficient");
                if (props.mu<0)
                    throwError("Incorrect rigid body input file - Friction coefficient should be >= 0");
                break;
            case RB_RESTITUTION_COEFF:
                if (sscanf(line, "%lf", &props.epsilon)!=1)
                    throwError("Incorrect rigid body input file - Expecting a value for the restitution coefficient");
                if (props.epsilon<0 || props.epsilon>1)
                    throwError("Incorrect rigid body input file - restitution coefficient should be between 0 and 1");
                break;
            case RB_ODE_GROUND_COEFFS:
                if (sscanf(line, "%lf %lf", &t1, &t2)!=2)
                    throwError("Two parameters need to be provided for the ODE ground parameter settings");
                props.groundSoftness = t1;
                props.groundPenalty = t2;
                break;
            case RB_PLANAR:
                props.isPlanar = true;
                break;
            default:
                throwError("Incorrect rigid body input file: \'%s\' - unexpected line.", buffer);
        }
    }
    throwError("Incorrect articulated body input file! No /End found");
}

