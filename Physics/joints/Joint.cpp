#include "Joint.h"

#include <iostream>

#include "Utils/Utils.h"
#include "Physics/rb/AbstractRBEngine.h"
#include "Physics/rb/RBUtils.h"
#include "Core/SimGlobals.h"

Joint::Joint(){
    static uint idx_count = 0;
    _idx = idx_count;
    idx_count++;

    this->_parent = NULL;
    this->_child = NULL;;
    _use_joint_limits = false;
}

void Joint::set_articuled_figure(ArticulatedFigure *figure)
{
    _child->_af = figure;
    _parent->_af = figure;
}

Joint::~Joint(void){

}

/**
    This method is used to compute the relative orientation between the parent and the child rigid bodies, expressed in
    the frame coordinate of the parent.
*/
void Joint::compute_relative_orientation(Quaternion& qRel){
    //if qp is the quaternion that gives the orientation of the parent, and qc gives the orientation of the child, then  qp^-1 * qc gives the relative
    //orientation between the child and the parent, expressed in the parent's coordinates (child to parent)
    qRel = _parent->getOrientation().getComplexConjugate() * _child->getOrientation();
}

void Joint::compute_relative_orientation_Future(Quaternion& qRel){
    Quaternion qp=_parent->getOrientationFuture();
    Quaternion qc=_child->getOrientationFuture();

    //if qp is the quaternion that gives the orientation of the parent,
    //and qc gives the orientation of the child, then  qp^-1 * qc gives the relative
    //orientation between the child and the parent, expressed in the parent's coordinates (child to parent)
    qRel = qp.getComplexConjugate() * qc;
}



/**
    This method is used to automatically fix the errors in the joints (i.e. drift errors caused by numercial integration). At some future
    point it can be changed into a proper stabilization technique.
*/
void Joint::fix_joint_constraints(bool fixOrientations, bool fixVelocities, bool recursive){
    if (!_child)
        return;

    //if it has a parent, we will assume that the parent's position is correct, and move the children to satisfy the joint constraint
    if (_parent){
        //fix the orientation problems here... hopefully no rigid body is locked (except for the root, which is ok)

        //first fix the relative orientation, if desired
        if (fixOrientations){
            Quaternion qRel;
            compute_relative_orientation(qRel);
            fix_angular_constraint(qRel);
        }

        //now worry about the joint positions

        //compute the vector rc from the child's joint position to the child's center of mass (in world coordinates)
        Vector3d rc = _child->getWorldCoordinates(Vector3d(_child_pos, Point3d(0,0,0)));
        //and the vector rp that represents the same quanity but for the parent
        Vector3d rp = _parent->getWorldCoordinates(Vector3d(_parent_pos, Point3d(0,0,0)));

        //the location of the child's CM is now: pCM - rp + rc
        _child->state.position = _parent->state.position + (rc - rp);

        //fix the velocities, if need be
        if (fixVelocities){
            //to get the relative velocity, we note that the child rotates with wRel about the joint (axis given by wRel
            //d = vector from joint position to CM of the child),
            //but it also rotates together with the parent with the parent's angular velocity,
            //so we need to combine these (all velocities are expressed in world coordinates already) (r is the vector
            //from the CM of the parent, to that of the child).
            Vector3d wRel = _child->state.angular_velocity - _parent->state.angular_velocity;
            Vector3d r = Vector3d(_parent->state.position, _child->state.position);
            Vector3d d = Vector3d(_child->getWorldCoordinates(_child_pos), _child->state.position);
            Vector3d vRel = _parent->state.angular_velocity.crossProductWith(r) + wRel.crossProductWith(d);
            _child->state.velocity = _parent->state.velocity + vRel;
        }
    }

    //make sure that we recursivley fix all the other joint constraints in the articulated figure
    if (recursive)
        for (uint i=0;i<_child->child_joints().size();i++)
            _child->_child_joints[i]->fix_joint_constraints(fixOrientations, fixVelocities, recursive);
}

/**
    This method is used to load the details of a joint from file. The PhysicalWorld parameter points to the world in which the objects
    that need to be linked live in.
*/
void Joint::load_from_file(FILE* f, AbstractRBEngine* world){
    if (f == NULL)
        throwError("Invalid file pointer.");
    if (world == NULL)
        throwError("A valid physical world must be passed in as a parameter");
    //have a temporary buffer used to read the file line by line...
    char buffer[200];
    char tempName[100];

    //this is where it happens.
    while (!feof(f)){
        //get a line from the file...
        fgets(buffer, 200, f);
        if (strlen(buffer)>195)
            throwError("The input file contains a line that is longer than ~200 characters - not allowed");
        char *line = lTrim(buffer);
        int lineType = getRBLineType(line);
        switch (lineType) {
            case RB_NAME:
                _name=std::string(line);
                rmv_white_spaces(_name);
                break;
            case RB_PARENT:
                sscanf(line, "%s", tempName);
                if (_parent != NULL){
                    throwError("This joint already has a parent");
                }
                _parent = world->getARBByName(tempName);
                if (_parent == NULL)
                    throwError("The articulated rigid body \'%s\' cannot be found!", tempName);
                break;
            case RB_CHILD:
                sscanf(line, "%s", tempName);
                if (_child != NULL)
                    throwError("This joint already has a parent");
                _child = world->getARBByName(tempName);
                if (_child == NULL)
                    throwError("The articulated rigid body \'%s\' cannot be found!", tempName);
                break;
            case RB_CPOS:
                _child_pos=Point3d(line);
                break;
            case RB_PPOS:
                _parent_pos=Point3d(line);
                break;
            case RB_END_JOINT:
                //we now have to link together the child and parent bodies
                if (_child == NULL){
                    throwError("A joint has been found that does not have a child rigid body");
                }
                if (_parent == NULL){
                    throwError("A parent has been found that does not have a child rigid body");
                }
                if (_child->parent_joint() != NULL){
                    throwError("The child body \'%s\' already has a parent.", _child->name());
                }
                _parent->_child_joints.push_back(this);
                _child->_parent_joint = this;
                return;//and... done
                break;
            case RB_JOINT_LIMITS:
                read_joint_limits(line);
                break;
            case RB_NOT_IMPORTANT:
                if (strlen(line)!=0 && line[0] != '#')
                    std::cerr<<"Ignoring input line: "<<line<<std::endl;
                break;
            default:
                throwError("Incorrect articulated body input file: \'%s\' - unexpected line.", buffer);
        }
    }
    throwError("Incorrect articulated body input file! No /ArticulatedFigure found");
}

#include <sstream>
std::string Joint::to_xml(int nb_tab)
{
    std::ostringstream oss;
    Point3d p=child_joint_position();
    oss<<"name=\""<<name()<<"\" "<<
         "pos=\""<<-p.x<<" "<<p.z<<" "<<p.y<<"\" ";
    return oss.str();

}

/**
    This method is used to pass in information regarding the rotation axes. The string that is passed in is expected to have
    been read from an input file.
*/
/*
void Joint::readAxes(char* axes){
}
//*/

/**
    This method is used to pass information regarding the joint limits for a joint. The string that is passed in is expected to
    have been read from an input file.
*/
/*
void Joint::readJointLimits(char* limits){
}
//*/



