#include "velocitycontroller.h"

#include <limits>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include "core/Character.h"
#include "core/Trajectory.h"
#include "Physics/rb/RigidBody.h"
#include "core/SimGlobals.h"
#include "Globals.h"
//for trajectory component (need to be moved to another file)
#include "core/SimBiConState.h"

#include "polyfit/polyfit.hpp"

VelocityController::VelocityController(Character* c)
{
    ///init ancien static memebrs
    prev_result_cor=0;
    prev_result_sag=0;
    end_recovery_step_id=0;
    m_is_currently_rotating=false;
    start_rotating_step_id=0;


    limit_switch_sagittal=-1.0;

    character=c;

    _torques.resize(c->getJointCount());

    velD_traj=NULL;
    velD_sagittal_offset=0;
    velD_coronal_right_offset=0;
    velD_coronal_left_offset=0;

    sagittal_comp=NULL;
    coronal_comp=NULL;

    last_virt_force_cumul = Vector3d(0, 0, 0);
    last_virt_force_cumul_signed = Vector3d(0, 0, 0);

    //read some values
    desired_heading=0;

    _need_recovery_steps=false;

    //all those members are specials
    //they are old static variables in functions but I could not find what was the problem with them
    //so I put them here so I'm ensured they are reinitialized between each evaluations ...
    //VelocityController::adapt_learning_curve
    avg_speed_x_previous=std::numeric_limits<double>::quiet_NaN();
    previous_offset_delta=0;
    prev_sag_offset=0;
    prev_prev_sag_offset=0;
    step_count=0;
    count_chained_steps = 0;

    //VelocityController::change_desired_heading
    current_mult=0;
    old_heading=0;
    target_heading=0;
    start_phi=0.0f;

    count_step_force_learn=0;

    first_step=true;

}

VelocityController::~VelocityController()
{
    vec_influence_previous.clear();
}


void VelocityController::init_velD_trajectory(Trajectory *new_traj, double sagittal_offset, double coronal_offset_right,
                                              double coronal_offset_left)
{
    velD_traj=new_traj;
    initial_velD_traj= new Trajectory();
    initial_velD_traj->cpy(velD_traj);

    //*
    //divide by 2 the initial values for some tests
    for( uint i = 0; i < velD_traj->components.size(); ++i ) {
        if (velD_traj->components[i]->rotationAxis!=Vector3d(0,0,1)){
            continue;
        }

        for (int k=0;k<velD_traj->components[i]->baseTraj.getKnotCount();++k){
            velD_traj->components[i]->baseTraj.setKnotValue(k,velD_traj->components[i]->baseTraj.getKnotValue(k)/2);
        }
    }
    //*/

    //this is a hard reset of all the curve, just don't worry about it ...
    //it is needed if you want to learn from zero
    /*
    TrajectoryComponent * affected_component=velD_coronal_component(RIGHT_STANCE);
    for (int i = 0; i < affected_component->baseTraj.getKnotCount(); ++i){
        affected_component->baseTraj.setKnotValue(i, 0);
    }
    affected_component=velD_coronal_component(LEFT_STANCE);
    for (int i = 0; i < affected_component->baseTraj.getKnotCount(); ++i){
        affected_component->baseTraj.setKnotValue(i, 0);
    }
    affected_component=velD_sagittal_component();
    for (int i = 0; i < affected_component->baseTraj.getKnotCount(); ++i){
        affected_component->baseTraj.setKnotValue(i, 0);
    }
    //*/



    velD_sagittal_offset=sagittal_offset;
    velD_coronal_left_offset=coronal_offset_left;
    velD_coronal_right_offset=coronal_offset_right;

    update_components();

    init_new_character_step();
}

void VelocityController::restart()
{
    velD_coronal_left_offset=0;
    velD_coronal_right_offset=0;
    velD_sagittal_offset=0;

    previous_speeds_z.clear();
    previous_speeds_x.clear();

    velD_traj->cpy(initial_velD_traj);

    init_new_character_step();



}

void VelocityController::load_trajectory_from_file()
{

}

void VelocityController::init_new_character_step(){
    //read some values
    change_desired_heading(SimGlobals::desiredHeading, false);


    //update the stance
    stance=character->stance();
    stance_coefficient=(stance == RIGHT_STANCE) ? 1 : -1;

    //update the pointers on the trajectory components
    update_components();

    //read the velocity from the interface
    velD_coronal=SimGlobals::velDCoronal;
    velD_sagittal=SimGlobals::velDSagittal;
    if (first_step){
        first_step=false;
    }

    //hard fixed values for now (need to change it so the user cna control it)
    virt_force_limit= 200; //old 100,0,200
    variation_moy_limit_sagittal = 0.1;
    variation_moy_limit_coronal = 0.1;
    evo_speed_sagittal=0.08;
    evo_speed_coronal=0.08;


    //reset values used in the internal computation
    vel_sagittal.clear();
    vel_coronal.clear();

    last_phi=0;
    times_vel_sampled=0;
    cumul_speed_z = 0;
    cumul_speed_x = 0;

    cur_phi_limit_z = sagittal_comp->baseTraj.getMinPosition();
    cur_knot_nbr_z = 0;

    cur_phi_limit_x = coronal_comp->baseTraj.getMinPosition();
    cur_knot_nbr_x = 0;

    last_virt_force_cumul=Vector3d(0,0,0);
    last_virt_force_cumul_signed=Vector3d(0,0,0);
    /*
    std::ostringstream oss;
    oss<<std::showpos <<std::fixed<< std::setprecision(8)<<
         "new step:"<<velD_coronal_left_offset <<" "<<velD_coronal_right_offset <<" "<<
         velD_sagittal_offset;
    std::cout<<oss.str();
    //*/
}

void VelocityController::store_velocity(double cur_phi, Vector3d v)
{

    change_desired_heading(SimGlobals::desiredHeading,true);

    //convert v to the current orientation
    v=Quaternion::getRotationQuaternion(desired_heading,SimGlobals::up).to_local_coordinate(v);

    //store the current speed to be able to know the avg speed at the end
    times_vel_sampled++;
    cumul_speed_z += v.z;
    cumul_speed_x += v.x;
    last_phi=cur_phi;


    if (cur_knot_nbr_z != -1 && (cur_phi+0.00001) > cur_phi_limit_z){

        /*
        static double phi_error=0;
        if (cur_knot_nbr_z==0){
            std::ostringstream oss;
            oss<<"phi_error_z: "<<phi_error;
            std::cout<<oss.str();
            phi_error=0;
        }
        phi_error+=std::abs(sagittal_comp->baseTraj.getKnotPosition(cur_knot_nbr_z)-cur_phi);
        //*/
        vel_sagittal.push_back(v.z-velD_sagittal);
        //we save the speed for this point


        //and we look at the next phi
        //if there are no next pt simply set the nbr of the next knot to -1
        cur_knot_nbr_z++;

        if (cur_knot_nbr_z < sagittal_comp->baseTraj.getKnotCount()){
            cur_phi_limit_z = sagittal_comp->baseTraj.getKnotPosition(cur_knot_nbr_z);
        }
        else{

            cur_knot_nbr_z = -1;
            cur_phi_limit_z = std::numeric_limits<double>::infinity();
        }

    }


    if (cur_knot_nbr_x != -1 && (cur_phi+0.00001) > cur_phi_limit_x){

        /*
        static double phi_error=0;
        if (cur_knot_nbr_x==0){
            std::ostringstream oss;
            oss<<"phi_error_x: "<<phi_error;
            std::cout<<oss.str();
            phi_error=0;
        }
        phi_error+=std::abs(coronal_comp->baseTraj.getKnotPosition(cur_knot_nbr_x)-cur_phi);
        //*/


        //we save the speed for this point
        vel_coronal.push_back(v.x*stance_coefficient -velD_coronal);

        //and we look at the next phi
        //if there are no next pt simply set the nbr of the next knot to -1
        cur_knot_nbr_x++;

        if (cur_knot_nbr_x < coronal_comp->baseTraj.getKnotCount()){
            cur_phi_limit_x = coronal_comp->baseTraj.getKnotPosition(cur_knot_nbr_x);
        }
        else{
            cur_knot_nbr_x = -1;
            cur_phi_limit_x = std::numeric_limits<double>::infinity();
        }
    }
}



#include <iostream>
void VelocityController::adapt_learning_curve(Vector3d v, double phi_last_step){


    step_count++;

    //just a quick check but we need to actualy have sampled the velocity some time to actualy do that
    if (times_vel_sampled<5){
        return;
    }

    last_phi=phi_last_step;

    //convert v to the current orientation
    v=Quaternion::getRotationQuaternion(desired_heading,SimGlobals::up).to_local_coordinate(v);


    bool recovery_step_asked_z = false;
    bool recovery_step_asked_x = false;

    //finish the computation for the avg virt force applied during the step
    //*
    previous_virt_force_signed=last_virt_force_signed;
    last_virt_force.x = last_virt_force_cumul.x / times_vel_sampled;
    last_virt_force.z = last_virt_force_cumul.z / times_vel_sampled;
    last_virt_force_cumul = Vector3d(0, 0, 0);
    last_virt_force_signed.x = last_virt_force_cumul_signed.x / times_vel_sampled;
    last_virt_force_signed.z = last_virt_force_cumul_signed.z / times_vel_sampled;
    last_virt_force_cumul_signed = Vector3d(0, 0, 0);
    //*/

    //I finish the computation of the avg speeds
    double avg_speed_z,avg_speed_x;
    avg_speed_z = cumul_speed_z/ times_vel_sampled;
    avg_speed_x = cumul_speed_x/ times_vel_sampled;

    //I make sure that there are no 0 values
    if (std::abs(avg_speed_z)<=0.0000001){
        avg_speed_z=0.0000001;
    }
    if (std::abs(avg_speed_x)<=0.0000001){
        avg_speed_x=0.0000001;
    }

    bool offset_sag_limit_reached=false;
    bool offset_cor_limit_reached=false;

    virtual_force_near_limit(offset_cor_limit_reached,offset_sag_limit_reached,0.95);


    previous_speeds_z.push_back(avg_speed_z);

    double offset_delta=0;
    double avg_signed_variation_sag=0;
    double avg_signed_variation_cor=0;

    //we start with the sagittal trajectory
    double sup_point_val;
    double reset_value;
    sup_point_val=v.z-velD_sagittal;
    reset_value=0;


    prev_result_sag=adapt_learning_component(sagittal_comp,vel_sagittal,sup_point_val,variation_moy_limit_sagittal,
                                             recovery_step_asked_z,avg_signed_variation_sag,0.50,reset_value,
                                             prev_result_sag,reset_value);

    if (!recovery_step_asked_z&&!is_currently_rotating()){

        //now we handle the variation of the offset
        offset_delta = velD_sagittal - previous_speeds_z.back();
        //we square it but keep the sign
        //offset_delta*=std::abs(offset_delta);



        offset_delta*=evo_speed_sagittal;



        //and we finaly affect the delta to the offset
        prev_prev_sag_offset=prev_sag_offset;
        prev_sag_offset=velD_sagittal_offset;
        previous_offset_z.push_back(velD_sagittal_offset);

        //std::cout<<offset_delta<<std::endl;

        //when the virtual force is already at the limit we check that we are not
        //pushing the offset even further since it would be useless and would make
        //going back to opposite evolution of the virtual force even harder
        if (!offset_sag_limit_reached||((last_virt_force_signed.z*offset_delta)<=0.0)){
            velD_sagittal_offset += offset_delta*0.7+0.1*previous_offset_delta;
            previous_offset_delta=offset_delta;
        }

    }


    //now I'll add a protection to prevent getting stupidely high independent values in the curve
    //I'll limit the delta to the velD to 2 times the absolute val of the velD
    double val_limit_sup = 3;
    double val_limit_inf = -1;
    for (int i = 0; i < sagittal_comp->baseTraj.getKnotCount(); ++i){
        double cur_val = sagittal_comp->baseTraj.getKnotValue(i);
        if (cur_val>val_limit_sup){
            sagittal_comp->baseTraj.setKnotValue(i, val_limit_sup);
        }
        else if (cur_val<val_limit_inf){
            sagittal_comp->baseTraj.setKnotValue(i, val_limit_inf);
        }
    }

    //we are finished with the sagittal trajectory

    //now I need to handle the coronal trajectory
    //skip the first step cause I don't have a real speed during this step
    int nbr_step_wait=2;
    //if (!std::isnan(avg_speed_x_previous)){
    if (step_count>nbr_step_wait){
        double stance_coef=1;
        if (stance!=RIGHT_STANCE){
            stance_coef=-1;
        }

        previous_speeds_x.push_back((avg_speed_x+ avg_speed_x_previous) / 2);

        //on the first iteration we had multiples copies of the value so that the vector has the same size as the sagitel speed one
        if (step_count==(nbr_step_wait+1)){
            for (int i=0;i<nbr_step_wait;++i){
                previous_speeds_x.push_back(previous_speeds_x[0]);
            }
        }

        prev_result_cor=adapt_learning_component(coronal_comp,vel_coronal,(v.x*stance_coef)-velD_coronal,variation_moy_limit_coronal,
                                                 recovery_step_asked_x,avg_signed_variation_cor,0.5,0,
                                                 prev_result_cor,0);

        //*
        if (!recovery_step_asked_x&&!is_currently_rotating()){
            //now we need to translate the curve depending on the speed it result in and the speed we want
            //evo_speed_coronal = 0.2;// +(velDSagittal / 0.7) / 2;

            //I'l simply divide by the ratio between the avg_speed and the velD
            offset_delta = (velD_coronal - previous_speeds_x.back());



            //I need to break the inertia effect that cause occilasion around the desired speed
            //I'll store the speed that I had at the previous steps to adapt the evolution of the factor on them
            int nbr_speeds = (int)previous_speeds_x.size();
            if (nbr_speeds > 1){
                double prev_delta = velD_coronal - previous_speeds_x[nbr_speeds - 2];
                if (std::signbit(prev_delta) == std::signbit(offset_delta)){
                    if (std::abs(prev_delta) < std::abs(offset_delta)){
                        offset_delta *= 2;
                    }
                    else{
                        if (nbr_speeds > 2){
                            double prev2_delta = velD_coronal - previous_speeds_x[nbr_speeds - 3];
                            if (std::signbit(prev2_delta) == std::signbit(prev_delta)){
                                if (std::abs(prev2_delta) < std::abs(prev_delta)){
                                    //offset_delta /= 2;
                                }
                            }
                        }
                    }
                }
            }

            offset_delta *= evo_speed_coronal;

            //I'll limit the possible translation
            double cor_delta_limit = 0;
            for (int i = 0; i < (int)vel_coronal.size(); ++i){
                double val = std::abs(coronal_comp->baseTraj.getKnotValue(i));
                if (val > cor_delta_limit){
                    cor_delta_limit = val;
                }
            }
            cor_delta_limit /= 10;


            if (offset_delta>cor_delta_limit){
                offset_delta = cor_delta_limit;
            }
            else if (offset_delta < -cor_delta_limit){
                offset_delta = -cor_delta_limit;
            }


            if (stance == RIGHT_STANCE){
                velD_coronal_right_offset += offset_delta;
                velD_coronal_left_offset -= 0.5*offset_delta;
            }
            else{
                velD_coronal_left_offset -= offset_delta;
                velD_coronal_right_offset += 0.5*offset_delta;
            }
            //*/

            //now I'll do the degradation if it's possible
            //the principle is that if the 2 factors have hte same sign and that we have a balanced state
            //then we are applying useless forces (at least force bigger than necessary)
            //if it happens I degrade the 2 facotrs by 10% of the biggest (or 50% of the smallest if the biggest*0.1>smallest
            //I limit it so that it cannot change the sign of one of the factors
            if (previous_speeds_x.size()>1){
                double speed_delta=velD_coronal-previous_speeds_x.back();
                double prev_delta=velD_coronal-previous_speeds_x[previous_speeds_x.size()-2];

                if (speed_delta < 0.001 &&prev_delta<0.001){ //meaning we are in a pretty stable state
                    if (std::signbit(velD_coronal_left_offset) == std::signbit(velD_coronal_right_offset)){
                        double abs_left = std::abs(velD_coronal_left_offset);
                        double abs_right = std::abs(velD_coronal_right_offset);
                        double degradation_val;

                        if (abs_left < abs_right){
                            degradation_val = velD_coronal_right_offset*0.1;
                            if (std::abs(degradation_val) > std::abs(velD_coronal_left_offset)){
                                degradation_val = velD_coronal_left_offset / 2;
                            }
                        }
                        else{
                            degradation_val = velD_coronal_left_offset*0.1;
                            if (std::abs(degradation_val) > std::abs(velD_coronal_right_offset)){
                                degradation_val = velD_coronal_right_offset/ 2;
                            }
                        }

                        velD_coronal_left_offset -= degradation_val;
                        velD_coronal_right_offset -= degradation_val;

                    }
                }
            }
        }
    }

    //*/

    //now finish to ckeck if we need a recovery step
    if (recovery_step_asked_z||recovery_step_asked_x){
        _need_recovery_steps = true;
    }
    else{
        if (_need_recovery_steps){
            end_recovery_step_id=step_count;
            _need_recovery_steps = false;
        }

        if ((step_count-start_rotating_step_id)>1&&(step_count-end_recovery_step_id)>3&&is_currently_rotating()){
            m_is_currently_rotating=false;
            std::cout<<"finished rotating"<<std::endl;
        }

    }


    //we save the coronal avg speed for use in the next step
    avg_speed_x_previous=avg_speed_x;
    /*
    {
        static int count=0;
        std::ostringstream oss;
        oss<<count<<" "<<avg_signed_variation_cor<<" "<<avg_signed_variation_sag<<std::endl;
        write_to_report_file(oss.str());
        count++;
    }//*/
}

void VelocityController::apply_virtual_force(const Vector3d& v)
{
    std::vector<Joint *> vec_hips;
    std::vector<double> vec_influence;
    character->get_leg_contact_influence(vec_hips,vec_influence);


    if (vec_influence_previous.size()<1){
        vec_influence_previous=vec_influence;
    }

    for (int i=0;i<vec_influence.size();++i){
        double old_val=vec_influence_previous[i];
        vec_influence_previous[i]=vec_influence[i];

        vec_influence[i]+=old_val;
        vec_influence[i]/=2;
    }

    virtual_force=compute_virtual_force(v);

    for (int i=0; i< _torques.size();++i){
        _torques[i]=Vector3d(0,0,0);
    }

    if (SimGlobals::foot_flat_on_ground){
        for (int i=0;i<vec_hips.size();++i){
            if (vec_influence[i]>0.00001){
                compute_leg_torques(vec_hips[i],vec_influence[i]);
            }
        }
    }


    //store it for external usage
    /*
    last_virt_force_cumul.x += std::abs(virtual_force.x);
    last_virt_force_cumul.z += std::abs(virtual_force.z);
    last_virt_force_cumul_signed.x += virtual_force.x;
    last_virt_force_cumul_signed.z += virtual_force.z;
    //*/

    //*
    //this can be used to show the forces
    ForceStruct show_force;
    show_force.F = virtual_force;
    show_force.pt = character->getCOM();
    SimGlobals::vect_forces.push_back(show_force);
    //*/

    /*
    //this can be used to show the forces
    ForceStruct show_force2;
    show_force2.F = last_virt_force_signed;
    show_force2.pt = character->getCOM();
    SimGlobals::vect_forces.push_back(show_force2);
    //*/


}


void VelocityController::virtual_force_near_limit(bool &coronal, bool &sagittal, double limit){
    if (last_virt_force_signed.length() > virt_force_limit *limit){
        //now I need to detect wich component caused us to be near the limit
        double abs_cos=std::abs(last_virt_force.x);
        double abs_sin=std::abs(last_virt_force.z);

        sagittal = false;
        coronal = false;
        if (abs_cos>0.5){coronal=true;}
        if (abs_sin>0.5){sagittal=true;}

    }
    /*
    if (std::abs(last_virt_force.x) > virt_force_limit.x* limit){
        coronal = true;
    }
    else{
        coronal = false;
    }

    if (std::abs(last_virt_force.z) > virt_force_limit.z* limit){
        sagittal = true;
    }
    else{
        sagittal = false;
    }
    //*/
}

void VelocityController::has_evolved_during_last_step(bool &coronal, bool &sagittal, double limit){
    sagittal=false;
    coronal=false;
    if (last_virt_force_signed.z<previous_virt_force_signed.z-20||
            previous_virt_force_signed.z+20<last_virt_force_signed.z){
        sagittal=true;
    }

    //I don't handle the coronal axis yet (since I don't neeed it and it would be a lot of work
}


void VelocityController::lower_force_intensity(double coef){
    velD_sagittal_offset*=(1-coef);
    velD_coronal_right_offset*=(1-coef);
    velD_coronal_left_offset*=(1-coef);
}


void VelocityController::update_components()
{
    sagittal_comp=velD_sagittal_component();
    coronal_comp=velD_coronal_component(stance);
}

int VelocityController::adapt_learning_component(TrajectoryComponent *affected_component, std::vector<double> &vel_vector,
                                                 double sup_point_val, double variation_moy_limit, bool& recovery_step_asked,
                                                 double& avg_signed_variation, double variation_ratio_used, double reset_value,
                                                 int previous_result, double zero_value, bool force_learning)
{
    if (vel_vector.size()>50){
        vel_vector.pop_back();
    }


    int nbr_values_original = vel_vector.size();

    /*
    //specialbloc for an imageforthe paper
    for (int i = 0; i < nbr_values_original; ++i){
        //we affect the variation
        double new_val = vel_vector[i]+0.2;
        affected_component->baseTraj.setKnotValue(i, new_val);
    }
    int remove_count=affected_component->baseTraj.getKnotCount()-nbr_values_original;
    for (int i = 0; i < remove_count; ++i){
        //we affect the variation
        affected_component->baseTraj.removeKnot(nbr_values_original);
    }
    return 0;
    //*/

    //first I need to center the values on their "0" value
    //instead of doing it relative to a zero value
    //I'll do it relative to each other.
    //Meaning I have to relocate the curves to each other.
    //I can't use avg because I don't want any perturbation to have a large
    //impact on recaling the two curves. (But anyway the on prturbed steps the learning
    //is disactivated so it might not be that important...
    //so My idea is to use the median instead of the avg. this way outlier will not have a big impact;
    std::vector<double> vect_err;
    vect_err.resize(nbr_values_original);
    for (int i = 0; i < nbr_values_original; ++i){
        vect_err[i]=vel_vector[i] - affected_component->baseTraj.getKnotValue(i);
    }

    std::sort(vect_err.begin(),vect_err.end());
    //set the offset to the median value
    double offset=vect_err[static_cast<int>(nbr_values_original/2)];

    //and not simply translate the current step values
    for (int i = 0; i < nbr_values_original; ++i){
        vel_vector[i]-=offset;
    }

    //so first I need to complete the curve of velocity if it is needed
    bool need_sup_point = static_cast<int>(vel_vector.size()) < affected_component->baseTraj.getKnotCount();
    if (need_sup_point){
        //compute the polynom corresponding to the registered points
        int ref_points_count=5;
        int poly_dim=1;
        std::vector<double> vect_x,vect_y;
        for (int i=0;i<ref_points_count;++i){
            vect_x.push_back(affected_component->baseTraj.getKnotPosition((int)vel_vector.size()-1-i));
            vect_y.push_back(vel_vector[vel_vector.size()-1-i]);
        }
        std::vector<double> coefs_sup_points=polyfit<double>(vect_x,vect_y,poly_dim);

        //and now we can generate the points that are missing
        std::vector<double> missing_points;
        for (int i=(int)vel_vector.size();i<affected_component->baseTraj.getKnotCount();++i){
            missing_points.push_back(affected_component->baseTraj.getKnotPosition(i));
        }
        std::vector<double> vect_sup_points=polyval<double>(coefs_sup_points,missing_points);

        //and now store them with the original points for the computations
        for (int i=0;i<vect_sup_points.size();++i){
            vel_vector.push_back(vect_sup_points[i]);
        }

        if (vel_vector.size()!=affected_component->baseTraj.getKnotCount()){
            throw("VelocityController::adapt_learning_component number of points after sup points generation different than number of reference points");
        }
    }

    /*
    //specialbloc for an imageforthe paper
    for (int i = 0; i < vel_vector.size(); ++i){
        //we affect the variation
        double new_val = vel_vector[i];
        affected_component->baseTraj.setKnotValue(i, new_val);
    }
    return 0;
    //*/

    //So we calculate the moy variation
    double variation_moy = 0;
    double variation_max = 0;
    double variation_moy_signed = 0;
    std::vector<double> variation_vector;
    for (int i = 0; i < (int)vel_vector.size(); ++i){
        variation_vector.push_back(vel_vector[i] - affected_component->baseTraj.getKnotValue(i));
    }

    //when computing the avg we do not cnsider the sup points
    //since they may not be stabe it the stable do not have exactly the same duration
    variation_max=std::abs(variation_vector[0]);
    for (int i = 0; i < nbr_values_original; ++i){
        variation_max=std::max(variation_max,std::abs(variation_vector[i]));
        variation_moy += std::abs(variation_vector[i]);
        variation_moy_signed += variation_vector[i];
    }

    variation_moy /= nbr_values_original;
    variation_moy_signed /= nbr_values_original;

    avg_signed_variation=variation_moy;

    //we check if we can evolve the curve or if we need to use a recovery step
    if (variation_moy < variation_moy_limit || force_learning){
        //we handle the points stored in the buffer
        for (int i = 0; i < nbr_values_original; ++i){
            //we limit the variation that we will affect to half (or as parametred) of the observed one
            variation_vector[i] *= variation_ratio_used;

            //we affect the variation
            double new_val = affected_component->baseTraj.getKnotValue(i) + variation_vector[i];
            affected_component->baseTraj.setKnotValue(i, new_val);
        }

        //here is to hande the sup points
        double variation_moy_extend=0;
        for (int i = nbr_values_original; i < (int)vel_vector.size(); ++i){
            variation_moy_extend += std::abs(variation_vector[i]);
        }
        variation_moy_extend/=(vel_vector.size()-nbr_values_original);

        for (int i = nbr_values_original; i < (int)vel_vector.size(); ++i){
            //we prevent that the variation we are gonna use is based on a value superior to the moy variation
            if (std::abs(variation_vector[i]) > variation_moy_extend){
                variation_vector[i] = variation_moy_extend*variation_vector[i] / std::abs(variation_vector[i]);
            }
            //we limit the variation that we will affect to half (or as parametred) of the observed one
            variation_vector[i] *= variation_ratio_used;

            //we affect the variation
            double new_val = affected_component->baseTraj.getKnotValue(i) + variation_vector[i];
            affected_component->baseTraj.setKnotValue(i, new_val);
        }

    }
    else{
        //I do smth in the case of continuous recovery steps
        if (_need_recovery_steps){
            count_chained_steps++;
        }
        else{
            count_chained_steps = 0;
        }

        if (count_chained_steps > 5){
            //if we have more than 5 recovery steps we reset the curve
            //if it is the case I reset the whole cursve to 1
            for (int i = 0; i < affected_component->baseTraj.getKnotCount(); ++i){
                affected_component->baseTraj.setKnotValue(i, reset_value);
            }
        }

        //this should mean the caracter is currenlty in an unstable state
        recovery_step_asked = true;
    }



    return 0;
}

inline TrajectoryComponent *VelocityController::velD_sagittal_component() {
    return velD_traj->components[2];
}

inline TrajectoryComponent *VelocityController::velD_coronal_component(int stance)
{
    if (stance == RIGHT_STANCE){
        return velD_traj->components[0];
    }
    else{
        return velD_traj->components[1];
    }
}


/**
this function get the desired sagital velocity (affected by the variation trajectory)
*/
inline double VelocityController::get_effective_desired_sagittal_velocity(double phi){
    return velD_sagittal +(sagittal_comp->baseTraj.evaluate_catmull_rom(phi)+velD_sagittal_offset);
}

/**
this function get the desired coronal velocity (affected by the variation trajectory)
*/
inline double VelocityController::get_effective_desired_coronal_velocity(double phi){
    double signChange;
    double add_factor;

    if (stance == RIGHT_STANCE){
        signChange = 1;
        add_factor = velD_coronal_right_offset;
    }
    else{
        signChange = -1;
        add_factor = velD_coronal_left_offset;
    }

    return velD_coronal +(coronal_comp->baseTraj.evaluate_catmull_rom(phi)+add_factor)*signChange;
    //return velDCoronal;
}

Vector3d VelocityController::compute_virtual_force(const Vector3d &v)
{

    Quaternion qheading= Quaternion::getRotationQuaternion(desired_heading, SimGlobals::up);

    //this is the desired acceleration of the center of mass
    Vector3d desA = Vector3d();

    Vector3d vh=qheading.to_local_coordinate(v);
    vh.y=0;

    double eff_velDSagittal = get_effective_desired_sagittal_velocity(last_phi);
    double eff_velDCoronal = get_effective_desired_coronal_velocity(last_phi);

    desA.z = (eff_velDSagittal - vh.z) *30;
    desA.x = (eff_velDCoronal - vh.x) * 20;



    /*
    ForceStruct show_force3;
    //this can be used to show the forces
    show_force3.F = Vector3d(eff_velDCoronal,0,eff_velDSagittal);
    show_force3.F = show_force3.F*50;
    show_force3.pt = character->getCOM();
    SimGlobals::vect_forces.push_back(show_force3);
    //*/

    /*/*
    //this can be used to show the forces
    show_force3.F = vh;
    show_force3.F = show_force3.F*50;
    show_force3.pt = character->getCOM();
    SimGlobals::vect_forces.push_back(show_force3);
    //*/

    if (stance == 1){
        //Vector3d errV = characterFrame.inverseRotate(doubleStanceCOMError*-1);
        //desA.x = (-errV.x + comOffsetCoronal) * 20 + (velDCoronal - v.x) * 9;
        //desA.z = (-errV.z + comOffsetSagittal) * 10 + (velDSagittal - v.z) * 150;
    }

    //and this is the force that would achieve that - make sure it's not too large...
    Vector3d fA = (desA)* character->getAF()->mass();

    /*
    std::ostringstream oss;
    oss<<"eff_vel_D_sag: "<<fA.x<<"  "<<fA.z;
    std::cout<<oss.str();
    //*/

    //boundToRange(&fA.x, -virt_force_limit.x, virt_force_limit.x);
    //    boundToRange(&fA.z, -virt_force_limit.z, virt_force_limit.z);
    if (fA.length()>virt_force_limit){
        fA=fA.unit()*virt_force_limit;
    }

    //*
    last_virt_force_cumul.x += std::abs(fA.x);
    last_virt_force_cumul.z += std::abs(fA.z);
    last_virt_force_cumul_signed.x += fA.x;
    last_virt_force_cumul_signed.z += fA.z;
    //*/
    /*
    {
        std::ostringstream oss;
        oss<< fA.length();
        write_to_report_file(oss.str());
    }//*/

    //now change this quantity to world coordinates...
    fA=qheading.to_world_coordinate(fA);
    fA = fA*SimGlobals::virtual_force_effectiveness;

    return fA;
}

void VelocityController::compute_leg_torques(Joint *hip, double leg_ratio)
{

    //applying a force at the COM induces the force f. The equivalent torques are given by the J' * f, where J' is
    // dp/dq, where p is the COM.
    Joint* pelvis_torso= character->getJointByName("pelvis_torso");
    Joint* knee = hip->child()->child_joints()[0];
    Joint* ankle= knee->child()->child_joints()[0];
    int id=pelvis_torso->idx();
    id=hip->idx();
    id=knee->idx();
    id=ankle->idx();

    ArticulatedRigidBody* tibia = ankle->parent();
    ArticulatedRigidBody* femur = knee->parent();
    ArticulatedRigidBody* pelvis = hip->parent();
    ArticulatedRigidBody* back = pelvis_torso->child();

    Point3d anklePos = ankle->child()->getWorldCoordinates(ankle->child_joint_position());
    Point3d kneePos = knee->child()->getWorldCoordinates(knee->child_joint_position());
    Point3d hipPos = hip->child()->getWorldCoordinates(hip->child_joint_position());
    Point3d backPos = pelvis_torso->child()->getWorldCoordinates(pelvis_torso->child_joint_position());

    //total mass...
    double m = tibia->getMass() + femur->getMass() + pelvis->getMass() + back->getMass();

    Vector3d fA = virtual_force;



    Vector3d f1 = Vector3d(anklePos, tibia->getCMPosition()) * tibia->getMass() +
            Vector3d(anklePos, femur->getCMPosition()) * femur->getMass() +
            Vector3d(anklePos, pelvis->getCMPosition()) * pelvis->getMass();
    f1 /= m;

    Vector3d f2 = Vector3d(kneePos, femur->getCMPosition()) * femur->getMass() +
            Vector3d(kneePos, pelvis->getCMPosition()) * pelvis->getMass();
    f2 /= m;

    Vector3d f3 = Vector3d(hipPos, pelvis->getCMPosition()) * pelvis->getMass();
    f3 /= m;

    Vector3d f4 = Vector3d(backPos, back->getCMPosition()) * back->getMass();
    f4 /= m;


    //now compute the torque
    Vector3d torque = f1.crossProductWith(fA)*leg_ratio;
    preprocess_ankle_virtual_torque(ankle, &torque);
    torque.y = 0;
    _torques[ankle->idx()] += torque;

    torque = f2.crossProductWith(fA)*leg_ratio;
    torque.y = 0;
    _torques[knee->idx()] += torque;

    torque = f3.crossProductWith(fA)*leg_ratio;
    torque.y = 0;
    _torques[hip->idx()] += torque;

    torque= f4.crossProductWith(fA)*leg_ratio;
    torque.y = 0;
    _torques[pelvis_torso->idx()] -= torque;

}

void VelocityController::preprocess_ankle_virtual_torque(Joint* ankle, Vector3d *ankleVTorque)
{
    ForceStruct heel_force,front_foot_force, toe_force;
    ArticulatedRigidBody* foot = ankle->child();
    character->get_force_info_on_foot(foot, heel_force, front_foot_force, toe_force);
    *ankleVTorque = foot->getLocalCoordinates(*ankleVTorque);

    if (front_foot_force.F.isZeroVector() || last_phi < 0.2 || last_phi > 0.8){
        ankleVTorque->x = 0;
    }

    Vector3d footRelativeAngularVel = foot->getLocalCoordinates(foot->get_angular_velocity_avg());
    if ((footRelativeAngularVel.z < -0.5 && ankleVTorque->z > 0) || (footRelativeAngularVel.z > 0.5 && ankleVTorque->z < 0)){
        ankleVTorque->z = 0;
    }

    if (fabs(footRelativeAngularVel.z) > 1.0){
        ankleVTorque->z = 0;
    }

    if (fabs(footRelativeAngularVel.x) > 1.0){
        ankleVTorque->x = 0;
    }

    if ((!heel_force.F.isZeroVector())&&(front_foot_force.F.isZeroVector())){
        ankleVTorque->x = 0;
        ankleVTorque->z = 0;
    }

    boundToRange(&ankleVTorque->z, -20, 20);

    *ankleVTorque = foot->getWorldCoordinates(*ankleVTorque);
}

void VelocityController::change_desired_heading(double new_heading, bool update_in_step_velocity)
{
    //if it's the same heading, do nothing
    if (desired_heading==new_heading){
        return;
    }

    if (previous_speeds_x.size()!=previous_speeds_z.size()){
        throw("VelocityController::change_desired_heading trying to switch orientation before "
        "the end of the initialisation fo the speeds is impossible");
    }

    m_is_currently_rotating=true;
    start_rotating_step_id=step_count+1;

    Quaternion q=Quaternion::getRotationQuaternion(new_heading,SimGlobals::up);
    Quaternion old_q=Quaternion::getRotationQuaternion(desired_heading,SimGlobals::up);

    if (update_in_step_velocity){
        Vector3d cur_vel=Vector3d(0,0,0);
        for (int i=0;i<vel_sagittal.size();++i){
            cur_vel.x=vel_coronal[i];
            cur_vel.z=vel_sagittal[i];

            //transform from old to new coordinate
            old_q.to_world_coordinate(cur_vel);
            q.to_local_coordinate(cur_vel);

            vel_coronal[i]=cur_vel.x;
            vel_sagittal[i]=cur_vel.z;
        }

    }

    //now the cumul speed
    Vector3d cumul_vel=Vector3d(0,0,0);
    cumul_vel.x=cumul_speed_x;
    cumul_vel.z=cumul_speed_z;

    //transform from old to new coordinate
    old_q.to_world_coordinate(cumul_vel);
    q.to_local_coordinate(cumul_vel);

    cumul_speed_x=cumul_vel.x;
    cumul_speed_z=cumul_vel.z;

    //now the previous steps speed
    Vector3d prev_vel=Vector3d(0,0,0);
    for (int i=0;i<previous_speeds_z.size();++i){
        prev_vel.x=previous_speeds_x[i];
        prev_vel.z=previous_speeds_z[i];

        //transform from old to new coordinate
        old_q.to_world_coordinate(prev_vel);
        q.to_local_coordinate(prev_vel);

        previous_speeds_x[i]=prev_vel.x;
        previous_speeds_z[i]=prev_vel.z;
    }


    //set to the new heading
    desired_heading=new_heading;
}




