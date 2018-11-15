#include "stancefootcontactcontroller.h"

#include <iostream>
#include <fstream>
#include <sstream>

#include "estimationworld.h"
#include "Core/Character.h"
#include "Core/pose_control/posecontroller.h"
#include "Physics/ODEWorld.h"
#include "Physics/rb/ArticulatedRigidBody.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include "Utils/Utils.h"


#include <thread>
#include <cstdio>
#define SHARK_USE_OPENMP
#include <shark/Core/OpenMP.h>
#include "Globals.h"

// Implementation of the CMA-ES
#include <shark/Algorithms/DirectSearch/CMA.h>
//acces to sphere
//#include <shark/ObjectiveFunctions/Benchmarks/Sphere.h>

StanceFootContactController::StanceFootContactController(Character *c)
{
    m_program_end=false;
    body_config=Globals::foot_contact_controller_config;
    print_results=false;
    save_results=false;

    character=c;

    /* initialize random seed: */
    srand (time(NULL));

    std::remove("eval_estimate/simulation_estimation.csv");
    std::remove("eval_estimate/test.csv");

    init_estimation_world();

    phi=0;
    m_result_torques.resize(3,Vector3d(0,0,0));

    count_cma_success=0;
    count_cma_usage=0;
    count_cma_evaluation=0;
    count_cma_success_global=0;
    count_cma_usage_global=0;
    count_cma_evaluation_global=0;
    count_sim_step=0;
    count_sim_step_with_problem=0;

}

StanceFootContactController::~StanceFootContactController()
{
    m_program_end=true;

    /*
    std::vector<EstimationWorld*> m_worlds; del
    std::vector<QMutex*> m_worlds_mutex; del
    QSemaphore* semaphore; del

    //struct to reuse threads
    std::vector<std::thread*> m_reusable_thread; del
    std::vector<QSemaphore*> m_semaphore_execute_thread; del
    std::vector<QSemaphore*> m_semaphore_end; del
     //*/

    for (int i=0;i<m_worlds.size();++i){
        if (m_reusable_thread[i]!=NULL){
            m_semaphore_execute_thread[i]->release();
            m_reusable_thread[i]->join();
            delete m_reusable_thread[i];
            m_reusable_thread[i]=NULL;
        }

        delete m_semaphore_execute_thread[i];
        m_semaphore_execute_thread[i]=NULL;

        delete m_semaphore_end[i];
        m_semaphore_end[i]=NULL;

        delete m_worlds_mutex[i];
        m_worlds_mutex[i]=NULL;

        delete m_worlds[i];
        m_worlds[i]=NULL;
    }
    delete semaphore;
    semaphore=NULL;

    m_reusable_thread.clear();
    m_semaphore_execute_thread.clear();
    m_semaphore_end.clear();
    m_worlds_mutex.clear();
    m_worlds.clear();


    m_result_torques.clear();
    m_contact_forces.clear();
    m_delta_torques.clear();


    SimGlobals::stanceFootWorld=NULL;
}


void StanceFootContactController::init_estimation_world()
{
    if (body_config>=0){

        for (int i=0;i<1;++i){
            EstimationWorld* w=new EstimationWorld(character,std::max(body_config,0));
            m_worlds.push_back(w);
            m_worlds_mutex.push_back(new QMutex());


            m_reusable_thread.push_back(NULL);

            m_semaphore_execute_thread.push_back(new QSemaphore(1));
            m_semaphore_end.push_back(new QSemaphore(0));

        }
        semaphore=new QSemaphore(m_worlds.size());
        //    semaphore=new QSemaphore(4);

    }
    init_new_character_step();
}

void StanceFootContactController::init_new_character_step()
{
    if (body_config>=0){
        m_current_position=0;

        //*
        for (int i=0;i<m_worlds.size();++i){
            m_worlds[i]->init_new_character_step();
        }

        SimGlobals::stanceFootWorld=m_worlds.front()->get_stance_world();
        //*/

        //reset the result torques
        m_result_torques.clear();
        m_result_torques.resize(3,Vector3d(0,0,0));

        m_countdown_switch_to_single_stance=5;
    }


    count_cma_usage_global+=count_cma_usage;
    count_cma_success_global+=count_cma_success;
    count_cma_evaluation_global+=count_cma_evaluation;
    std::cout<<"global results: "<<count_cma_usage_global<<"  "<<count_cma_success_global<<"   "<<
               ((double)count_cma_success_global)*100/count_cma_usage_global<<"   "<<
               ((double)count_cma_evaluation_global)/((double)count_cma_success_global)<<
               std::endl;
    std::cout<<"curren results: "<<count_cma_usage<<"  "<<count_cma_success<<"   "<<
               ((double)count_cma_success)*100/count_cma_usage<<"   "<<
               ((double)count_cma_evaluation)/((double)count_cma_success)<<
               std::endl;
    std::cout<<"real_world_eval: "<<count_sim_step<<"  "<<count_sim_step_with_problem<<"   "<<
               ((double)count_sim_step_with_problem)*100/((double)count_sim_step)<<
               std::endl;
    count_cma_usage=0;
    count_cma_evaluation=0;
    count_cma_success=0;

}

void StanceFootContactController::preprocess_simulation_step(double phase)
{
    phi=phase;


}

void StanceFootContactController::simulation_step(std::vector<Vector3d> torques)
{

    if (save_results&&!m_contact_forces.empty()){
        //*
        std::vector<Vector3d> f=m_contact_forces;
        //*
        const Vector3d* f2;
        Vector3d f2_toes;
        if (phi!=0.0){
            f2=character->force_stance_foot();
            f2_toes=character->force_stance_toes();
        }else{
            f2=character->force_swing_foot();
            f2_toes=character->force_swing_toes();
        }
        //*/

        //        const Vector3d* f2=character->force_stance_foot();
        //        const Vector3d f2_toes=character->force_stance_toes();
        std::vector<Vector3d> f_real;
        f_real.push_back(f2[0]);
        f_real.push_back(f2[1]);
        f_real.push_back(f2[2]);
        f_real.push_back(f2[3]);
        f_real.push_back(f2_toes);

        /*
        {
            double eval_reducted_model=evaluate_simulation_state(f);
            double eval_real_world=evaluate_simulation_state(f_real);
            std::ofstream ofs ("eval_estimate/simulation_estimation.csv", std::ofstream::app);

            //write the toes force
            ofs<<phi<<","<<eval_real_world<<","<<eval_reducted_model;
            ofs<<std::endl;

            ofs.close();
        }
        //*/
        //*
        {
            std::ofstream ofs ("eval_estimate/test.csv", std::ofstream::app);

            static bool first_pass=true;
            if (first_pass){
                ofs<<"phi"<<","
                            "bl virt, bl real"<<","<<
                     "br virt, br real"<<","<<
                     "fl virt, fl real"<<","<<
                     "fr virt, fr real"<<","<<
                     "toes virt, toes real"<<std::endl;
                first_pass=false;
            }
            //write the foot forces
            ofs<<phi<<",";
            for (int i=0;i<4;++i){
                ofs<<f[i].y<<","<<f2[i].y<<",";
            }
            //write the toes force
            ofs<<f[4].y<<","<<f2_toes.y;
            ofs<<std::endl;

            ofs.close();
        }       //*/

    }

    //eval real world
    {   const Vector3d* f2;
        Vector3d f2_toes;
        if (phi!=0.0){
            f2=character->force_stance_foot();
            f2_toes=character->force_stance_toes();
        }else{
            f2=character->force_swing_foot();
            f2_toes=character->force_swing_toes();
        }
        //*/

        //        const Vector3d* f2=character->force_stance_foot();
        //        const Vector3d f2_toes=character->force_stance_toes();
        std::vector<Vector3d> f_real;
        f_real.push_back(f2[0]);
        f_real.push_back(f2[1]);
        f_real.push_back(f2[2]);
        f_real.push_back(f2[3]);
        f_real.push_back(f2_toes);


        count_sim_step++;

        limit_cor*=0.75;
        limit_sag*=0.75;
        EvalStruct eval_result=evaluate_simulation_state(f_real,character->stance_foot()->getOrientation());
        limit_cor/=0.75;
        limit_sag/=0.75;

        if (eval_result.value()>0.1){
            count_sim_step_with_problem++;
        }
    }


    if (body_config<0){
        return;
    }

    //we nee to know if we use the single stance or the dual stance for complex worlds
    //we will keep the dual one if the real world as been in it for less than
    //3 simulation step aerlier
    if (character->is_swing_foot_in_contact()){
        if (m_countdown_switch_to_single_stance==-1){

            std::cout<<"now using dual stance estimation"<<std::endl;
            for (int i=0;i<m_worlds.size();++i){
                m_worlds[i]->switch_to_dual_stance_estimation();
            }

            SimGlobals::stanceFootWorld=m_worlds.front()->get_stance_world();

        }
        m_countdown_switch_to_single_stance=10;
    }else{
        m_countdown_switch_to_single_stance--;
    }

    if (m_countdown_switch_to_single_stance==0){
        std::cout<<"now using single stance estimation"<<std::endl;
        for (int i=0;i<m_worlds.size();++i){
            m_worlds[i]->switch_to_single_stance_estimation();
        }

        SimGlobals::stanceFootWorld=m_worlds.front()->get_stance_world();

        m_countdown_switch_to_single_stance--;
    }


    for (int i=0;i<m_worlds.size();++i){
        m_worlds[i]->set_torques_base(torques);
    }

    m_delta_torques.clear();
    m_delta_torques.resize(3,Vector3d(0,0,0));
    m_worlds.front()->set_delta_torques(m_delta_torques);
    //do a test to see if the initial values are good enougth
    m_worlds.front()->run_simulation(phi);

    double eval_reducted_model_initial;
    std::vector<Vector3d> f_initial;
    {
        std::vector<Vector3d> f=m_worlds.front()->get_contact_forces();

        limit_cor*=0.75;
        limit_sag*=0.75;
        EvalStruct eval_result=evaluate_simulation_state(f,m_worlds.front()->get_stance_foot_orientation());
        limit_cor/=0.75;
        limit_sag/=0.75;


        eval_reducted_model_initial=eval_result.value();
        initial_evaluation_details=eval_result;
        f_initial=f;
    }

    //if we have a perfect solution without having anything to do it's perfect
    //we can just skip all the following computations
    if (eval_reducted_model_initial<0.1)
    {
        m_result_torques_parent_coord=m_delta_torques;
        m_result_torques=m_delta_torques;
        m_contact_forces=f_initial;

        //std::cout<<"StanceFootContactController::simulation_step sup torque not needed";
        return;
    }


    /*
//    if (eval_reducted_model_initial>50&&eval_reducted_model_initial<100)
    if(initial_evaluation_details.angle_sag<0.0)
    {
        m_result_torques_parent_coord=m_delta_torques;
        m_result_torques=m_delta_torques;
        m_contact_forces=f_initial;

        //std::cout<<"StanceFootContactController::simulation_step sup torque not needed";
        return;
    }
    //*/
    bool use_cma_version=true;
    if (use_cma_version){
        m_result_torques_parent_coord=find_cma_solution(f_initial);


        m_result_torques.clear();
        m_result_torques.resize(3,Vector3d(0,0,0));
        Vector3d delta_t,delta_t_w;
        delta_t=m_result_torques_parent_coord[0];
        delta_t_w=character->stance_foot()->getWorldCoordinates(delta_t);
        m_result_torques[0]=delta_t_w;

        delta_t=m_result_torques_parent_coord[1];
        delta_t_w=character->stance_foot()->parent_joint()->parent()->getWorldCoordinates(delta_t);
        m_result_torques[1]=delta_t_w;

        delta_t=m_result_torques_parent_coord[2];
        delta_t_w=character->stance_foot()->parent_joint()->parent()->parent_joint()->parent()->getWorldCoordinates(delta_t);
        m_result_torques[2]=delta_t_w;

        if(print_results){
            m_worlds.front()->set_delta_torques(m_result_torques_parent_coord);
            m_worlds.front()->run_simulation(phi);
            std::vector<Vector3d> f=m_worlds.front()->get_contact_forces();
            m_contact_forces=f;
        }


        /*
        std::vector<Vector3d> f=m_contact_forces.back();
        double eval=evaluate_simulation_state(f);

        std::ostringstream oss;
        oss<<"eval of cma solution: "<<eval;
        std::cout<<oss.str();
        //*/

    }

}

void StanceFootContactController::postprocess_simulation_step(bool new_character_step)
{
    m_current_position++;




    if (new_character_step){
        init_new_character_step();
    }
}



EvalStruct StanceFootContactController::evaluate_simulation_state(std::vector<Vector3d> f, Quaternion stance_foot_orientation)
{
    EvalStruct eval_result;

    //check if the foot touch the ground
    double sum_complete=0;
    for(int i=0;i<5;++i){
        sum_complete+=f[i].y;
    }

    double sum=sum_complete-f[4].y;
    eval_result.sum_complete=sum_complete;
    eval_result.sum_complete_wo_toes=sum;

    if (sum_complete <100){
        eval_result.foot_eval=1.0E16/(std::pow(1.0E1,(static_cast<int>(sum_complete/10)+1)));
        return eval_result;
    }
    if (sum<75){
        eval_result.foot_eval=1E5;
        return eval_result;
    }




    double eval=0;
    double eval_toes=0;

    //    //and evaluate the result
    double val_left = (f[0].y + f[2].y)/sum;
    double val_right = (f[1].y+ f[3].y)/sum;
    double val_back = (f[0].y + f[1].y)/sum_complete;
    double val_front = (f[2].y+ f[3].y+f[4].y)/sum_complete;
    eval_result.val_left=val_left;
    eval_result.val_right=val_right;
    eval_result.val_back=val_back;
    eval_result.val_front=val_front;

    bool one_sided=false;
    if (val_left<limit_cor){
        if (val_left<limit_cor){
            double delta=limit_cor-val_left;
            eval+= delta*delta*1000;
            if (val_left<0.000001){
                one_sided=true;
            }
        }
    }

    if (val_right<limit_cor){
        if (val_right<limit_cor){
            double delta=limit_cor-val_right;
            eval+= delta*delta*1000;
            if (val_right<0.000001){
                one_sided=true;
            }
        }
    }

    if (one_sided){
        Quaternion q=stance_foot_orientation.decomposeRotation(Vector3d(0,0,1));
        double angle_z=q.getRotationAngle(Vector3d(0,0,1));
        double delta=angle_z*100;
        eval+=delta*delta;

        eval_result.angle_cor=angle_z;
    }

    if (SimGlobals::foot_flat_on_ground){
        one_sided=false;

        if (val_back<limit_sag){
            double delta=limit_sag-val_back;
            eval+= delta*delta*1000;
            if (val_back<0.000001){
                one_sided=true;
            }
        }

        if (val_front<limit_sag){
            double delta=limit_sag-val_front;
            eval+= delta*delta*1000;
            if (val_front<0.000001){
                one_sided=true;
            }
        }

        if (one_sided){
            Quaternion q=stance_foot_orientation.decomposeRotation(Vector3d(1,0,0));
            double angle_x=q.getRotationAngle(Vector3d(1,0,0));
            double delta=angle_x*100;
            eval+=delta*delta;

            eval_result.angle_sag=angle_x;
        }

        //now I'll penalize if the foot is only standing on it's toes for the front part
        if (val_front>0.0001){

            double toes_on_front=f[4].y/(f[2].y+ f[3].y+f[4].y);
            eval_result.toes_on_front=toes_on_front;

            if (toes_on_front>0.5){
                double delta=toes_on_front-0.5;
                eval_toes= delta*delta*100;
            }else if (toes_on_front<0.1){
                double delta=0.1-toes_on_front;
                eval_toes= delta*delta*100;
            }
        }else{
            //we set max penal if the front doen't touch
            eval_toes=0.5*0.5*100;
        }
    }

    eval_result.foot_eval=eval;
    eval_result.toes_eval=eval_toes;

    return eval_result;
}

EvalStruct StanceFootContactController::evaluate_distance_to_original(std::vector<Vector3d> f, std::vector<Vector3d> f_initial)
{
    EvalStruct eval_result;

    double sum_complete_init=0;
    for(int i=0;i<5;++i){
        sum_complete_init+=f_initial[i].y;
    }
    double sum_init=sum_complete_init-f_initial[4].y;

    //this case mean the distance to original is ignored (since it has no real meaning with tsuch low forces
    //so we set it to the maximum possible value
    if ((sum_complete_init<=25)||(sum_init<=0.1)){
        double eval=0;
        double eval_toes=0;
        eval_result.foot_eval=eval;
        eval_result.toes_eval=eval_toes;
        eval=42.0;
        eval_toes=0.5;
        return eval_result;
    }

    double val_left_init = (f_initial[0].y + f_initial[2].y)/sum_init;
    double val_right_init = (f_initial[1].y+ f_initial[3].y)/sum_init;
    double val_back_init = (f_initial[0].y + f_initial[1].y)/sum_complete_init;
    double val_front_init = (f_initial[2].y+ f_initial[3].y+f_initial[4].y)/sum_complete_init;

    if (val_left_init<limit_cor){
        val_left_init=limit_cor;
        val_right_init=1-limit_cor;
    }

    if (val_right_init<limit_cor){
        val_left_init=1-limit_cor;
        val_right_init=limit_cor;
    }

    if (val_back_init<limit_sag){
        val_back_init=limit_sag;
        val_front_init=1-limit_sag;
    }

    if (val_front_init<limit_sag){
        val_back_init=1-limit_sag;
        val_front_init=limit_sag;
    }



    //check if the foot touch the ground
    double sum_complete=0;
    for(int i=0;i<5;++i){
        sum_complete+=f[i].y;
    }
    double sum=sum_complete-f[4].y;

    //normaly this case cannot happen or at leas tis meaningless since the quality criterium will
    //reject anithing that doesn't have a sum of 25
    //still I need to set the return to the max possible value for coherence
    double eval=0;
    double eval_toes=0;
    if ((sum_complete<=25)||(sum<=0.1)){
        //at max we have a delta of 1
        //so the total value would be 4*(1*1)*10=40;
        eval=42.0;
        eval_toes=0.5;
    }else{
        double val_left = (f[0].y + f[2].y)/sum;
        double val_right = (f[1].y+ f[3].y)/sum;
        double val_back = (f[0].y + f[1].y)/sum_complete;
        double val_front = (f[2].y+ f[3].y+f[4].y)/sum_complete;

        //and evaluate the result
        eval+=(val_left_init-val_left)*(val_left_init-val_left)*10;
        eval+=(val_right_init-val_right)*(val_right_init-val_right)*10;


        if (SimGlobals::foot_flat_on_ground){
            eval+=(val_back_init-val_back)*(val_back_init-val_back)*10;
            eval+=(val_front_init-val_front)*(val_front_init-val_front)*10;


            if ((f_initial[2].y+ f_initial[3].y+f_initial[4].y)>0.001){
                if (val_front>0.001){
                    double val_toes_on_front_init=f_initial[4].y/(f_initial[2].y+ f_initial[3].y+f_initial[4].y);

                    if (val_toes_on_front_init>0.5){
                        val_toes_on_front_init=0.5;
                    }

                    if (val_toes_on_front_init<0.1){
                        val_toes_on_front_init=0.1;
                    }

                    double val_toes_on_front=f[4].y/(f[2].y+ f[3].y+f[4].y);

                    eval_toes=(val_toes_on_front_init-val_toes_on_front)*(val_toes_on_front_init-val_toes_on_front)*2;
                }else{
                    //it's the maximun possible value
                    eval_toes=0.5;
                }
            }
        }

        if (eval!=eval){
            std::cout<<"StanceFootContactControlObjectiveFunction::eval ack the eval result is nan ...";
        }
    }

    eval_result.foot_eval=eval;
    eval_result.toes_eval=eval_toes;

    return eval_result;

}


std::vector<Vector3d> StanceFootContactController::find_cma_solution(std::vector<Vector3d> f_init)
{
    shark::CMA cma;
    StanceFootContactControlObjectiveFunction* objective_func;
    objective_func=new StanceFootContactControlObjectiveFunction(this,f_init);

    cma.init(*objective_func);//the parameter here is the error function
    cma.setSigma(0.15); // Explicitely set initial globael step size.
    int nbr_iter = 0;
    const int nbr_eval_limit = 50;
    const int eval_value_limit = 1.0;
    // Iterate the optimizer until a solution of sufficient quality is found.




    EvalStruct eval_result=evaluate_simulation_state(f_init,m_worlds.front()->get_stance_foot_orientation());
    EvalStruct eval_detail_init=eval_result;
    double cur_val=eval_result.value()*10;


    if (print_results){
        std::ostringstream oss;
        oss<< "cma_original_evaluation: "<<eval_result.foot_eval<<"   "<<eval_result.toes_eval;
        std::cout << oss.str();

    }
    shark::RealVector result;
    CompleteEval eval_detail_best;
    do {

        //        std::cout<<std::endl;
        //evolve the parameters
        cma.step(*objective_func);


        // Report information on the optimizer state and the current solution to the console.
        if (print_results)
        {
            std::cout<< nbr_iter << " :: " << objective_func->evaluationCounter() << " :: " <<
                        cma.solution().value << " :: " << cma.sigma()<<std::endl;
        }

        if (cur_val > cma.solution().value){
            //save the current best solution
            cur_val = cma.solution().value;
            result = cma.solution().point;
            eval_detail_best= objective_func->m_sup_data[cma.best_id()].second;
        }

        //delete ceval;
        objective_func->clear_sup_data();



    } while ((cma.solution().value > eval_value_limit)&&(objective_func->evaluationCounter()<nbr_eval_limit));
    //*/


    std::vector<Vector3d> cma_solution;
    cma_solution.resize(3,Vector3d(0,0,0));
    if (!result.empty()){
        std::vector<Vector3d> sample;
        Vector3d delta;
        double v_x,v_z;
        double val;
        int articulation_id=0;
        int result_id=0;


        //toes
        if (objective_func->handle_toes_joint){
            val=result[result_id++];
            v_x=objective_func->sample_max[articulation_id].x*val+objective_func->sample_min[articulation_id].x*(1-val);
            delta=Vector3d(v_x,0,0);
            articulation_id++;
        }else{
            delta=Vector3d(0,0,0);
        }
        sample.push_back(delta);


        //ankle
        val=result[result_id++];
        v_x=objective_func->sample_max[articulation_id].x*val+objective_func->sample_min[articulation_id].x*(1-val);
        val=result[result_id++];
        v_z=objective_func->sample_max[articulation_id].z*val+objective_func->sample_min[articulation_id].z*(1-val);
        delta=Vector3d(v_x,0,v_z);
        sample.push_back(delta);
        articulation_id++;

        //knee
        val=result[result_id++];
        v_x=objective_func->sample_max[articulation_id].x*val+objective_func->sample_min[articulation_id].x*(1-val);
        delta=Vector3d(v_x,0,0);
        sample.push_back(delta);
        articulation_id++;


        cma_solution=sample;




        if (print_results){
            std::ostringstream oss;
            oss<< "cma_result: "<<result[0]<<" "<<result[1]<<" "<<result[2]<<" "<<result[3];
            std::cout << oss.str();
        }
    }



    if (false){

        std::string file_name="eval_estimate/foot_eval_details.csv";
        static bool first_pass=true;
        if (first_pass){

            std::remove(file_name.c_str());

            std::ofstream ofs (file_name, std::ofstream::app);

            ofs<<"sum_complete_init"<<", "<<
                 "sum_complete_wo_toes_init"<<", "<<
                 "val_left_init"<<", "<<
                 "val_right_init"<<", "<<
                 "angle_cor_init"<<", "<<
                 "val_back_init"<<", "<<
                 "val_front_init"<<", "<<
                 "angle_sag_init"<<", "<<
                 "toes_on_front_init"<<", "<<
                 "foot_flat_on_ground"<<", "<<
                 "initial_eval"<<", "<<
                 "cma_eval"<<", "<<
                 "best_quality_eval"<<", "<<
                 "best_distance_eval"<<", "<<
                 "nb_estimation"<<", "<<
                 /*
                                                                                                           "sum_complete"<<", "<<
                                                                                                           "sum_complete_wo_toes"<<", "<<
                                                                                                           "val_left"<<", "<<
                                                                                                           "val_right"<<", "<<
                                                                                                           "angle_cor"<<", "<<
                                                                                                           "val_back"<<", "<<
                                                                                                           "val_front"<<", "<<
                                                                                                           "angle_sag"<<", "<<
                                                                                                           "toes_on_front"<<", "<<
                                                                                                           //*/
                 "torque_toes_x"<<", "<<
                 "torque_ankle_x"<<", "<<
                 "torque_ankle_z"<<", "<<
                 "torque_knee_x"<<
                 std::endl;

            ofs.close();
            first_pass=false;
        }

        std::ofstream ofs (file_name, std::ofstream::app);

        ofs<<
              eval_detail_init.sum_complete<<", "<<
              eval_detail_init.sum_complete_wo_toes<<", "<<
              eval_detail_init.val_left<<", "<<
              eval_detail_init.val_right<<", "<<
              eval_detail_init.angle_cor<<", "<<
              eval_detail_init.val_back<<", "<<
              eval_detail_init.val_front<<", "<<
              eval_detail_init.angle_sag<<", "<<
              eval_detail_init.toes_on_front<<", "<<
              SimGlobals::foot_flat_on_ground<<", "<<
              eval_detail_init.value()<<", "<<
              cur_val/10.0<<", "<<
              eval_detail_best.eval_quality.value()<<", "<<
              eval_detail_best.eval_distance.value()<<", "<<
              objective_func->evaluationCounter()<<", "<<
              /*
                                                                                                                          eval_detail_best.eval_quality.sum_complete<<", "<<
                                                                                                                          eval_detail_best.eval_quality.sum_complete_wo_toes<<", "<<
                                                                                                                          eval_detail_best.eval_quality.val_left<<", "<<
                                                                                                                          eval_detail_best.eval_quality.val_right<<", "<<
                                                                                                                          eval_detail_best.eval_quality.angle_cor<<", "<<
                                                                                                                          eval_detail_best.eval_quality.val_back<<", "<<
                                                                                                                          eval_detail_best.eval_quality.val_front<<", "<<
                                                                                                                          eval_detail_best.eval_quality.angle_sag<<", "<<
                                                                                                                          eval_detail_best.eval_quality.toes_on_front<<", "<<
                                                                                                                          //*/
              cma_solution[0].x<<", "<<
              cma_solution[1].x<<", "<<
              cma_solution[1].z<<", "<<
              cma_solution[2].x<<
              std::endl;

        ofs.close();
        //*/

    }


    count_cma_usage++;
    if (cma.solution().value<eval_value_limit){
        count_cma_evaluation+=objective_func->evaluationCounter();
        count_cma_success++;
    }else{
        cma_solution.clear();
        cma_solution.push_back(Vector3d(0,0,0));
        cma_solution.push_back(Vector3d(0,0,0));
        cma_solution.push_back(Vector3d(0,0,0));
    }

    delete objective_func;

    return cma_solution;

}




/***********************************************************************************************/
/*                           END OF StanceFootContactController                                */
/***********************************************************************************************/


/***********************************************************************************************/
/*                   START OF StanceFootContactControlObjectiveFunction                        */
/***********************************************************************************************/


StanceFootContactControlObjectiveFunction::StanceFootContactControlObjectiveFunction(StanceFootContactController *i_controller, std::vector<Vector3d> f_init): shark::SingleObjectiveFunction(){
    m_features |= HAS_VALUE;
    m_features |= CAN_PROPOSE_STARTING_POINT;
    //    m_features |= IS_THREAD_SAFE;

    c=i_controller;
    f_initial=f_init;
    m_mutex_sup_data=new QMutex();


    if (!SimGlobals::foot_flat_on_ground){
        handle_toes_joint=false;
    }else{
        handle_toes_joint=true;
    }

    //I'll desactivate the toes joint for now anyway. it ill be handled in a second phase
    handle_toes_joint=false;

    int toes_joint_idx=c->character->stance_foot()->child_joints()[0]->idx();
    int ankle_idx=c->character->stance_foot()->parent_joint()->idx();
    int knee_idx=c->character->stance_foot()->parent_joint()->parent()->parent_joint()->idx();

    EvalStruct init_details=c->initial_evaluation_details;
    Vector3d center=Vector3d(0,0,0);

    int starting_point_type=2;


    Vector3d t;
    //toes joint
    if (handle_toes_joint){
        //    t.x=0.08;t.y=0;t.z=0;
        //    t=c->pose_controller->compute_torque_for_joint_from_angular_displacement(toes_joint_idx,t);
        t.x=0.8;
        t.y=0;
        t.z=0;
        sample_min.push_back(center-t);
        sample_max.push_back(center+t);
    }

    //ankle
    //    t.x=0.08;t.y=0;t.z=0.08;
    //    t=c->pose_controller->compute_torque_for_joint_from_angular_displacement(ankle_idx,t);

    //*
    t=Vector3d(0,0,0);
    center=Vector3d(0,0,0);
    if (starting_point_type==0){
        t.x=2.4;    t.y=0;    t.z=2.4;
    }else if (starting_point_type==1){
        center.x=10; t.x=12.5;
        center.z=0; t.z=20;
    }else{
        if (!SimGlobals::foot_flat_on_ground){
            //X rules
            //general case
            center.x=10; t.x=7.5;

            //Z rules
            //general case
            center.z=0; t.z=10;

            if (init_details.angle_cor!=0.0){
                if (init_details.angle_cor<0.0){
                    center.z=-5; t.z=5;
                }else{
                    center.z=5; t.z=5;
                }
            }else {
                if (init_details.val_left!=0.0){
                    if (init_details.val_left>0.8){
                        center.z=-11.25*init_details.val_left+9.5; t.z=-1.25*init_details.val_left+5.5;
                    }else if (init_details.val_left<0.2){
                        center.z=-25*init_details.val_left+5; t.z=5;
                    }
                }
            }
        }else{
            //X rules
            //general case
            center.x=10; t.x=12.5;

            if (init_details.angle_sag>0.0){
                center.x=1775*init_details.angle_sag+19.5; t.x=225*init_details.angle_sag+15.5;
            }else if (init_details.angle_sag<0.0){
                //                center.x=1775*init_details.angle_sag+19.5; t.x=-225*init_details.angle_sag+15.5;
                center.x=-50; t.x=30;
            }else {
                if (init_details.val_back!=0.0){
                    if (init_details.val_back>0.8){
                        center.x=-80*init_details.val_back+64; t.x=16;
                    }else if (init_details.val_back<0.2){
                        center.x=-20*init_details.val_back+16; t.x=12;
                    }
                }
            }

            //Z rules
            //general case
            center.z=0; t.z=20;
            if (init_details.angle_cor!=0.0){
                if (init_details.angle_cor<0.0){
                    center.z=375*init_details.angle_cor-1.25; t.z=-125*init_details.angle_cor+3.75;
                }else{
                    center.z=375*init_details.angle_cor+1.25; t.z=125*init_details.angle_cor+3.75;
                }
            }else {
                if (init_details.val_left!=0.0){
                    if (init_details.val_left>0.8){
                        center.z=-45*init_details.val_left+30; t.z=9;
                    }else if (init_details.val_left<0.2){
                        center.z=-50*init_details.val_left+16; t.z=9;
                    }
                }
            }
        }
    }
    sample_min.push_back(center-t);
    sample_max.push_back(center+t);
    //*/


    //knee
    //    t.x=0.08;t.y=0;t.z=0;
    //    t=c->pose_controller->compute_torque_for_joint_from_angular_displacement(knee_idx,t);
    t=Vector3d(0,0,0);
    center=Vector3d(0,0,0);
    if (starting_point_type==0){
        t.x=24; t.y=0; t.z=0;
    }else if (starting_point_type==1){
        center.x=40; t.x=70;
    }else{
        center.x=40; t.x=70;
        if (!SimGlobals::foot_flat_on_ground){
            center.x=35; t.x=40;
        }else{
            center.x=50; t.x=75;
        }
    }
    sample_min.push_back(center-t);
    sample_max.push_back(center+t);


    /*
    double previous_input_damping=0.5;
    for (int i=0;i<3;++i){
        sample_min[i]+=c->m_result_torques_parent_coord[i]*previous_input_damping;
        sample_max[i]+=c->m_result_torques_parent_coord[i]*previous_input_damping;
    }
    //*/

    int nb_variables=4;

    //without the toes we have one less variable to worry about
    if (!handle_toes_joint){
        nb_variables--;
    }

    variable_vector.clear();
    for (int i=0;i<nb_variables;++i){
        variable_vector.push_back(0.5);
    }


    //we set the dimentionality
    setNumberOfVariables(variable_vector.size());
}

StanceFootContactControlObjectiveFunction::~StanceFootContactControlObjectiveFunction()
{
    f_initial.clear();
    variable_vector.clear();
    sample_max.clear();
    sample_min.clear();
}

StanceFootContactControlObjectiveFunction::ResultType StanceFootContactControlObjectiveFunction::eval(const SearchPointType &input, int offspring_id)
{
    m_evaluationCounter++;
    SIZE_CHECK(input.size() == m_dimensions);


    EvalStruct eval_quality;
    EvalStruct eval_distance;

    if (this->isThreadSafe()){

        if (false){
            std::thread::id this_id = std::this_thread::get_id();
            std::ostringstream oss;
            oss<<"  thread num start: "<<this_id<<std::endl;
            std::cout<<oss.str();
        }



        //lock semaphore
        c->semaphore->acquire();


        //look for free wold and lock it
        int selected_world=-1;
        for (int i=0;i<c->m_worlds.size();++i){
            if (c->m_worlds_mutex[i]->tryLock())
            {
                selected_world=i;
                break;
            }

        }


        if (false){
            std::thread::id this_id = std::this_thread::get_id();
            std::ostringstream oss;
            oss<<"used world: "<<selected_world<<"  worlds left: "<<c->semaphore->available();
            oss<<"  is thread safe: "<< this->isThreadSafe() ;
            oss<<"  thread num: "<<this_id;
            oss<<std::endl;
            std::cout<<oss.str();
        }


        std::vector<Vector3d> sample;
        Vector3d delta;
        double v_x,v_z;
        double val;

        int cur_id=0;
        int articulation_id=0;
        //toes
        if (handle_toes_joint){
            val=input[cur_id++];
            v_x=sample_max[articulation_id].x*val+sample_min[articulation_id].x*(1-val);
            delta=Vector3d(v_x,0,0);
            sample.push_back(delta);
            articulation_id++;
        }


        //ankle
        val=input[cur_id++];
        v_x=sample_max[articulation_id].x*val+sample_min[articulation_id].x*(1-val);
        val=input[cur_id++];
        v_z=sample_max[articulation_id].z*val+sample_min[articulation_id].z*(1-val);
        delta=Vector3d(v_x,0,v_z);
        sample.push_back(delta);
        articulation_id++;

        //knee
        val=input[cur_id++];
        v_x=sample_max[articulation_id].x*val+sample_min[articulation_id].x*(1-val);
        delta=Vector3d(v_x,0,0);
        sample.push_back(delta);
        articulation_id++;

        c->m_worlds[selected_world]->set_delta_torques(sample);

        //*
        if (c->m_reusable_thread[selected_world]==NULL){
            c->m_reusable_thread[selected_world]=new std::thread(thread_world_execution,c,selected_world);
        }else{
            c->m_semaphore_execute_thread[selected_world]->release();
        }
        c->m_semaphore_end[selected_world]->acquire();
        //*/

        //    c->m_forward_sim_mutex->lock();
        //    c->m_worlds[selected_world]->run_simulation(c->phi);
        //    c->m_forward_sim_mutex->unlock();

        std::vector<Vector3d> f=c->m_worlds[selected_world]->get_contact_forces();

        eval_quality=c->evaluate_simulation_state(f,c->m_worlds[selected_world]->get_stance_foot_orientation());

        //release the world
        c->m_worlds_mutex[selected_world]->unlock();

        c->semaphore->release();
        //    return 0;

        eval_distance=c->evaluate_distance_to_original(f,f_initial);


        CompleteEval ceval;
        ceval.eval_quality=eval_quality;
        ceval.eval_distance=eval_distance;

        std::pair<int,CompleteEval> pair_item(offspring_id,ceval);
        m_sup_data.push_back(pair_item);

        m_mutex_sup_data->lock();
        m_sup_data.push_back(pair_item);
        m_mutex_sup_data->unlock();
    }else{

        std::vector<Vector3d> sample;
        Vector3d delta;
        double v_x,v_z;
        double val;

        int cur_id=0;
        int articulation_id=0;
        //toes
        if (handle_toes_joint){
            val=input[cur_id++];
            v_x=sample_max[articulation_id].x*val+sample_min[articulation_id].x*(1-val);
            delta=Vector3d(v_x,0,0);
            sample.push_back(delta);
            articulation_id++;
        }


        //ankle
        val=input[cur_id++];
        v_x=sample_max[articulation_id].x*val+sample_min[articulation_id].x*(1-val);
        val=input[cur_id++];
        v_z=sample_max[articulation_id].z*val+sample_min[articulation_id].z*(1-val);
        delta=Vector3d(v_x,0,v_z);
        sample.push_back(delta);
        articulation_id++;

        //knee
        val=input[cur_id++];
        v_x=sample_max[articulation_id].x*val+sample_min[articulation_id].x*(1-val);
        delta=Vector3d(v_x,0,0);
        sample.push_back(delta);
        articulation_id++;

        c->m_worlds.front()->set_delta_torques(sample);

        c->m_worlds.front()->run_simulation(c->phi);

        std::vector<Vector3d> f=c->m_worlds.front()->get_contact_forces();

        eval_quality=c->evaluate_simulation_state(f,c->m_worlds.front()->get_stance_foot_orientation());
        eval_distance=c->evaluate_distance_to_original(f,f_initial);


        CompleteEval ceval;
        ceval.eval_quality=eval_quality;
        ceval.eval_distance=eval_distance;

        std::pair<int,CompleteEval> pair_item(offspring_id,ceval);
        m_sup_data.push_back(pair_item);
    }

    //I want to keep the best results on the quality criterium first
    //then if we have multiples pretty similar result I want the nearest from the initial
    //some notes if the foot does not touch the ground correctly, the system will just ingore the distance to original in the evaluation.

    double eval_result= eval_quality.value()*10+eval_distance.value();//c->evaluate_simulation_state();

    if (eval_result!=eval_result){
        std::cout<<"StanceFootContactControlObjectiveFunction::eval ack the eval result is nan ...";
    }

    if (false){
        std::thread::id this_id = std::this_thread::get_id();
        std::ostringstream oss;
        oss<<"  thread num end: "<<this_id<<std::endl;
        std::cout<<oss.str();
    }

    return eval_result;
}

void StanceFootContactControlObjectiveFunction::clear_sup_data()
{
    m_sup_data.clear();
}


/***********************************************************************************************/
/*                    END OF StanceFootContactControlObjectiveFunction                         */
/***********************************************************************************************/


void thread_world_execution(StanceFootContactController *c, int selected_world)
{
    c->m_worlds[selected_world]->load_ode_data();
    do{
        //        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        c->m_semaphore_execute_thread[selected_world]->acquire();

        if (c->m_program_end){
            return;
        }

        c->m_worlds[selected_world]->run_simulation(c->phi);

        c->m_semaphore_end[selected_world]->release();
    }while(true);
}
