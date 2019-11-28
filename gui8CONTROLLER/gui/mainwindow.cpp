#include "gui/mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <iostream>
#include "consolewindow.h"
#include "simulationwindow.h"
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>

//#include <drvapi_error_string.h>
bool save_intermediary_results=false;



#include <GL/freeglut.h>
#include <GL/GL.h>
void display(void)
{
    // clear all pixels
    glClear (GL_COLOR_BUFFER_BIT);

    // draw white polygon (rectangle) with corners at
    //(0.25, 0.25, 0.0) and (0.75, 0.75, 0.0)

    glColor3f (1.0, 1.0, 1.0);
    glBegin(GL_POLYGON);
    glVertex3f (0.25, 0.25, 0.0);
    glVertex3f (0.75, 0.25, 0.0);
    glVertex3f (0.75, 0.75, 0.0);
    glVertex3f (0.25, 0.75, 0.0);
    glEnd();

    // don't wait!
    //start processing buffered OpenGL routines

    glFlush ();
}

void init (void)
{
    // select clearing (background) color
    glClearColor (0.0, 0.0, 0.0, 0.0);

    //initialize viewing values
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);
}

#include <iostream>
void test_opengl(){
    //dummies parameters
    char *myargv [1];
    int myargc=1;
    myargv [0]=strdup ("Myappname");

    glutInit(&myargc, myargv);
    glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize (250, 250);
    glutInitWindowPosition (100, 100);
    glutCreateWindow ("hello");
    init ();
    glutDisplayFunc(display);
    glutMainLoop();
}

#include "UI.h"
void test_api(){
    //Globals::animationRunning=true;
    //  core_init();
    // launch_gl();
}


MainWindow::MainWindow(int argc, char *argv[], QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //finish the ui

    //test_cuda();

    link_signals();

    //hide the 3 boxes useless for the presentation
    /*
    ui->spin_box_iter->hide();
    ui->spin_box_velD->hide();
    ui->spin_box_level->hide();

    //also rmv the 2 options
    ui->combo_box_mode->removeItem(1);
    ui->combo_box_mode->removeItem(1);
    //*/
    if (argc>1){
        try {
            int number = std::stoi(argv[1]);
            ui->spin_box_control_frequency->setValue(number);
        } catch (std::exception const &e) {
            // This could not be parsed into a number so an exception is thrown.
            // atoi() would return 0, which is less helpful if it could be a valid value.
            std::cout<<argc<<std::endl;
            exit(6589);
        }
    }

    if (argc>2){
        try {
            int number = std::stoi(argv[2]);
            ui->check_box_look_for_min_gains->setChecked(number==0);
        } catch (std::exception const &e) {
            // This could not be parsed into a number so an exception is thrown.
            // atoi() would return 0, which is less helpful if it could be a valid value.
            std::cout<<argc<<std::endl;
            exit(6589);
        }
    }

    if (argc>3){
        try {
            int number = std::stoi(argv[3]);
            ui->combo_box_mode->setCurrentIndex(number);
        } catch (std::exception const &e) {
            // This could not be parsed into a number so an exception is thrown.
            // atoi() would return 0, which is less helpful if it could be a valid value.
            std::cout<<argc<<std::endl;
            exit(6589);
        }
    }

    if (argc>4){
        try {
            int number = std::stoi(argv[4]);
            ui->spin_box_motion_type->setValue(number);
        } catch (std::exception const &e) {
            // This could not be parsed into a number so an exception is thrown.
            // atoi() would return 0, which is less helpful if it could be a valid value.
            std::cout<<argc<<std::endl;
            exit(6589);
        }
    }

    console_window= new ConsoleWindow();
    console_window->show();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::link_signals()
{
    connect(ui->btn_start, SIGNAL(clicked(bool)), this, SLOT(start_click()));




}

void MainWindow::closeEvent(QCloseEvent *event)
{
    (void)event;
    exit(0);
}

#include "gui/ControllerEditor.h"
void MainWindow::start_click()
{
    SimGlobals::nb_container_boxes=ui->spin_box_container_boxes->value();
    SimGlobals::nb_filler=ui->spin_box_filler_objects->value();
    Globals::use_fluid_heading=ui->check_box_fluid_heading->isChecked();

    Globals::evolution_type=1;
    Globals::evolution_push_type=2;
    Globals::look_for_min_gains=ui->check_box_look_for_min_gains->isChecked();

    //read the values from the ui
    SimGlobals::velDSagittal=ui->spin_box_velD->value();
    SimGlobals::velDCoronal=0;
    SimGlobals::water_level = ui->spin_box_level->value();
    SimGlobals::dt=1.0/(ui->spin_box_control_frequency->value());

    Globals::motion_cycle_type=ui->spin_box_motion_type->value();

    Globals::foot_contact_controller_config=ui->spin_box_foot_contact_control_config->value();
    if (Globals::foot_contact_controller_config>=0){
        Globals::use_contact_controller=true;
    }

    switch(ui->combo_box_mode->currentIndex()){
        case 0:
        {
            Globals::use_hand_position_tracking=false;
            Globals::evolution_mode = 0;
            Globals::animationRunning = 0;
            Globals::save_mode=false;
            Globals::close_after_saving=false;

            //Just start the simulation with standart values
            hide();
            console_window->show();
            core_init();
            sim_window=new SimulationWindow();
            connect(this->sim_window, SIGNAL(show_console()), this->console_window, SLOT(show()));
            ControllerEditor* editor=static_cast<ControllerEditor*>(Globals::app);
            connect(&editor->signal_messenger, SIGNAL(step_ended(double,double)),
                    this->sim_window, SLOT(character_step_ended(double,double)));
            connect(&editor->signal_messenger, SIGNAL(new_phase(int)),
                    this->sim_window, SLOT(slider_phase_changed(int)));
            connect(&editor->signal_messenger, SIGNAL(set_target_sagital_speed(int)),
                    this->sim_window, SLOT(slider_target_sagittal_speed_changed(int)));
            connect(&editor->signal_messenger, SIGNAL(set_target_coronal_speed(int)),
                    this->sim_window, SLOT(slider_target_coronal_speed_changed(int)));
            connect(&editor->signal_messenger, SIGNAL(set_desired_heading(int)),
                    this->sim_window, SLOT(slider_target_orientation_changed(int)));
            connect(&editor->signal_messenger, SIGNAL(set_step_width(int)),
                    this->sim_window, SLOT(slider_step_width_changed(int)));
            connect(&editor->signal_messenger, SIGNAL(set_liquid_level(int)),
                    this->sim_window, SLOT(slider_liquid_level_changed(int)));

            connect(sim_window, SIGNAL(close_signal()),
                    this, SLOT(end_simulation()));


            sim_window->show();
            launch_gl();

            //the only way to reach this place it that the simulation has been stoped
            //So I need to delete the structures
            delete sim_window;
            core_close();

            //finaly I show this window
            show();

            break;
        }
        case 1:{
            Globals::use_hand_position_tracking=true;
            Globals::evolution_mode = 1;
            //here Ill doo the optimisation/learning
            console_window->show();

            Globals::use_contact_controller=false;

            std::ostringstream oss_folder;
            //oss_folder<<SimGlobals::velDSagittal;
            oss_folder<<"results_opti_gains";
            std::string evo_folder= oss_folder.str();


            std::string result_folder = "optimisation_solution";

            Globals::use_normalized_sum=true;

            if (true){
                int i=0;
                for (;i<45;++i)
                {
                    std::ostringstream oss;
                    //            oss <<"starting evolution for water_lvl:" << SimGlobals::water_level;
                    oss <<"starting evolution for time step_lvl:" << 1.0f/SimGlobals::dt;
                    if (Globals::look_for_min_gains){
                        oss<< "    target min gains ";
                    }else{
                        oss<< "    target max gains ";
                    }

                    std::cout << oss.str();
                    int nb_iter=ui->spin_box_iter->value();
                    if (i>2){
                        nb_iter=50;
                    }
                    double result=cma_program(evo_folder,result_folder,1,nb_iter,i);

                    if (!Globals::look_for_min_gains){
                        if (result<=1.0){
                            Globals::max_gains_limit_sum*=2;
                        }
                    }

                    std::ostringstream oss2;
                    oss2 << "execution successfully finished "<<i<<" times";
                    std::cout << oss2.str();
                }
            }else{
                int nb_iter=ui->spin_box_iter->value();
                double result=cma_program(evo_folder,result_folder,1,nb_iter,0);
            }

            /*
            //save test
            double result=evaluate_solution(false);
            std::ostringstream oss;
            oss<<result;
            std::cout<<"result: "<<oss.str();
            //*/
            break;
        }
        case 2:
        {
            Globals::use_hand_position_tracking=true;
            Globals::evolution_mode = 1;
            //the evaluation mode
            //Just start the simulation with standart values
            hide();
            console_window->show();
            init_evaluate_solution(false,false);
            //*
            sim_window=new SimulationWindow();
            connect(this->sim_window, SIGNAL(show_console()), this->console_window, SLOT(show()));
            ControllerEditor* editor=static_cast<ControllerEditor*>(Globals::app);
            connect(&editor->signal_messenger, SIGNAL(step_ended(double,double)),
                    this->sim_window, SLOT(character_step_ended(double,double)));
            connect(&editor->signal_messenger, SIGNAL(new_phase(int)),
                    this->sim_window, SLOT(slider_phase_changed(int)));

            connect(sim_window, SIGNAL(close_signal()), this, SLOT(end_simulation()));
            sim_window->show();
            //*/
            evaluate_solution();

            //the only way to reach this place it that the simulation has been stoped
            //So I need to delete the structures
            delete sim_window;


            //finaly I show this window
            show();


            break;
        }
        default:{
            std::cerr<<"this mode is not handled yet"<<std::endl;
            break;
        }
    }
}

void MainWindow::end_simulation()
{
    //I need to stop the opengl loop
    stop_gl();
}


//*
SimbiconOnjectiveFunction::SimbiconOnjectiveFunction(){
    m_features |= HAS_VALUE;
    m_features |= CAN_PROPOSE_STARTING_POINT;

    //first we will need to load
    std::ostringstream oss;
    oss << Globals::data_folder_path;
    oss << "init/";
    oss << "input_optimisation.conF";

    inputFile = new char[100];
    strcpy(inputFile, oss.str().c_str());

    SimBiConFramework* con = new SimBiConFramework(inputFile, NULL);


    //I store the names
    vect_traj_name.push_back("root");
    vect_traj_name.push_back("STANCE_Knee");
    vect_traj_name.push_back("SWING_Ankle");
    vect_traj_name.push_back("STANCE_Ankle");
    vect_traj_name.push_back("swing_foot");
    vect_traj_name.push_back("pelvis_torso");


    //this should load all the concerned trajectories
    for (int k = 0; k < (int)vect_traj_name.size(); ++k){
        Trajectory* cur_traj = con->getController()->getState()->getTrajectory(vect_traj_name[k].c_str());
        for (int j = 0; j < (int)cur_traj->components.size(); ++j){
            TrajectoryComponent* traj_comp = cur_traj->components[j];
            for (int i = 0; i < (int)traj_comp->baseTraj.getKnotCount(); ++i){
                variable_vector.push_back(traj_comp->baseTraj.getKnotValue(i));
            }
        }
    }

    //then we add the step delta at the end
    //variable_vector.push_back(SimGlobals::ipm_alteration_effectiveness);

    //and delete the structure
    delete con;



    //we set the dimentionality
    setNumberOfVariables(variable_vector.size());

}

SimbiconOnjectiveFunction::ResultType SimbiconOnjectiveFunction::eval(const SearchPointType & input, int offspring_id) {
    SIZE_CHECK(input.size() == m_dimensions);

    //cannot remove those because the function is const so I can't put them as class variables ...
    static shark::RealVector last_vect;
    static double last_eval = -1;

    if (last_eval != -1){
        bool is_same = true;
        for (int i = 0; i < (int)input.size(); ++i){
            if (input[i] != last_vect[i]){
                is_same = false;
                break;
            }
        }
        //is we reach this point it means the point are the same so just cut the sim
        if (is_same){
            return last_eval;
        }
    }

    last_vect = input;

    //now I need to save the parameters back in the file so that the next eecution will see them
    SimBiConFramework* con = new SimBiConFramework(inputFile, NULL);

    //so we push the values in a structure
    int cur_pos = 0;
    for (int k = 0; k < (int)vect_traj_name.size(); ++k){
        Trajectory* cur_traj = con->getController()->getState()->getTrajectory(vect_traj_name[k].c_str());
        for (int j = 0; j < (int)cur_traj->components.size(); ++j){
            TrajectoryComponent* traj_comp = cur_traj->components[j];
            for (int i = 0; i < (int)traj_comp->baseTraj.getKnotCount(); ++i){
                traj_comp->baseTraj.setKnotValue(i, input[cur_pos]);
                ++cur_pos;
            }
        }
    }

    //I I detect the ipm alte I add it
    if (cur_pos < input.size()){
        SimGlobals::ipm_alteration_effectiveness = input[cur_pos];
        //limit it to 1 (since I want it to go down )
        if (SimGlobals::ipm_alteration_effectiveness > 1){
            SimGlobals::ipm_alteration_effectiveness = 1;
        }
        ++cur_pos;
    }
    else{
        SimGlobals::ipm_alteration_effectiveness = 1;
    }

    //also I'll prevent the system from doing the ondulations with the pelvis
    Trajectory* cur_traj = con->getController()->getState()->getTrajectory(vect_traj_name[0].c_str());
    for (int j = 0; j < (int)cur_traj->components.size(); ++j){
        TrajectoryComponent* traj_comp = cur_traj->components[j];
        for (int i = 0; i < (int)traj_comp->baseTraj.getKnotCount(); ++i){
            traj_comp->baseTraj.setKnotValue(i, std::fmin(0.1, traj_comp->baseTraj.getKnotValue(i)));
        }
    }

    //I only wanna learn the x component of the walk so I'l override the others with the value I had
    cur_traj = con->getController()->getState()->getTrajectory(vect_traj_name[5].c_str());
    for (int j = 0; j < (int)cur_traj->components.size(); ++j){
        TrajectoryComponent* traj_comp = cur_traj->components[0];
        traj_comp->baseTraj.setKnotValue(0, 0.000340);
        traj_comp->baseTraj.setKnotValue(1, -0.100323);
        traj_comp->baseTraj.setKnotValue(2, -0.001158);

        traj_comp = cur_traj->components[2];
        traj_comp->baseTraj.setKnotValue(0, 0.0);
        traj_comp->baseTraj.setKnotValue(1, 0.015874);
        traj_comp->baseTraj.setKnotValue(2, 0.0);
    }



    //and we save the structure
    con->save(true, false);

    //clean the memory
    delete con;


    //test the simulation
    init_evaluate_solution(false,true);
    last_eval=evaluate_solution();


    return last_eval;
}
//*/


//*
#include "Core/pose_control/posecontroller.h"
SimbiconOnjectiveFunctionGains::SimbiconOnjectiveFunctionGains(bool use_symetrics_gains, int cma_number,
                                                               std::string solution_folder): shark::SingleObjectiveFunction(){
    m_features |= HAS_VALUE;
    m_features |= CAN_PROPOSE_STARTING_POINT;

    use_symetry=use_symetrics_gains;
    asymetrics_legs=true;

    //first we will need to load
    std::ostringstream oss;
    oss << Globals::data_folder_path;
    oss << "init/";
    oss << "input_optimisation.conF";

    inputFile = new char[100];
    strcpy(inputFile, oss.str().c_str());

    SimBiConFramework* con = new SimBiConFramework(inputFile, NULL);


    //now we will do the save in the folder that only keep the best solution
    int a=1.0f/SimGlobals::dt;

    std::string solution_folder_path=get_folder_path(solution_folder,7);
    std::ostringstream oss2;
    oss2 << solution_folder_path << a <<"_"<<cma_number-1<< ".sbc";
    if (boost::filesystem::exists(oss2.str())){
        con->getController()->pose_controller->read_gains(oss2.str());
    }else{
        std::cout<<"SimbiconOnjectiveFunctionGains::SimbiconOnjectiveFunctionGains: could not load preivous result for iter: "<<cma_number<<std::endl;
    }


    const std::vector<ControlParams>& cp=con->getController()->get_control_params();
    const ControlParams& cp_root=con->getController()->get_root_control_params();


    //load the gains
    //I normalize the values
    original_val.push_back(cp_root.kp);
    original_val.push_back(cp_root.kd);

    if (use_symetry){
        int idx=0;
        //pelvis torso
        original_val.push_back(cp[idx].kp);
        original_val.push_back(cp[idx].kd);
        idx++;

        //hips
        original_val.push_back(cp[idx].kp);
        original_val.push_back(cp[idx].kd);
        idx+=2;
        if (asymetrics_legs){
            idx--;
            original_val.push_back(cp[idx].kp);
            original_val.push_back(cp[idx].kd);
            idx++;
        }

        //torso head
        original_val.push_back(cp[idx].kp);
        original_val.push_back(cp[idx].kd);
        idx++;

        //shoulder
        original_val.push_back(cp[idx].kp);
        original_val.push_back(cp[idx].kd);
        idx+=2;

        //knee
        original_val.push_back(cp[idx].kp);
        original_val.push_back(cp[idx].kd);
        idx+=2;
        if (asymetrics_legs){
            idx--;
            original_val.push_back(cp[idx].kp);
            original_val.push_back(cp[idx].kd);
            idx++;
        }

        //elbow
        original_val.push_back(cp[idx].kp);
        original_val.push_back(cp[idx].kd);
        idx+=2;

        //ankle
        original_val.push_back(cp[idx].kp);
        original_val.push_back(cp[idx].kd);
        idx+=2;
        if (asymetrics_legs){
            idx--;
            original_val.push_back(cp[idx].kp);
            original_val.push_back(cp[idx].kd);
            idx++;
        }

        //toes joint
        original_val.push_back(cp[idx].kp);
        original_val.push_back(cp[idx].kd);
        idx+=2;
        if (asymetrics_legs){
            idx--;
            original_val.push_back(cp[idx].kp);
            original_val.push_back(cp[idx].kd);
            idx++;
        }

        if (idx!=(int)cp.size()){
            exit(25683);
        }
    }else{
        for (int j = 0; j < (int)cp.size(); ++j){
            original_val.push_back(cp[j].kp);
            original_val.push_back(cp[j].kd);
        }
    }

    for (int j=0;j< (int) original_val.size();++j){
        variable_vector.push_back(1.0);
    }



    //and delete the structure
    delete con;

    //we set the dimentionality
    setNumberOfVariables(variable_vector.size());
}

SimbiconOnjectiveFunctionGains::ResultType SimbiconOnjectiveFunctionGains::eval(const SearchPointType & input, int offspring_id) {

    m_evaluationCounter++;

    SIZE_CHECK(input.size() == m_dimensions);

    //cannot remove those because the function is const so I can't put them as class variables ...
    static shark::RealVector last_vect;
    static double last_eval = -1;


    if (last_eval != -1){
        bool is_same = true;
        for (int i = 0; i < (int)input.size(); ++i){
            if (input[i] != last_vect[i]){
                is_same = false;
                break;
            }
        }
        //is we reach this point it means the point are the same so just cut the sim
        if (is_same){
            return last_eval;
        }
    }

    last_vect = input;

    double eval_sup_value=0.0f;
    {
        //now I need to save the parameters back in the file so that the next eecution will see them
        SimBiConFramework* con = new SimBiConFramework(inputFile, NULL);

        //so we push the values in a structure
        std::vector<ControlParams>& cp=con->getController()->get_control_params();
        ControlParams& cp_root=con->getController()->get_root_control_params();

        //load the gains
        int cur_pos=0;
        cp_root.kp=original_val[cur_pos]*input[cur_pos];
        cp_root.kd=original_val[cur_pos+1]*input[cur_pos+1];
        eval_sup_value+=input[cur_pos]+input[cur_pos+1];
        cur_pos+=2;

        if (use_symetry){
            int cp_idx=0;
            //pelvis torso
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            cur_pos+=2;

            //hips
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            if (asymetrics_legs){
               cur_pos+=2;
            }
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            cur_pos+=2;

            //torso head
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            cur_pos+=2;

            //shoulder
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            cur_pos+=2;

            //knee
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            if (asymetrics_legs){
               cur_pos+=2;
            }
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            cur_pos+=2;

            //elbow
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            cur_pos+=2;

            //ankle
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            if (asymetrics_legs){
               cur_pos+=2;
            }
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            cur_pos+=2;

            //toes joint
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            if (asymetrics_legs){
               cur_pos+=2;
            }
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            eval_sup_value+=input[cur_pos]+input[cur_pos+1];
            cur_pos+=2;

            eval_sup_value/=(cp.size()*2);
            if (cp_idx!=(int)cp.size()){
                exit(25684);
            }
            if (cur_pos!=(int)original_val.size()){
                exit(25685);
            }
        }else{
            for (int j = 0; j < (int)cp.size(); ++j){
                eval_sup_value+=input[cur_pos]+input[cur_pos+1];
                cp[j].kp=original_val[cur_pos]*input[cur_pos++];
                cp[j].kd=original_val[cur_pos]*input[cur_pos++];
            }
            eval_sup_value/=(cp.size()*2);

        }

        //the *4 is because my current eval is done 4 time with differnetn coronal speed
        if (!Globals::look_for_min_gains){
            eval_sup_value=(3.0-eval_sup_value)/3;

            //add the weight
            eval_sup_value*=16*4;
        }else{
            //add the weight
            eval_sup_value*=16*4;
        }




        SimGlobals::ipm_alteration_effectiveness = 1;


        //and we save the structure
        con->save(true, false);

        //clean the memory
        delete con;
    }

    //test the simulation
    init_evaluate_solution(false,true);

    last_eval=evaluate_solution();

    //add the weight value;
    if (Globals::use_normalized_sum){
        last_eval+=eval_sup_value;
    }

    /*
    {
        std::cout<<"last_eval: "<<last_eval<<"   eval_sup_value: "<<eval_sup_value<<std::endl;

    }
    //*/

    if (save_intermediary_results){
        //now I need to save the parameters back in the file so that the next eecution will see them
        SimBiConFramework* con = new SimBiConFramework(inputFile, NULL);

        //so we push the values in a structure
        std::vector<ControlParams>& cp=con->getController()->get_control_params();
        ControlParams& cp_root=con->getController()->get_root_control_params();

        //load the gains
        int cur_pos=0;
        cp_root.kp=original_val[cur_pos]*input[cur_pos++];
        cp_root.kd=original_val[cur_pos]*input[cur_pos++];

        if (use_symetry){
            int cp_idx=0;
            //pelvis torso
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            cur_pos+=2;

            //hips
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            if (asymetrics_legs){
               cur_pos+=2;
            }
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            cur_pos+=2;

            //torso head
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            cur_pos+=2;

            //shoulder
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            cur_pos+=2;

            //knee
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            if (asymetrics_legs){
               cur_pos+=2;
            }
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            cur_pos+=2;

            //elbow
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            cur_pos+=2;

            //ankle
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            if (asymetrics_legs){
               cur_pos+=2;
            }
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            cur_pos+=2;

            //toes joint
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            if (asymetrics_legs){
               cur_pos+=2;
            }
            cp[cp_idx].kp=original_val[cur_pos]*input[cur_pos];
            cp[cp_idx++].kd=original_val[cur_pos+1]*input[cur_pos+1];
            cur_pos+=2;

            if (cp_idx!=(int)cp.size()){
                exit(25684);
            }
            if (cur_pos!=(int)original_val.size()){
                exit(25685);
            }
        }else{
            for (int j = 0; j < (int)cp.size(); ++j){
                cp[j].kp=original_val[cur_pos]*input[cur_pos++];
                cp[j].kd=original_val[cur_pos]*input[cur_pos++];
            }
        }



        //we set the save configurations
        //we switch the save folder
        std::string primary_save_config = "controllers/bipV2/primary_save_config.txt";
        std::string secondary_save_config = "controllers/bipV2/learning_files_names.txt";


        std::ostringstream oss;
        oss<<"results_opti_gains_inter";
        std::string evo_folder= oss.str();

        oss.clear();
        oss.str("");
        oss << Globals::data_folder_path;
        oss << primary_save_config;

        std::ofstream myfile1(oss.str());
        if (myfile1.is_open())
        {
            myfile1 << evo_folder << "/inter_" << m_evaluationCounter << "_" <<
                       last_eval << "_state.rs" << std::endl;
            myfile1 << evo_folder << "/inter_" <<  m_evaluationCounter << "_" <<
                       last_eval  << ".sbc" << std::endl;
        }
        myfile1.close();

        //we switch to the primary save file
        Globals::primary_save_config = primary_save_config;


        //and we save the structure
        con->save(true, false);


        //and go back t the secondary save file
        Globals::primary_save_config = secondary_save_config;


        //clean the memory
        delete con;
    }

    if (last_eval<=0.0){
        last_eval=0.0;
    }
    return last_eval;
}
//*/




// Implementation of the CMA-ES
#include <shark/Algorithms/DirectSearch/CMA.h>
//acces to sphere
#include <shark/ObjectiveFunctions/Benchmarks/Sphere.h>
#include "Core\SimBiConFramework.h"


double cma_program(std::string save_folder_name, std::string solution_folder, int optimisation_type, int nb_iter_max, int cma_number) {

    // Adjust the floating-point format to scientific and increase output precision.
    std::cout.setf(std::ios_base::scientific);
    std::cout.precision(10);
    //*


    std::string result_controller_path_and_name;
    {
        int a=1.0f/SimGlobals::dt;

        std::ostringstream oss_base_folder;
        oss_base_folder<<Globals::data_folder_path << "controllers/bipV2/";
        std::string controllers_folder=oss_base_folder.str();

        std::string solution_folder_path=get_folder_path(solution_folder,7,"/",controllers_folder);
        std::ostringstream oss_complete_path;
        oss_complete_path<< solution_folder_path << a <<"_"<<cma_number;
        result_controller_path_and_name=oss_complete_path.str();
        std::cout<<result_controller_path_and_name<<std::endl;

        /*
        //test process
        std::ostringstream oss_file_path;
        oss_file_path<<controllers_folder<<result_controller_path_and_name<<".sbc";
        std::ofstream myfile2(oss_file_path.str());
        if (myfile2.is_open())
        {
            myfile2 << result_controller_path_and_name<<"_state.rs" << std::endl;
            myfile2 << result_controller_path_and_name<< ".sbc" << std::endl;
        }else{
            std::cout<<"failed opening the file"<<oss_file_path.str();
        }
        myfile2.close();
        return 0;
        //*/
    }

    //we create the objective function
    std::ostringstream oss;
    oss << "bipV2/" << save_folder_name << "/learning_walk_waterlvl" << SimGlobals::water_level << ".sbc";
    std::string save_line = oss.str();


    //we set the save configurations
    //we switch the save folder
    std::string primary_save_config = "controllers/bipV2/primary_save_config.txt";
    std::string secondary_save_config = "controllers/bipV2/learning_files_names.txt";


    oss.clear();
    oss.str("");
    oss << Globals::data_folder_path;
    oss << primary_save_config;

    std::ofstream myfile1(oss.str());
    if (myfile1.is_open())
    {
        //        if (Globals::evolution_type==0){
        myfile1 << save_folder_name << "/learning_walk_waterlvl" << SimGlobals::water_level << "_state.rs" << std::endl;
        myfile1 << save_folder_name << "/learning_walk_waterlvl" << SimGlobals::water_level << ".sbc" << std::endl;
        //        }
    }
    myfile1.close();

    //*
    std::stringstream oss2;
    oss2 << Globals::data_folder_path;
    oss2 << secondary_save_config;
    std::ofstream myfile2(oss2.str());
    if (myfile2.is_open())
    {
        myfile2 << "learning_walk_state.rs" << std::endl;
        myfile2 << "learning_walk.sbc" << std::endl;
    }
    myfile2.close();
    //I don't want the secondary save in that program (just needed to fill that file for later use)
    //so I disactivate the secondary save
    Globals::primary_save_config = secondary_save_config;
    Globals::secondary_save_config = "";
    //*/



    // Initialize the optimizer for the objective function instance.
    shark::CMA cma;
    shark::SingleObjectiveFunction* objective_func;
    if (optimisation_type==0){
        objective_func= new SimbiconOnjectiveFunction() ;
    }else if (optimisation_type==1){
        objective_func= new SimbiconOnjectiveFunctionGains(true,cma_number,solution_folder) ;
    }

    cma.init(*objective_func);//the parameter here is the error function
    cma.setSigma(0.05); // Explicitely set initial globael step size.

    int nbr_iter = 0;
    // Iterate the optimizer until a solution of sufficient quality is found.


    //generate the path to the folder for the result





    double cur_val=10E40;
    bool first_time = true;
    //int cur_save_trigger = 10;
    do {

        nbr_iter++;
        if (nbr_iter > nb_iter_max){ break; }

        //this is used to update the starting pos (so it fit the movement better)
        /*
        if (nbr_iter >= cur_save_trigger){
            cur_save_trigger += 10;
            Globals::save_mode_controller=save_line;
            double result=evaluate_solution(true);

            std::ostringstream oss_print;
            oss<< "save result: " <<result;
            std::cout<< oss_print.str();

            //we finished saving and resume normal  running
            Globals::save_mode_controller="";

        }
        //*/

        //evolve the parameters
        cma.step(*objective_func);


        // Report information on the optimizer state and the current solution to the console.
        std::ostringstream oss_step;
        oss_step<< nbr_iter << " :: " << objective_func->evaluationCounter() << " :: " <<
                   cma.solution().value << " :: " << cma.sigma()<<std::endl;
        std::cout << oss_step.str();
        if (first_time){
            first_time = false;
        }
        else{
            if (cur_val < cma.solution().value){
                //if our new solution ain't better we don't save if (but still continue from it)
                continue;
            }
        }

        //save the current best solution
        cur_val = cma.solution().value;

        //we save the last solution in a file
        shark::RealVector result = cma.solution().point;

        SimBiConFramework* con;

        if (optimisation_type==0){
            SimbiconOnjectiveFunction* objective_func_local=dynamic_cast<SimbiconOnjectiveFunction*>(objective_func);

            //now I need to save the parameters back in the file so that the next eecution will see them
            con = new SimBiConFramework(objective_func_local->inputFile, NULL);

            //so we push the values in a structure
            int cur_pos = 0;
            for (int k = 0; k < (int)objective_func_local->vect_traj_name.size(); ++k){
                Trajectory* cur_traj = con->getController()->getState()->getTrajectory(objective_func_local->vect_traj_name[k].c_str());
                for (int j = 0; j < (int)cur_traj->components.size(); ++j){
                    TrajectoryComponent* traj_comp = cur_traj->components[j];
                    for (int i = 0; i < (int)traj_comp->baseTraj.getKnotCount(); ++i){
                        traj_comp->baseTraj.setKnotValue(i, result[cur_pos]);
                        ++cur_pos;
                    }
                }
            }

            //I I detect the ipm alte I add it
            if (cur_pos < result.size()){
                SimGlobals::ipm_alteration_effectiveness = result[cur_pos];
                //limit it to 1 (since I want it to go down )
                if (SimGlobals::ipm_alteration_effectiveness > 1){
                    SimGlobals::ipm_alteration_effectiveness = 1;
                }
                ++cur_pos;
            }

            //*
            //also I'll prevent the system from doing the ondulations with the pelvis
            Trajectory* cur_traj = con->getController()->getState()->getTrajectory(objective_func_local->vect_traj_name[0].c_str());
            for (int j = 0; j < (int)cur_traj->components.size(); ++j){
                TrajectoryComponent* traj_comp = cur_traj->components[j];
                for (int i = 0; i < (int)traj_comp->baseTraj.getKnotCount(); ++i){
                    traj_comp->baseTraj.setKnotValue(i, std::fmin(0.1, traj_comp->baseTraj.getKnotValue(i)));
                }
            }
            //*/
            //*
            //I only wanna learn the x component of the walk so I'l override the others with the value I had
            cur_traj = con->getController()->getState()->getTrajectory(objective_func_local->vect_traj_name[5].c_str());
            for (int j = 0; j < (int)cur_traj->components.size(); ++j){
                TrajectoryComponent* traj_comp = cur_traj->components[0];
                traj_comp->baseTraj.setKnotValue(0, 0.000340);
                traj_comp->baseTraj.setKnotValue(1, -0.100323);
                traj_comp->baseTraj.setKnotValue(2, -0.001158);

                traj_comp = cur_traj->components[2];
                traj_comp->baseTraj.setKnotValue(0, 0.0);
                traj_comp->baseTraj.setKnotValue(1, 0.015874);
                traj_comp->baseTraj.setKnotValue(2, 0.0);
            }
        }else if (optimisation_type==1){
            SimbiconOnjectiveFunctionGains* objective_func_local=dynamic_cast<SimbiconOnjectiveFunctionGains*>(objective_func);

            //now I need to save the parameters back in the file so that the next eecution will see them
            con = new SimBiConFramework(objective_func_local->inputFile, NULL);

            //so we push the values in a structure
            std::vector<ControlParams>& cp=con->getController()->get_control_params();
            ControlParams& cp_root=con->getController()->get_root_control_params();

            //load the gains
            const std::vector<double>& original_val=objective_func_local->original_val;
            int cur_pos=0;
            cp_root.kp=original_val[cur_pos]*result[cur_pos++];
            cp_root.kd=original_val[cur_pos]*result[cur_pos++];

            if (objective_func_local->use_symetry){
                int cp_idx=0;
                //pelvis torso
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                cur_pos+=2;

                //hips
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                if (objective_func_local->use_symetry){
                   cur_pos+=2;
                }
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                cur_pos+=2;

                //torso head
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                cur_pos+=2;

                //shoulder
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                cur_pos+=2;

                //knee
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                if (objective_func_local->use_symetry){
                    cur_pos+=2;
                 }
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                cur_pos+=2;

                //elbow
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                cur_pos+=2;

                //ankle
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                if (objective_func_local->use_symetry){
                   cur_pos+=2;
                }
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                cur_pos+=2;

                //toes joint
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                if (objective_func_local->use_symetry){
                   cur_pos+=2;
                }
                cp[cp_idx].kp=original_val[cur_pos]*result[cur_pos];
                cp[cp_idx++].kd=original_val[cur_pos+1]*result[cur_pos+1];
                cur_pos+=2;

                if (cp_idx!=(int)cp.size()){
                    exit(25689);
                }
                if (cur_pos!=(int)original_val.size()){
                    exit(25690);
                }
            }else{
                for (int j = 0; j < (int)cp.size(); ++j){
                    cp[j].kp=original_val[cur_pos]*result[cur_pos++];
                    cp[j].kd=original_val[cur_pos]*result[cur_pos++];
                }
            }

            oss.clear();
            oss.str("");
            oss << Globals::data_folder_path;
            oss << primary_save_config;

            std::ofstream myfile1(oss.str());
            if (myfile1.is_open())
            {
                myfile1 << save_folder_name << "/result_" << objective_func->evaluationCounter() << "_" <<
                           cma.solution().value << "_state.rs" << std::endl;
                myfile1 << save_folder_name << "/result_" <<  objective_func->evaluationCounter() << "_" <<
                           cma.solution().value  << ".sbc" << std::endl;
            }else{
                std::cout<<"failed opening the file"<<oss.str();
            }
            myfile1.close();

            //we switch to the primary save file
            Globals::primary_save_config = primary_save_config;

            //and we save the structure
            if (save_intermediary_results){
                con->save(true, false);
            }


            //now we will do the save in the folder that only keep the best solution
            oss.clear();
            oss.str("");
            oss << Globals::data_folder_path;
            oss << primary_save_config;

            std::ofstream myfile2(oss.str());
            if (myfile2.is_open())
            {
                myfile2 << result_controller_path_and_name<<"_state.rs" << std::endl;
                myfile2 << result_controller_path_and_name<< ".sbc" << std::endl;
            }else{
                std::cout<<"failed opening the file"<<oss.str();
            }
            myfile2.close();
        }

        //we switch to the primary save file
        Globals::primary_save_config = primary_save_config;


        //and we save the structure
        con->save(true, false);

        /*
        std::ostringstream oss30;
        oss30<<"!!!!!!!!!!!!!!SIVING RESULT start !!!!!!!!!!!!!!!"<<std::endl;
        oss30<<"result saved: "<<cur_val<<std::endl;


        const std::vector<ControlParams>& cp=con->getController()->get_control_params();
        const ControlParams& root_param=con->getController()->get_root_control_params();

        oss30<<"kp kd: "<<root_param.kp<<"   "<<root_param.kd<<std::endl;

        for (int i=0;i<cp.size();++i){
            oss30<<"kp kd: "<<cp[i].kp<<"    "<<cp[i].kd<<std::endl;
        }
        oss30<<"!!!!!!!!!!!!!!SIVING RESULT end !!!!!!!!!!!!!!!"<<std::endl;
        std::cout<<oss30.str();
        //*/

        //and go back t the secondary save file
        Globals::primary_save_config = secondary_save_config;


        delete con;



    } while (cma.solution().value > 1E-20);
    //*/


    delete objective_func;

    //std::cin.clear();
    //std::cin.ignore();

    //int a = 2;
    //std::cin >> a;

    return cur_val;
}
